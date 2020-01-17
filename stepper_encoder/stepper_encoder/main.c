/*
* Motor control program for oriental motor with encoder on filter wheel in LII
* Proram looks at encoder falling and risign edges to ensure proper position
* to deal with the handler storm clock is run at 12 Mhz
* Not completley robust but will make up for minor lost steps
* If more than 10 steps are lost then a flag is raised
* program considers Z output for homing 
* Z output as well as phase A edges are detected through EICs
* In phase A EIC handler, phase B is considered to determine step direction
* Note a lot of gitter is seen in final step with the filter wheel mass
* to mitigate this settings on driver board are as follows:
* steps resolution is 400 steps/rev
* run current is set "E"
* filtering is on
* holding current set ot max
* current setup is using NMOS to level shift to 5V pulses going to motor driver board which inverts mcu pulse outputs
* when using level shifters I will need ot invert outputs
*/



#include "sam.h"
#define DIR		PORT_PB27;	//controls direction of motor
#define PLS		PORT_PB28;	// pull down to microstep motor
#define PB      PORT_PB26	// phase B of encoder 
#define cw		1
#define ccw		0
 
/* Prototypes */
void clock_setup(void);
void port_setup(void);
void wait(volatile int d);
void accel_wait(volatile int d);
void motor_EIC_setup(void);
void home(void);
void change_filter(int a);

/* Global Variables */
volatile int eic_counter;	//counts EIC pulse on phase A
volatile int Z = 0;	//tracks Z output using EIC
volatile int speed = 100;	//length of wait for acceleration
volatile int current_filter = 0;	//tracks current filter position
//volatile const int SPEED_ARRAY[] = {1000,962,924,886,848,810,772,734,696,658,620,582,544,506,468,430,393,354,316,278,240,202,164,126,88};	//linearly decreasing speed for acceleration
volatile const int SPEED_ARRAY[] = {1000,940,879,820,761,703,646,591,537,485,435,388,342,300,260,223,189,158,131,107,86,70,57,57,57};	//sinusoidal change in speed array for acceleration with lower end limit 
//volatile const int SPEED_ARRAY[] = {1000,940,879,820,761,703,646,591,537,485,435,388,342,300,260,223,189,158,131,107,86,70,57,47,41};	//sinusoidal change in speed array for acceleration
volatile int direction  = 0;
volatile int rising_trig = 0;
volatile int falling_trig = 0;
volatile int motor_flag = 0;

int main(void){

    SystemInit();	
	clock_setup();
	port_setup();
	motor_EIC_setup();
	home();
	wait(9);
	
	while(1){
		wait(1);
		change_filter(6);
		wait(1);
		change_filter(0);
	}
}
/* 
* CLock source is 12MHz needed ot deal with EIC storm
*/
void clock_setup(void){
	/* 12MHz crystal on board selected mapped to PB22/PB23 */
	OSCCTRL->XOSCCTRL[1].bit.ENALC = 1;	//enables auto loop ctrl to control amp of osc
	OSCCTRL->XOSCCTRL[1].bit.IMULT = 4;
	OSCCTRL->XOSCCTRL[1].bit.IPTAT = 3;
	OSCCTRL->XOSCCTRL[1].bit.ONDEMAND = 1;
	OSCCTRL->XOSCCTRL[1].bit.RUNSTDBY = 1;
	OSCCTRL->XOSCCTRL[1].bit.XTALEN = 1;	//select ext crystal osc mode
	OSCCTRL->XOSCCTRL[1].bit.ENABLE = 1;
	OSC32KCTRL->OSCULP32K.bit.EN32K = 1;
	
	GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC_XOSC1 | GCLK_GENCTRL_RUNSTDBY | !(GCLK_GENCTRL_DIVSEL) | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(1);	//divide by 1 = 12MHz
	GCLK->GENCTRL[1].reg = GCLK_GENCTRL_SRC_XOSC1 | GCLK_GENCTRL_RUNSTDBY | !(GCLK_GENCTRL_DIVSEL) | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(1);	//divide by 1 = 12MHz
	while(GCLK->SYNCBUSY.reg){}	//wait for sync
	
	GCLK->PCHCTRL[7].bit.CHEN = 0;	//disable for safety first
	GCLK->PCHCTRL[4].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;	//EIC
	GCLK->PCHCTRL[7].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;	//SERCOM0

	MCLK->CPUDIV.reg = 1;	//divide by 1
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;	//unmask EIC
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0;	//unmask sercom0

}

void port_setup(void){
	Port *por = PORT;
	PortGroup *porA = &(por->Group[0]);
	PortGroup *porB = &(por->Group[1]);
	
	//12MHz crystal on board selected mapped to PB22/PB23
	
	//ports for controlling motor
	porB->DIRSET.reg = DIR;	//direction SET==CCW
	porB->DIRSET.reg = PLS;	//pulse for microstepping motor
	porB->OUTCLR.reg = DIR;	//CCW
	porB->OUTCLR.reg = PLS;
	
	/* EIC Z output*/
	porA->PMUX[3].bit.PMUXE = 0;	//PA06 EXTINT[6]
	porA->PINCFG[6].bit.PMUXEN = 1;
	
	/* EIC Phase A output*/
	porB->PMUX[2].bit.PMUXE = 0;	//PB04 EXTINT[4]
	porB->PINCFG[4].bit.PMUXEN = 1;
	
	/* Phase B of encoder input */
	porB->DIRCLR.reg = PB;	//set as input
	porB->PINCFG[26].bit.INEN = 1;
	
	/* EIC Phase A falling edge */
	porB->PMUX[2].bit.PMUXO = 0;	//PB05 EXTINT[5]
	porB->PINCFG[5].bit.PMUXEN = 1;
}

/* 
*  EIC for phase A falling and risng edge and Z output 
*/
void motor_EIC_setup(void){
	EIC->CTRLA.reg = 0;
	while(EIC->SYNCBUSY.reg){}
	EIC->CTRLA.bit.CKSEL = 0;	//EIC is clocked by GCLK
	EIC->INTENSET.reg |= 1<<6 | 1<<4 | 1<<5;
	EIC->ASYNCH.reg |= 1<<6 | 1<<4 | 1<<5;	//asynchronous mode
	EIC->CONFIG[0].bit.SENSE6 = 1;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CONFIG[0].bit.SENSE4 = 1;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CONFIG[0].bit.SENSE5 = 2;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CTRLA.reg |= 1<<1;	//enable
	while(EIC->SYNCBUSY.reg){}
	NVIC->ISER[0] |= 1<<18 | 1<<16 | 1<<17;	//enable the NVIC handler
}

/* 
*  EIC handler for Z output 
*  sets var Z high when Z output detected
*  only used for homing   
*/
void EIC_6_Handler(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	EIC->INTFLAG.reg |= 1<<6;	//clear int flag
	Z = 1;
}

/* 
*  EIC handler for phase A RISING edge
*  checks state of phase B and sets rising_trig var
*/
void EIC_4_Handler(void){
	EIC->INTFLAG.reg |= 1<<4;	//clear int flag
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	if(!(porB->IN.reg & PB))
		{rising_trig = cw;}
	else {rising_trig = ccw;}
}

/* 
*  EIC handler for phase A FALLING edge
*  checks state of phase B 
*  if both rising edge and falling edge are valid then ++/-- eic_counter var
*/
void EIC_5_Handler(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	EIC->INTFLAG.reg |= 1<<5;	//clear int flag
	if(!(porB->IN.reg & PB))
		{falling_trig = ccw;}
	else {falling_trig = cw;}
		
	if(falling_trig && rising_trig)
		{eic_counter++;}
	else if(!falling_trig && !rising_trig)
		{eic_counter--;}
}

/* 
*  controls speed of motor during homing
*  not used in acceleration 
*/
void wait(volatile int d){
	int count = 0;
	while (count < d*1000000){
		count++;}
}

/* 
*  Funciton for Homing motor
*  turns CCW until Z output is detected
*  then turns CW to filter 0 
*/
void home(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	volatile int motor_state = 0;	//state for stepping state machine
	Z = 0;
	
	while(!Z){
		/* Stepping going CCW looking for Z output */
		porB->OUTCLR.reg = PLS;
		wait(20);
		porB->OUTSET.reg = PLS;
		wait(20);
	}
	eic_counter = 0;	//reset eic_counter 
	direction = cw;
	porB->OUTSET.reg = DIR;	//change dir to CW
	/* 
	* Stepping motor CW to set position, filter 0
	* position of filter 0 is (encoder cts/rev / # of filters / 2) == (400/6/2 = 33.33) 
	*/
	while(eic_counter < 34){	
		porB->OUTCLR.reg = PLS;
		wait(20);
		porB->OUTSET.reg = PLS;
		wait(20);		
	}
}

/* 
* Function for changing filter position
*  takes in desired filter positon and compares to current position to determine direction of steps
*  increasing filter postion = CW   e.g. fitler0 -> filter2
*  acceleration state machine has bounded top and low speed 
*  calls variable accel_wait() to change speed 
*/ 
void change_filter(int a){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	volatile int accel_state = 0;	//acceleration state machine state var
	volatile int steps;	//holds total # of steps needed to move to final position
	volatile int stuff;
	eic_counter = 0;	//reset eic_counter
	speed = 2000;	//reset speed
	volatile int pulse_ct = 0;
	
	/* determine direction of steps by comparing current and requested filter position */
	if(a > current_filter){
		porB->OUTSET.reg = DIR;	//CW
		direction = cw;
		steps = (a-current_filter)*67;
	}	
	else{
		porB->OUTCLR.reg = DIR;	//CCW
		direction = ccw;
		steps = (current_filter-a)*67;
	}	
	
	if(!direction){stuff = eic_counter*-1;}
	else{stuff=eic_counter;}
	
	/* Acceleration state machine */
	while(stuff < steps){
		if(!direction){stuff = eic_counter*-1;}
		else{stuff=eic_counter;}
		switch(accel_state){
			case 0:	//accel
					if(stuff>24){	//condition assures that we will have enought time to decel and go into slow state
					accel_state = 1;}
				else{
					accel_state = 0;
					speed = SPEED_ARRAY[stuff];}
					//speed = speed-SPEED_INC;}
					break;
			case 1:	//fast
				if((steps-stuff)<25){
					accel_state = 2;}
				else{
				accel_state = 1;}
						break;
			case 2:	//decel
				if(speed>1980){
					//if(i<0){
					//if((steps-eic_counter)<=SLOW_GAP){	//limits lower speed and ensures we have enough slow time
					accel_state = 3;}
				else{
					accel_state = 2;
					speed = SPEED_ARRAY[steps-stuff];}
					break;
			case 3:	//slow finish
				break;
			default :
				break;
			}
		
		/* stepping motor */
		porB->OUTCLR.reg = PLS;
		accel_wait(speed); 
		porB->OUTSET.reg = PLS;
		accel_wait(speed); 
		pulse_ct++;
	}
	current_filter = a;// stores current filter positon
	
	/* throws flag if motor missed too many steps */ 
	if((pulse_ct-stuff)>10)
		{motor_flag=1;}
}

/* Tuneable wait function for changing speed of motor
*  input pararm is desired speed */
void accel_wait(volatile int d){
	int count = 0;
	while (count < d*20){
		count++;}
}



