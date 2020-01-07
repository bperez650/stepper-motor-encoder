/**********************************************
* motor control for oriental motor with encoder
* open collector using 3.3V
* 400ct/rev encoder using 800 microstep/rev
* using interrrupt on phase A to confirm movement
* Not checking phase B for direction
* Using Z output for homing
* has acceleration
***********************************************/

  

#include "sam.h"
#define DIR PORT_PB27;	//controls direction of motor
#define PLS PORT_PB28;	// pull down to microstep motor

/* Prototypes */
void clock_setup(void);
void port_setup(void);
void wait(volatile int d);
void accel_wait(volatile int d);
void motor_EIC_setup(void);
void terminal_UART_setup(void);	
void write_terminal(char *a);
void convert(int a);
void home(void);
void change_filter(int a);

volatile int eic_counter;	//counts EIC pulse on phase A
volatile char convert_array[4];
volatile char *convert_array_ptr;
volatile int Z = 0;	//tracks Z output using EIC
volatile int speed = 1000;	//length of wait for acceleration
volatile int current_filter = 0;	//tracks current filter position


int main(void){

    SystemInit();	
	clock_setup();
	port_setup();
	motor_EIC_setup();
	terminal_UART_setup();
	home();
	wait(10);
	change_filter(5);
	wait(10);
	change_filter(1);
	
	volatile char startArr[] = "Start\n";
	volatile char *startPtr;
	startPtr = startArr;
	write_terminal(startPtr);	
	convert_array_ptr = convert_array;

	Port *por = PORT;
	PortGroup *porA = &(por->Group[0]);
	PortGroup *porB = &(por->Group[1]);


	
	while(1){

	}
}
/* CLock source is 12MHz divided to 1MHz */
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
	
	GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC_XOSC1 | GCLK_GENCTRL_RUNSTDBY | !(GCLK_GENCTRL_DIVSEL) | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(12);	//divide by 12 = 1MHz
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
	
	porA->PMUX[2].bit.PMUXE = 3;	//PA04 pad0 Tx
	porA->PINCFG[4].bit.PMUXEN = 1;	//PA05 pad1 Rx
	porA->PMUX[2].bit.PMUXO = 3;
	porA->PINCFG[5].bit.PMUXEN = 1;
	
	//ports for controlling motor
	porB->DIRSET.reg = DIR;	//direction SET==CCW
	porB->DIRSET.reg = PLS;	//pulse for microstepping motor
	porB->OUTSET.reg = DIR;	//CCW
	porB->OUTSET.reg = PLS;
	
	/* EIC Z output*/
	porA->PMUX[3].bit.PMUXE = 0;	//PA06 EXTINT[6]
	porA->PINCFG[6].bit.PMUXEN = 1;
	
	/* EIC Phase A output*/
	porB->PMUX[2].bit.PMUXE = 0;	//PB04 EXTINT[4]
	porB->PINCFG[4].bit.PMUXEN = 1;
}

/* EIC for phase A and Z output */
void motor_EIC_setup(void){
	EIC->CTRLA.reg = 0;
	while(EIC->SYNCBUSY.reg){}
	EIC->CTRLA.bit.CKSEL = 0;	//EIC is clocked by GCLK
	EIC->INTENSET.reg |= 1<<6 | 1<<4;
	EIC->ASYNCH.reg |= 1<<6 | 1<<4;	//asynchronous mode
	EIC->CONFIG[0].bit.SENSE6 = 1;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CONFIG[0].bit.SENSE4 = 1;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CTRLA.reg |= 1<<1;	//enable
	while(EIC->SYNCBUSY.reg){}
	NVIC->ISER[0] |= 1<<18 | 1<<16;	//enable the NVIC handler
}

/* EIC handler for Z output 
*  sets var Z high when Z output detected
*  only used for homing   */
void EIC_6_Handler(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	EIC->INTFLAG.reg |= 1<<6;	//clear int flag
	Z = 1;
}

/* EIC handler for phase A 
*  counts the numnber of phase A pulses
*  not checking direction */
void EIC_4_Handler(void){
	EIC->INTFLAG.reg |= 1<<4;	//clear int flag
	eic_counter++;
}

/* controls sped of motor during homeing
*  not used in acceleration */
void wait(volatile int d){
	int count = 0;
	while (count < d*1000){
		count++;}
}

void terminal_UART_setup(void){
	Sercom *ser = SERCOM0;
	SercomUsart *uart = &(ser->USART);
	uart->CTRLA.reg = 0;	//enable protected regs
	while(uart->SYNCBUSY.reg){}
	uart->CTRLA.bit.DORD = 1;	//LSB transferred first
	uart->CTRLA.bit.CMODE = 0;	//asynchronous mode
	uart->CTRLA.bit.SAMPR = 0;	//16x oversampling using arithmetic
	uart->CTRLA.bit.RXPO = 1;	//RX is pad1 PA05
	uart->CTRLA.bit.TXPO = 2;	//TX is pad0 PA04
	uart->CTRLA.bit.MODE = 1;	//uart with internal clock
	uart->CTRLB.bit.RXEN = 1;	//enable RX
	uart->CTRLB.bit.TXEN = 1;	//enable TX
	uart->CTRLB.bit.PMODE = 0;	//even parity mode
	uart->CTRLB.bit.SBMODE = 0;	//1 stop bit
	uart->CTRLB.bit.CHSIZE = 0;	//8bit char size
	while(uart->SYNCBUSY.reg){}
	uart->BAUD.reg = 55470;	//for fbaud 9600 at 1Mhz fref
	uart->INTENSET.bit.RXC = 1;	//receive complete interr
	NVIC->ISER[1] |= 1<<16;	//enable sercom0 RXC int
	uart->CTRLA.reg |= 1<<1;	//enable
	while(uart->SYNCBUSY.reg){}
}

void write_terminal(char *a){
	Sercom *ser = SERCOM0;
	SercomUsart *uart = &(ser->USART);
		while(*a){
			while(!(uart->INTFLAG.bit.DRE)){}
			uart->DATA.reg = *a++;
			while((uart->INTFLAG.bit.TXC)==0){}	
		}
		uart->DATA.reg = 10;
		//wait(100);
}

void convert(int a){
	int i = 100;   //divisor
	int j = 0;  //array counter
	int m = 1;  //counter
	int n = 100;    //increment to divisor

	while(j <= 3){
		int b = a % i;
		if(b == a) {
			int p = (m-1);
			switch(p) {
				case 0:
				convert_array[j++] = '0';
				break;
				case 1:
				convert_array[j++] = '1';
				break;
				case 2:
				convert_array[j++] = '2';
				break;
				case 3:
				convert_array[j++] = '3';
				break;
				case 4:
				convert_array[j++] = '4';
				break;
				case 5:
				convert_array[j++] = '5';
				break;
				case 6:
				convert_array[j++] = '6';
				break;
				case 7:
				convert_array[j++] = '7';
				break;
				case 8:
				convert_array[j++] = '8';
				break;
				case 9:
				convert_array[j++] = '9';
				break;
				default:
				convert_array[j++] = 'A';
				break;
			}
			a = a - (n*(m-1));
			m = 1;

			if(j == 1){
				i = 10;
				n = 10;
			}
			if(j == 2){
				i = 1;
				n = 1;
			}
			
		}
		else{
			m++;
			i = i + n;
		}
	}
	convert_array[3] = 0;	//force pointer to end here
	write_terminal(convert_array_ptr);
}

/* Funciton for Homing motor
*  turns CCW until Z output is detected
*  then turns CW to filter 0 */
void home(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	volatile int motor_state = 0;	//state for stepping state machine
	Z = 0;
	
	while(!Z){
		/* First state machine is going CCW looking for Z output
		*  microstepping state machine state0 pulls down and cause motor to step
		*  state1 pulls up and motor doesnt move */
		switch(motor_state){	
			case 0:	//pull down microstep
				porB->OUTCLR.reg = PLS;
				motor_state = 1;
				wait(1);
				break;
			case 1: //pull up 
				porB->OUTSET.reg = PLS;
				motor_state = 0;
				wait(1);
				break;
			default:
				motor_state = 0;
				break;
		}
	}
	eic_counter = 0;	//reset eic_counter 
	porB->OUTCLR.reg = DIR;	//change dir to CW
	/* Seocnd state machine is going CW to set position, filter 0
	* position of filter 0 is (encoder cts/rev / # of filters / 2) == (400/6/2 = 33.33)
	*  microstepping state machine state0 pulls down and cause motor to step
	*  state1 pulls up and motor doesnt move */
	while(eic_counter < 34){	
		switch(motor_state){
			case 0:	//pull down microstep
				porB->OUTCLR.reg = PLS;
				motor_state = 1;
				wait(1);
				break;
			case 1: //pull up
				porB->OUTSET.reg = PLS;
				motor_state = 0;
				wait(1);
				break;
			default:
				motor_state = 0;
				break;
		}
		
	}
}

/* Function for changing filter position
*  takes in desired filter positon and compares to current position to determine direction of steps
*  increasing filter postion = CW   e.g. fitler0 -> filter2
*  acceleration state machine has bounded top and low speed 
*  calls variable accel_wait() to change speed */ 
void change_filter(int a){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	volatile int accel_state = 0;
	volatile int motor_state = 0;
	volatile int temp = 0;
	volatile int steps;
	eic_counter = 0;
	
	/* determine direction of steps by comparing current and requested filter position */
	if(a > current_filter){
		porB->OUTCLR.reg = DIR;	//CW
		steps = (a-current_filter)*67;
	}	
	else{
		porB->OUTSET.reg = DIR;	//CCW
		steps = (current_filter-a)*67;
	}	
	/* Acceleration state machine*/ 
	while(eic_counter < (steps+1)){
		switch(accel_state){
			case 0:	//slow start
				if(eic_counter>=10){
					accel_state = 1;}
				else{
					accel_state = 0;}
				break;
			case 1:	//accel
				if(eic_counter>=(steps/2))
					{accel_state = 3;}
				else if(speed<=100){
					accel_state = 2;
					temp = eic_counter-10;}
				else{
					accel_state = 1;
					speed = speed-10;}
				break; 				
			case 2:	//fast
				if(eic_counter>=(steps-10-temp)){
					accel_state = 3;}
				else{
					accel_state = 2;}
				break;
			case 3:	//decel
				if(speed>=1000 || ((steps-eic_counter)<=10)){
					accel_state = 4;}
				else{
					accel_state = 3;
					speed = speed+10;}
				break;
			case 4:	//slow finish
				break;
			default :
				break;	
		}
	
		/*  microstepping state machine state0 pulls down and cause motor to step
		*  state1 pulls up and motor doesnt move */
		switch(motor_state){
			case 0:	//pull down microstep
				porB->OUTCLR.reg = PLS;
				motor_state = 1;
				break;
			case 1: //pull up
				porB->OUTSET.reg = PLS;
				motor_state = 0;
				break;
			default:
				motor_state = 0;
		}
		/* call wait function to change speed */ 
		accel_wait(speed); 
	}
	current_filter = a;// stores current filter positon 
}

/* Tuneable wait function for changing speed of motor
*  input pararm is desired speed */
void accel_wait(volatile int d){
	int count = 0;
	while (count < d){
		count++;}
}
