
  
/*
 * i changed double to float for temp calculation and eliminated the timer also for temp calculation
 * changed the menu, added row and deleted the '#' should not need it, so also deleted stuff in writeUart accordingly
 * changed DAC SSs might have a problem with these
 *
 * Must change digital to initialize all ports 
 * Must change DAC stuff for different IC 
 *
 * Using Termite Terminal
 * USing SAME54P20A mcu on XPlained Pro developement board
 *
 * Created: 10/15/2018 11:49:12 AM
 * Author : bryant
 */ 

#include "sam.h"
#define DIR PORT_PB27;
#define PLS PORT_PB28;

/* Prototypes */
void clock_setup(void);
void port_setup(void);
void wait(volatile int d);
void motor_EIC_setup(void);
void check(void);
void terminal_UART_setup(void);	//USART
void write_terminal(char *a);
void convert(int a);


volatile int cw = 0;	
volatile int ccw = 0;	
volatile int eic_counter;
volatile int pls_counter = 0;
volatile int correct;
volatile int state = 0;
volatile int state1 = 0;
volatile char convert_array[4];
volatile char *convert_array_ptr;
volatile int Z = 0;

int main(void){

	/* Initializing functions*/
    SystemInit();	
	clock_setup();
	port_setup();
	motor_EIC_setup();
	terminal_UART_setup();
	
	volatile char startArr[] = "Start\n";
	volatile char *startPtr;
	startPtr = startArr;
	write_terminal(startPtr);
	
	volatile int initial = 0;
	
	convert_array_ptr = convert_array;
	//terminal_input_array_ptr = terminal_input_array;

	Port *por = PORT;
	PortGroup *porA = &(por->Group[0]);
	PortGroup *porB = &(por->Group[1]);

	//porB->OUTCLR.reg = PLS;
	
	/* Polling loop looking for Terminal request */
	while(1){
		switch(state1){
			case 0 :
				if(Z){
					state1 = 1;
					Z = 0;
					eic_counter = 0;
					wait(10);
					break;
				}
				else{
					switch(state){
						case 0:	//idle
						porB->OUTCLR.reg = PLS;
						state = 1;
						break;
						case 1: //move motor
						porB->OUTSET.reg = PLS;
						state = 0;
						break;
						default:
						state = 0;
					}
					wait(1);
					break;				
				}
			
			case 1: 
				if(Z){
					state1 = 2;
					Z = 0;
					break;
				}
				else{
					switch(state){
						case 0:	//idle
						porB->OUTCLR.reg = PLS;
						state = 1;
						break;
						case 1: //move motor
						porB->OUTSET.reg = PLS;
						pls_counter++;
						state = 0;
						break;
						default:
						state = 0;
					}
					wait(1);
					break;
				}
			case 2:
				//convert(pls_counter);
				convert(eic_counter);
				pls_counter = 0;
				state1 = 0;
				Z = 0;
				break;
			default: state1 = 0;
			break;
		}
		

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

	//PORTs
	porB->DIRCLR.reg = PORT_PB26; //phase B
	porB->PINCFG[26].bit.INEN = 1;
	
	//!DIR=CCW
	porB->DIRSET.reg = DIR;
	porB->DIRSET.reg = PLS;
	porB->OUTCLR.reg = DIR;
	porB->OUTCLR.reg = PLS;
	
	/* EIC Z */
	porA->PMUX[3].bit.PMUXE = 0;	//PA06 EXTINT[6]
	porA->PINCFG[6].bit.PMUXEN = 1;
	
	/* EIC Phase A */
	porB->PMUX[2].bit.PMUXE = 0;	//PB04 EXTINT[4]
	porB->PINCFG[4].bit.PMUXEN = 1;
}

/* EIC for encoder Z */
void motor_EIC_setup(void){
	EIC->CTRLA.reg = 0;
	while(EIC->SYNCBUSY.reg){}
	EIC->CTRLA.bit.CKSEL = 0;	//EIC is clocked by GCLK
	EIC->INTENSET.reg |= 1<<6 | 1<<4;
	EIC->ASYNCH.reg |= 1<<6 | 1<<4;	//asynchronous mode
	EIC->CONFIG[0].bit.SENSE6 = 2;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CONFIG[0].bit.SENSE4 = 2;	//4=high level detection 1=rising edge 2= falling edge
	EIC->CTRLA.reg = 1<<1;	//enable
	while(EIC->SYNCBUSY.reg){}
	NVIC->ISER[0] |= 1<<18 | 1<<16;	//enable the NVIC handler
}

/* EIC handler for encoder */
void EIC_6_Handler(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	EIC->INTFLAG.reg = 1<<6;	//clear int flag
	//convert(pls_counter);
	Z = 1;
	//if(eic_counter==200){
		//convert((pls_counter));
		//eic_counter = 0;
		//pls_counter = 0;
	//}
	
	//if(porB->IN.reg & PORT_PB26){
		//cw = 1;
		//ccw = 0;
	//}
	//else{
		//cw = 0;
		//ccw = 1;
	//}
	
}

/* EIC handler for encoder */
void EIC_4_Handler(void){
	EIC->INTFLAG.reg = 1<<4;	//clear int flag
	eic_counter++;
}

void wait(volatile int d){
	int count = 0;
	while (count < d*1000){
		count++;
	}
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