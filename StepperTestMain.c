// StepperTestMain.c
// Runs on TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 *///

// PD3 connected to driver for stepper motor coil A/In1
// PD2 connected to driver for stepper motor coil A'/In2
// PD1 connected to driver for stepper motor coil B/In3
// PD0 connected to driver for stepper motor coil B'/In4

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "stepper.h" // This header file includes controls for the stepper motor
#include "systick.h" // This file includes the functions that intitialize the systick timer

#define T1ms 16000    // assumes using 16 MHz Clock 
#define Turn90 500 //135

// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
#define GREEN 	0x8
#define RED 		0x2
#define BLUE 		0x4
#define YELLOW  0xA



extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt
#define IRSENSOR   (*((volatile uint32_t *)0x40024004)) // PORT E, pin 0	

void PortF_Init(void);
void ToggleRedLed(int toggle);
void Delay (float i);
void IRSensor_Init(void);

unsigned int open;
unsigned int close;


int main(void){
	unsigned int i=0;
	unsigned long In;
	PortF_Init();
  Stepper_Init();
	IRSensor_Init();
	EnableInterrupts();
	GPIO_PORTF_DATA_R |= GREEN;

  while(1){

		while(!open)
		{
			WaitForInterrupt();
		}
	

		//Rotates clockwise to open the gate
		// turn counter clockwise 90 degrees	

				GPIO_PORTF_DATA_R &= ~0xE;
			
			for (i=0;i<Turn90; i++) {
				Stepper_CCW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
				if (i%50 == 0)
					GPIO_PORTF_DATA_R ^= RED;    // LED is RED - Toggles every 500ms
			}
				GPIO_PORTF_DATA_R &= ~RED;    // LED is RED - Red LED off
						GPIO_PORTF_DATA_R |= GREEN;

		while(!close){}
		//Rotates counter-clockwise to open the gate
		// turn counter counter-clockwise 90 degrees	

			GPIO_PORTF_DATA_R &= ~0xE;

			for (i=0;i<Turn90; i++) {
					Stepper_CW(10*T1ms);   // output every 
				if (i%50 == 0)
					GPIO_PORTF_DATA_R ^= RED;    // LED is RED	
				}
			GPIO_PORTF_DATA_R &= ~RED;    // LED is RED
			GPIO_PORTF_DATA_R |= BLUE;

			close = 0;
		
  }
	
}


void PortF_Init(void){ 
  SYSCTL_RCGC2_R |= 0x00000020;     	// activate F clock
	while ((SYSCTL_RCGC2_R&0x00000020)!=0x00000020){} // wait for the clock to be ready
		
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   	// unlock PortF PF0  
	GPIO_PORTF_CR_R |= 0x1F;         		// allow changes to PF4-0 :11111->0x1F     
  GPIO_PORTF_AMSEL_R &= ~0x1F;        // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; 	// GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~0x11;          // PF4,PF0 input   
  GPIO_PORTF_DIR_R |= 0x0E;          	// PF3,PF2,PF1 output   
	GPIO_PORTF_AFSEL_R &= ~0x1F;        // no alternate function
  GPIO_PORTF_PUR_R |= 0x11;          	// enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R |= 0x1F;          	// enable digital pins PF4-PF0        
	
	GPIO_PORTF_IS_R &=~0x11;																// PF0-1 is level sensitive
	GPIO_PORTF_IBE_R |= 0x11;
	GPIO_PORTF_ICR_R = 0x11;																// clear flag
	GPIO_PORTF_IM_R |= 0x11;																// arm interrupt on PF0-1
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFFF1FFFFF) | 0x00080000;	// priority 1
	NVIC_EN0_R = 0x40000000;																// enable F port interrupt (bit 30) in NVIC	
	open=0;
	close=1;
}

void GPIOPortF_Handler(void) {
		for (uint32_t time = 727240*100/91; time>0;time--){}
		GPIO_PORTF_ICR_R = 0x11; // clear flag
		open=1;
}

void GPIOPortE_Handler(void){
	for(uint32_t time = 727240*100/91; time>0; time--){}
	if((open==0)||(GPIO_PORTE_DATA_R&0x01)==0x00){
		GPIO_PORTE_ICR_R |= 0x01;
	  open = 1;
	}
	if((open==1)&&(GPIO_PORTE_DATA_R&0x01)==0x01)
	{
		GPIO_PORTE_ICR_R |= 0x01;
		open=0;
	  close = 1;
	}
}


void IRSensor_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;                              // Activate Port E clocks
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOE)!=SYSCTL_RCGC2_GPIOE){}  // wait for clock to be active
		
	GPIO_PORTE_AMSEL_R &= ~0x01;                       // Disable analog function on PE0
  GPIO_PORTE_PCTL_R &= ~0x000000FF;                  // Enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x01;                         // Inputs on PE0
  GPIO_PORTE_AFSEL_R &= 0x01;                        // Regular function on PE0
  GPIO_PORTE_DEN_R |= 0x01;                          // Enable digital signals on PE0
	GPIO_PORTE_PUR_R |= 0x01;                          // enable weak pull-up on PE0

		
	GPIO_PORTE_IS_R &= ~0x01;                          // (d) PE0 is edge-sensitive
	GPIO_PORTE_IBE_R |= 0x01;                          //     PE0 is both edges
	GPIO_PORTE_ICR_R |= 0x01;                          // (e) clear flag 1
	GPIO_PORTE_IM_R |= 0x01;                           // (f) arm interrupt on PE0
		
	NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF1F)|0x00000060; // (g) priority 3
	NVIC_EN0_R |= 0x00000010;                          // (h) enable inturrupt 4 in NVIC

}
