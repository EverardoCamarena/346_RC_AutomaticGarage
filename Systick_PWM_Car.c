/////////////////////////////////////////////////////////////////////////////
// Systick_PWM_Car.c
// Example project for using Systick timer to generate PWM signals to control 
// the speed of a DC motor Car
// Description: 
// Hardware connections: assume L298N is used for DC motor driver
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
// By Min He, 4/20/2023
/////////////////////////////////////////////////////////////////////////////

//////////////////////1. Pre-processor Directives Section////////////////////
#include "tm4c123gh6pm.h"
#include "SystickPWM.h"
/////////////////////////////////////////////////////////////////////////////

//////////////////////2. Declarations Section////////////////////////////////

////////// Local Global Variables //////////

////////// Function Prototypes //////////
void Delay(float t);

extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

//////////////////////3. Subroutines Section/////////////////////////////////

#define IRSENSOR   (*((volatile uint32_t *)0x40024004)) // PORT E, pin 0
#define SWITCH     (*((volatile uint32_t *)0x400253FC)) // PF4 PF0 for onboard switches
	
uint8_t IRIndicator = 0;
uint8_t SequenceOne = 0;
uint8_t SequenceTwo = 0;



int main(void){
	uint8_t dir;
//  Motor_Init(MID);  // medium speed: 50% duty cycle
  Motor_Init(SLOW); // slow speed: 30% duty cycle
//  Motor_Init(FAST); // fast speed: 90% duty cycle
	
	DisableInterrupts();
	IRSensor_Init();
	Switch_Init();
	EnableInterrupts();
	
		while(!SequenceOne){}
		SequenceOne = 0;
		
		Motor_Start(FORWARD_STRAIGHT); // go forwards
		Delay(2.5);
		Motor_Stop(); // stop both wheels
		Delay(1);
			
		Motor_Start(FORWARD_LEFT); // turn left
		Delay(1.3);
		Motor_Stop(); // stop both wheels
		Delay(1);
			
		Motor_Start(FORWARD_STRAIGHT); // go forward towards garage
		while ((IRIndicator==0)||(GPIO_PORTE_DATA_R&0x01)==0x01){} // wait here until sensor detected obstacle within range
		Motor_Stop(); // stop both wheels
		Delay(1);
		
		while(!((IRIndicator==0)||(GPIO_PORTE_DATA_R&0x01)==0x01)){} //check to see if obstacle is removed, if removed, move forward for a fixed distance determined by the house
		Delay(4.3);
		Motor_Start(FORWARD_STRAIGHT); // go into garage
		Delay(0.66);
		Motor_Stop(); // stop both wheels
		Delay(1);
		
		while(!SequenceTwo){}
		SequenceTwo = 0;
		
		Motor_Start(BACKWARD_STRAIGHT); // go backwards
		Delay(0.83);
		Motor_Stop(); // stop both wheels
		Delay(1);
			
		Motor_Start(FORWARD_RIGHT); // turn right
		Delay(1.1);
		Motor_Stop(); // stop both wheels
		Delay(3);
		
		Motor_Start(FORWARD_STRAIGHT); // go forwards
		Delay(2);
		Motor_Stop(); // stop both wheels
		Delay(1);
		
		while(1){}
	}

// Subroutine to wait 0.25 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(float t){
	unsigned long volatile time;
  time = (727240*500/250)*t;  // 0.5sec
  while(time){
		time--;
  }
}

void Switch_Init(void){
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;         // activate F clock
  while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){} // wait for the clock to be ready

  GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock PortF
  GPIO_PORTF_CR_R |= 0x1F;                // allow changes to PF4-0 :11111->0x1F
		
  GPIO_PORTF_AMSEL_R &= ~0x1F;                       // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF;                  // GPIO clear bit PCTL
  GPIO_PORTF_DIR_R &= ~0x11;                         // PF4,PF0 input
  GPIO_PORTF_AFSEL_R &= ~0x1F;                       // no alternate function
  GPIO_PORTF_DEN_R |= 0x1F;                          // enable digital pins PF4-PF0
	GPIO_PORTF_PUR_R |= 0x11;                          // enable weak pull-up on PF4

	
	GPIO_PORTF_IS_R &= ~0x11;                            // (d) PF0 & PF4 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x11;                           //     PF0 & PF4 not both edge
	GPIO_PORTF_IEV_R |= 0x11;
	GPIO_PORTF_ICR_R |= 0x11;                          // (e) clear flag 4
	GPIO_PORTF_IM_R |= 0x11;                           // (f) arm interrupt on PF0 & PF4
		
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFFF1FFFF)|0x00080000; // (g) priority 4
	NVIC_EN0_R |= 0x40000000;                          // (h) enable inturrupt 30 in NVIC
}

//// Interrupt handler for the sensor
void GPIOPortF_Handler(void){
	if(GPIO_PORTF_RIS_R&0x10){
		GPIO_PORTF_ICR_R |= 0x10;
		SequenceOne = 1;
	}
	if(GPIO_PORTF_RIS_R&0x01){
		GPIO_PORTF_ICR_R |= 0x01;
		SequenceTwo = 1;		
	}
}

// Initialize Sensor interface
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
	IRIndicator = 0;

}

// Interrupt handler for the sensor
void GPIOPortE_Handler(void){
	//for(uint32_t time = 727240*100/91; time>0; time--){}
	if(GPIO_PORTE_RIS_R&0x01){
		GPIO_PORTE_ICR_R |= 0x01;
	  IRIndicator = 1;
	}
}
