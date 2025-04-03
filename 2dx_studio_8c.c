/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up



/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////////////////////Global Variables/////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

#define NUM_MEASUREMENTS 32

// note: for degrees we consider cw positive and ccw negative i.e. homeDeg = -45 -> 45 deg CCW
//home should be 0, or the ToF wires will get tangled
double homeDeg = 0; // sets the point at which to return. i.e. 45 means 45 cw from the direction the motor starts
double currDeg = 0; //variable to store our relative position to the starting point
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
uint32_t position = 0;
int status=0;
char aBuffer[1023]; // full buffer that will hold every distance on a single rotation

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////////////////////Global Variables/////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////




void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortH_Init(void){ // used for stepper control
	// Enable clock for Port H
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};	// Wait for clock to stabilize
	// Set PH0-PH3 as outputs for stepper motor control
	GPIO_PORTH_DIR_R |= 0x0F;        							
	// Disable alternate functions on PH0-PH3 (configure as GPIO)
	GPIO_PORTH_AFSEL_R &= ~0x0F;     							
	// Enable digital function on PH0-PH3
	GPIO_PORTH_DEN_R |= 0x0F;        							
	// Disable analog function on PH0-PH3
	GPIO_PORTH_AMSEL_R &= ~0x0F;     							
	
	return;
}

void PortN_Init(void){ // Initialize Port N for onboard LED output (LED0, LED1)
	// Enable clock for Port N
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};	// Wait for clock to stabilize
	// Configure PN0-PN1 as outputs for LED control
	GPIO_PORTN_DIR_R |= 0x03;        							
	// Disable alternate functions on PN0-PN1 (configure as GPIO)
	GPIO_PORTN_AFSEL_R &= ~0x03;     							
	// Enable digital function on PN0-PN1
	GPIO_PORTN_DEN_R |= 0x03;        							
	// Clear data on PN0-PN1 (turn off LEDs initially)
	GPIO_PORTN_DATA_R &= ~0x03;										
	// turn on LED2 (PN1) initially 		
	//GPIO_PORTN_DATA_R |= 0x01;
	// Disable analog function on PN0-PN1
	GPIO_PORTN_AMSEL_R &= ~0x03;     							
	
	return;
}

void PortF_Init(void){ // Initialize Port F for onboard LED output (LED2, LED3)
	// Enable clock for Port F
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};	// Wait for clock to stabilize
	// Configure PF0,PF4 as outputs for LED control
	GPIO_PORTF_DIR_R |= 0x11;        							
	// Disable alternate functions on PF0,PF4 (configure as GPIO)
	GPIO_PORTF_AFSEL_R &= ~0x11;     							
	// Enable digital function on PF0,PF4
	GPIO_PORTF_DEN_R |= 0x11;        							
	// Clear data on PF0,PF4 (turn off LEDs initially)
	GPIO_PORTF_DATA_R &= ~0x11;
	// LED2 on initially (PF4)
	//GPIO_PORTF_DATA_R |= 0x10;
	// Disable analog function on PF0,PF4
	GPIO_PORTF_AMSEL_R &= ~0x11;     							
	
	return;
}

void PortJ_Init(void){ // Initialize Port J for onboard switches
	// Enable clock for Port J
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};	// Wait for clock to stabilize
	// Configure PJ0-PJ1 as inputs for switch signals
	GPIO_PORTJ_DIR_R &= ~0x03;        							
	// Disable alternate functions on PJ0-PJ1 (configure as GPIO)
	GPIO_PORTJ_AFSEL_R &= ~0x03;     							
	// Enable digital function on PJ0-PJ1
	GPIO_PORTJ_DEN_R |= 0x03;        							
	// Disable analog function on PJ0-PJ1
	GPIO_PORTJ_AMSEL_R &= ~0x03;
	// Enable pull up resistors on pins PJ0-PJ1
	GPIO_PORTJ_PUR_R |= 0x03;	
	
	return;
}





// past inits //////////////////////////////////////////////////////////////////////////////////////////////






// can toggle any onboard led
void toggleLED (int num){
	switch(num){
		case 0: //PN1 - UART Transmission Indicator, at the beginning of a transmission block flash the defined LED
			GPIO_PORTN_DATA_R ^= 0x02;
			break;
		case 1: //PN0 - Measurement Status, Distance measurement requires an LED for status indication
			GPIO_PORTN_DATA_R ^= 0x01;
			break;
		case 2: //PF4 - Troubleshooting, used for blinking the LED every 11.25 degrees
			GPIO_PORTF_DATA_R ^= 0x10;
			break;
		case 3: //PF0 - N/A
			GPIO_PORTF_DATA_R ^= 0x01;
			break;
	}
}


// if debounce is 0, check for btnPress
// if debounce is 1, while loop for debounce
int checkBtnPresses(int btnNum, int debounce){
	if (!debounce){
		// for each of these cases, we return 1 when the btn is being pressed
		switch(btnNum){
			case 0: // port j0, motor start/stop data acquisition
				return ((GPIO_PORTJ_DATA_R & 0x01) == 0) ? 1 : 0;
				break;
			case 1: // port j1, start/stop rotation and measurement
				return ((GPIO_PORTJ_DATA_R & 0x02) == 0) ? 1 : 0;
				break;
		}
	} else{
		switch(btnNum){
			case 0: // port j0, motor start/stop
				while ((GPIO_PORTJ_DATA_R & 0x01) == 0);
				SysTick_Wait10ms(10);
				break;
			case 1:
				while ((GPIO_PORTJ_DATA_R & 0x02) == 0);
				SysTick_Wait10ms(10);
				break;
		}
	}
	// garbage value
		return -100;
}


void motorIteration(int direction){
	int delay = 200; // each wait call is 10ms, making this delay (10*delay) ms, this is global for testing
	
	if (direction == 0){ //CW
		GPIO_PORTH_DATA_R = 0b0011;
		SysTick_Wait10us(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
		GPIO_PORTH_DATA_R = 0b0110;													// Complete the missing code.
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b1100;													// Complete the missing code.
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b1001;													// Complete the missing code.
		SysTick_Wait10us(delay);
	} else { //CCW
		GPIO_PORTH_DATA_R = 0b1001;
		SysTick_Wait10us(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
		GPIO_PORTH_DATA_R = 0b1100;													// Complete the missing code.
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b0110;													// Complete the missing code.
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b0011;													// Complete the missing code.
		SysTick_Wait10us(delay);
	}
}

int roundUp(double num) {
    int intPart = (int)num;  // Get the integer part of the dbl
    return (num > intPart) ? (intPart + 1) : intPart; // return the rounded up version
}
	
void returnHome (){
	//return home is being displayed as active for troubleshooting
	toggleLED(2);
	
	int numIterations;
	//how far we need to travel home (must be +ve)
	double degDiff = currDeg - homeDeg;
	degDiff = degDiff < 0 ? degDiff*-1 : degDiff;
	
	if (degDiff >= 360 || degDiff <= 0){ // prevents tangling of ToF
		numIterations = 512; // full rotation
	} else {
		// home may not be possible to achieve due to the step size not matching up exactly,
		// meaning that an extra 0.703125 iteration may be taken
		degDiff = degDiff/0.703125;
		numIterations = roundUp(degDiff); 
	}
	
	// you need to go CCW
	//move ccw til home
	for(int i = 0; i < numIterations; i++){												
		motorIteration(1);
	}
	
	currDeg = homeDeg;
	toggleLED(2);
}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	//onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init(); // Motor used
	PortN_Init(); // 2 LEDs used
	PortF_Init(); // 1 LED used
	PortJ_Init(); // buttons

	int running = 0;
	
	while (1){
		//PJ1, motor rotation
		if (checkBtnPresses(1,0)) {
			checkBtnPresses(1,1); // debounces
			running = 1;
		} else{
			running = 0;
		}
		
		//PJ0, data acquisition
		if (checkBtnPresses(0,0)) {
			checkBtnPresses(0,1); // debounces
			// send the UART for all readings
			UART_printf(aBuffer);
			//empty the buffer for the next runthrough
			memset(aBuffer, 0, sizeof(aBuffer));
			/*
			NOTE: if i need to change this so its actually starting and stopping acquisition
			all i need to do is make a boolean of isAcquiring for the sprintf concat lines, 
			while also making sure to count the number of steps, sending the uart when the count gets to 32
			*/
			toggleLED(0);
			SysTick_Wait10ms(1);
			toggleLED(0);
			//FlashLED1(1); // transmission indicator
			//add transmit
		}
		
		// start rotation and measurement
		if (running){
			char temp[10]; // Temporary buffer to store a single distance value before concat

			
			
			// 1 Wait for device booted
			while(sensorState==0){
				status = VL53L1X_BootState(dev, &sensorState);
				SysTick_Wait10ms(10);
			}
			FlashAllLEDs(); // booted indication, for debugging
			//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

			/* 2 Initialize the sensor with the default setting  */
			status = VL53L1X_SensorInit(dev);
			//Status_Check("SensorInit", status); DEBUG


			/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
			//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
			//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
			//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

			// 4 What is missing -- refer to API flow chart
			status = VL53L1X_StartRanging(dev);	   // This function has to be called to enable the ranging

			// Get the Distance Measures 8 times / 32 times
			int i = 0;
			while ((i < NUM_MEASUREMENTS) && (GPIO_PORTJ_DATA_R & 0x02) && (GPIO_PORTJ_DATA_R & 0x01)) {
				
				// 5 wait until the ToF sensor's data is ready
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
							//FlashLED3(1);
							VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;
				
				//7 read the data values from ToF sensor
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);				//The Measured Distance value
				status = VL53L1X_GetSignalRate(dev, &SignalRate);
				status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
				status = VL53L1X_GetSpadNb(dev, &SpadNum);
				
				toggleLED(1);
				SysTick_Wait10ms(1);
				toggleLED(1);
				//FlashLED2(1); // flash PN0 for measurement status indication

				status = VL53L1X_ClearInterrupt(dev);	 /* 8 clear interrupt has to be called to enable next interrupt*/
				
				// print the resulted readings to UART
				//sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
				// print the resulted readings to UART immediately
				//sprintf(printf_buffer, "%u\r\n", Distance);
				//UART_printf(printf_buffer);
				
				// accumulate all readings to distBuffer
				sprintf(temp, "%u ", Distance); // Format as a string
				strcat(aBuffer, temp); // Append to main buffer
				
				
				SysTick_Wait10ms(20);
				
				int j = 0;
				// 45 deg -> 64 (8 measures), 11.25 deg -> 16 (32 measures) 
				while(j < 512/NUM_MEASUREMENTS && (GPIO_PORTJ_DATA_R & 0x02) && (GPIO_PORTJ_DATA_R & 0x01)) {
					motorIteration(0);
					position++;	
					j++;
					currDeg += 0.703125; // how much distance was covered in the previous 8 lines of code (CW)
					if (currDeg >= 360){
						currDeg -=360;
					}
				}
				
				i++;
			}
			// debounces if necessary
			checkBtnPresses(1,1);
			// return home here			
			returnHome();
			/*
			for(int j = 0; j < 64*8; j++) {//512
					motorIteration(1);
					position--;
			}
			*/
			
			VL53L1X_StopRanging(dev);				
			running = 0; // finished this run
		}
		
	}
}

