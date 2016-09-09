/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//*******************************************************************************
//  MSP430F552x Demo - Timer0_A5, PWM TA1.1-2, Up Mode, DCO SMCLK
//
//  Description: This program generates two PWM outputs on P1.2,P1.3 using
//  Timer1_A configured for up mode. The value in CCR0, 512-1, defines the PWM
//  period and the values in CCR1 and CCR2 the PWM duty cycles. Using ~1.045MHz
//  SMCLK as TACLK, the timer period is ~500us with a 75% duty cycle on P1.2
//  and 25% on P1.3.
//  ACLK = n/a, SMCLK = MCLK = TACLK = default DCO ~1.045MHz.
//
//                MSP430F552x
//            -------------------
//        /|\|                   |
//         | |                   |
//         --|RST                |
//           |                   |
//           |         P1.2/TA0.1|--> CCR1 - 75% PWM
//           |         P1.3/TA0.2|--> CCR2 - 25% PWM
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

//Goal
// routine should
// press button 1, it will
// blink LED 1 time, then
// collect an analog data stream at 1000 samples per second using ADC12
// store it in an array using DMA
// later - it will ping-pong between two arrays
// simultaneously output the result to an LED as a PWM from a timer
// initially it will output results at 1/10 normal speed
// if you press button 2, it will output at 1/20 normal speed
// button 2 cycles between these two speeds
// press button 1 again, it will blink the LED twice, and stop
// write the results to a file?
// use FFT?
// graphing
// write to SPI SRAM
// write to SPI display?
// This code
//   Timer A1 set for a PWM out
//    The duty cycle is a triangle wave on P2.0
//   Timer A0 is being used to create a timer interrupt
//    The interrupt service routine simply sets a global variable that indicates the timer has occurred
//    The main routine checks for the flag, takes action the action is to toggle both LED outputs, and then clears the flag

#include <msp430.h>
volatile unsigned int i;	// volatile to prevent optimization
volatile int zz;
volatile int yy;
volatile int timerInterruptFlag; //timer Interrupt to be 20 kHz 50 usec
volatile int ADCtimeCount;  // ADC set for 10 kHz
#define Timer0Count 32      //this gives a 0.5 msec interval
#define ADCtimeCountTarget	1 // ADC at 2 msec intervals
volatile int LEDtimeCount;
#define LEDtimeCountTarget	1  // LED at 50 msec intervals  (1/25th of real time)
volatile unsigned int latestADCresult;
#define   NumOfResults   1000
volatile unsigned int resultsIndex, playbackIndex, playbackSpeed, playbackSpeedCounter;
volatile int  ADCconversionComplete;
volatile int loopCount;
volatile int tIntFlagResults[100];
volatile int tindex;
volatile int foundtIntFlag;
volatile int copyOfTimerInterruptFlag;
volatile int Sw1One, Sw1Zero, checkSw1Count, sw1Value, checkState, currentState, lastState, blinkState, greenLEDCount;
volatile int Sw2One, Sw2Zero, checkSw2Count, sw2Value, newSw2State;
#define checkSw1CountTarget 1
#define checkSw2CountTarget 1

volatile unsigned int results[NumOfResults];
                                            // Needs to be global in this
                                            // example. Otherwise, the
                                            // compiler removes it because it
                                            // is not used for anything.
//volatile unsigned int timerMin, timerMax, thisTimerVal, timerOverflow;
//volatile long int timerCount;

//set up for reading buttons
//P2.1 is S1, P1.1 is S2

//set up for state machine
volatile int state;   //state changes 1 is demo mode, 2 is full speed reflection of input, 3 is slow speed of input

//set up for green LED
//P4.7 is Green LED

//volatile float timerSum, timerAvg;
volatile unsigned int timerOverflow;
// timerStats 0 latestValue, 1 average, 2 max, 3 min, 4 Sum, 5 count
volatile float  timerStats[6]={0,0,0,4096,0,0};
volatile float  tFlagStats[6]={0,0,0,4096,0,0};

updateStats( float *statsArray)
{

 if (statsArray[0] > statsArray[2]) {statsArray[2]=statsArray[0];}
 if (statsArray[0] < statsArray[3]) {statsArray[3]=statsArray[0];}
 statsArray[5]=statsArray[5]+1;
 statsArray[4]=statsArray[4]+statsArray[0];
 statsArray[1]=statsArray[4]/statsArray[5];

}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  P2DIR |= BIT0;                       // P2.0 output
  P2SEL |= BIT0;                       // P2.0 options select
//  TA1CCR0 = 4096-1;                          // PWM Period
  TA1CCR0 = 512-1;                          // PWM Period
  TA1CCTL1 = OUTMOD_7;                      // CCR1 reset/set

  TA1CCR1 = 384;                            // CCR1 PWM duty cycle

  TA1CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR

  checkSw1Count=0;
  checkState=0;  //rename this to newSw1State
  sw1Value=0;
  Sw1One=0;
  Sw1Zero=0;
  checkSw2Count=0;
  newSw2State=0;
  sw2Value=0;
  Sw2One=0;
  Sw2Zero=0;
  currentState=1;
  lastState=6;
  blinkState=8;
  //Green LED is P4.7
  P4DIR |= 0x80;
  P4SEL &= 0x7F;
  //S1 button is P2.1
  P2DIR &= 0xFD;
  P2SEL &= 0xFD;
  P2REN |= 0x02;
  P2OUT |= 0x02;
  //S2 button is P1.1
  P1DIR &= 0xFD;
  P1SEL &= 0xFD;
  P1REN |= 0x02;
  P1OUT |= 0x02;

  //diag outputs
  //diag9 is P3.6 (Left, Inner, Pin 9)
   P3DIR |= 0x40;
   P3SEL &= 0xBF;
   #define diag9 P3OUT
   #define diag9_H 0x40
   #define diag9_L 0xEF
   //diag6 is P3.4 (Left, Inner, Pin 6)
    P6DIR |= 0x08;
    P6SEL &= 0xF7;
    #define diag6 P6OUT
    #define diag6_H 0x08
    #define diag6_L 0xF7
    //diag10 is P3.5 (Left, Inner, Pin 10)
    P3DIR |= 0x20;
    P3SEL &= 0xDF;
#define diag10 P3OUT
#define diag10_H 0x20
#define diag10_L 0xDF

  //diag7ain is P6.4 (Left, Inner, Pin 7)
  P6DIR |= 0x10;
  P6SEL &= 0xEF;
  #define diag7 P6OUT
  #define diag7_H 0x10
  #define diag7_L 0xEF

  //diag8ain is P7.0 (Left, Inner, Pin 8)
  P7DIR |= 0x01;
  P7SEL &= 0x7E;
  #define diag8 P7OUT
  #define diag8_H 0x01
  #define diag8_L 0x7E




  //Timer 0 setup for interrupt routine
  P1DIR |= 0x01;                            // P1.0 output
  P4DIR |= 0x80;                            // P4.7 output
  TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
  TA0CCR0 = Timer0Count;								// set timer to 0.5 msec
  TA0CTL = TASSEL_1 + MC_1 + TACLR;         // was TASSEL_2 - SMCLK, now TASSEL_1 - ACLK, upmode, clear TAR
  	  	  	  	  	  	  	  	  	  	  	// was SMCLK 8 MHz - 8 clocks = 1 usec 8000 clocks is 1 msec
  	  	  	  	  	  	  	  	  	  	  	// now ACLK 32 kHz - 32 clocks=1 msec
  	  	  	  	  	  	  	  	  	  	  	// 50000 clocks is 6.25 usec (160 Hz)

 // Setup for ADC12 interrupt routine
  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;
  P6SEL |= 0x01;                            // P6.0 ADC option select ... pin 3 on the inside of the left connector
  P1DIR |= 0x01;                            // P1.0 output
  ADC12CTL0 |= ADC12SC;                     // Start conversion

  __bis_SR_register(GIE);       			//  enable interrupts

  resultsIndex=0;
  playbackIndex=0;
  playbackSpeed=10;
  playbackSpeedCounter=0;
  yy=1;
  loopCount=0;
  timerOverflow=0;
  timerInterruptFlag=0; 					//initialize flag that indicates whether or not timer has expired
  copyOfTimerInterruptFlag=0;
  LEDtimeCount=0; 							//use this to count til LEDtime occurs
  ADCtimeCount=0; 							//use this to count til time to start next ADC
  tindex=0;
  foundtIntFlag=0;
  greenLEDCount=0;
  //  __bis_SR_register(LPM0_bits); // Enter LPM0

//***************** top of loop *****************************

for (;;) {
	diag6 |= diag6_H;  // Set diag6 H to indicate top of loop
  copyOfTimerInterruptFlag=timerInterruptFlag;

//timer service routine activities /////////////////////////////////////
  if (copyOfTimerInterruptFlag > 0 )
     {
	  if (copyOfTimerInterruptFlag > 1) { timerOverflow=timerOverflow+1;}
	  timerInterruptFlag=0;
	  LEDtimeCount=LEDtimeCount+copyOfTimerInterruptFlag;
	  ADCtimeCount=ADCtimeCount+copyOfTimerInterruptFlag;
	  checkSw1Count=checkSw1Count+copyOfTimerInterruptFlag;
	  checkSw2Count=checkSw2Count+copyOfTimerInterruptFlag;
	  if ( blinkState > 0 ) {greenLEDCount=greenLEDCount+copyOfTimerInterruptFlag;}
  P1OUT ^= 0x01;                            // Toggle P1.0 to show timer interrupt speed
     }
// this is really Switch 2
// check Switch1 ////////////////////////////////////////////////////////////
if (checkSw1Count >= checkSw1CountTarget)
{
checkSw1Count=checkSw1Count-checkSw1CountTarget;
	if (P1IN & 0x02)
		{
		  //P1.1 is set
		  if (Sw1One<100) {Sw1One=Sw1One+1; }
		  else            {Sw1One=100;}
		  if (Sw1One > 3 & sw1Value == 0) { checkState = 1;  sw1Value=1; }
			Sw1Zero=0;
		}
	else
		{
		  //P1.1 is not set
		  if (Sw1Zero<100) {Sw1Zero=Sw1Zero+1;}
		  else             {Sw1Zero=100;}
		  if (Sw1Zero > 3 & sw1Value == 1) { checkState = 1;  sw1Value=0; }
			Sw1One=0; }
}

// this is really switch 1
// check Switch2 ////////////////////////////////////////////////////////////
if (checkSw2Count >= checkSw2CountTarget)
{
checkSw2Count=checkSw2Count-checkSw2CountTarget;
	if (P2IN & 0x02)
		{
		  //P2.1 is set
		  if (Sw2One<100) {Sw2One=Sw2One+1; }
		  else            {Sw2One=100;}
		  if (Sw2One > 3 & sw2Value == 0) { newSw2State = 1;  sw2Value=1; }
			Sw2Zero=0;
		}
	else
		{
		  //P2.1 is not set
		  if (Sw2Zero<100) {Sw2Zero=Sw2Zero+1;}
		  else             {Sw2Zero=100;}
		  if (Sw2Zero > 3 & sw2Value == 1) { newSw2State = 1;  sw2Value=0; }
			Sw2One=0; }
}


// State Machine //////////////////////////////////////////////////////////////////////////////
if (checkState > 0 | blinkState > 0)
{
//State Machine
//There are 3 modes of operation: Demo mode, Capture mode with full speed playback, and Capture mode with slow speed playback
//Power up -->
// 	State=1  //Demo mode, state button is up
//     		If debounced button down =Y, go to State=1a, else stay in State=1
//	State=1a //Demo mode, state button down.  Waiting for button up
//     		If debounced button up =Y, set newStateBit, go to State=2, else stay in State=1a
//
// 	State=2  //Capture mode, full speed playback, state button is up
//     		If debounced button down =Y, go to State=2a, else stay in State=2
//	State=2a //Capture mode, full speed playback, state button down.  Waiting for button up
//     		If debounced button up =Y, set newStateBit, go to State=3, else stay in State=2a
//
// 	State=3  //Capture mode, slow speed playback, state button is up
//     		If debounced button down =Y, go to State=3a, else stay in State=3
//	State=3a //Capture mode, slow speed playback, state button down.  Waiting for button up
//     		If debounced button up =Y, set newStateBit, go to State=1, else stay in State=3a

//if (sw1Value == 0){ 	   P4OUT &= 0x7F;}                            // Toggle P4.7
//if (sw1Value == 1){ 	   P4OUT |= 0x80;}                            // Toggle P4.7
switch ( currentState ) {
case 1: if (lastState != 1) {blinkState = 1; checkState=0;}
		lastState=1;
		if (sw1Value == 1) { currentState = 2; checkState=0;}
		break;
case 2: lastState=2;
		if (sw1Value == 0 & blinkState == 0) {currentState = 3; checkState=0;} // wait till blink has occurred and switch has occurred
		break;
case 3: if (lastState != 3) {blinkState = 2; checkState=0;}
		lastState=3;
		if (sw1Value == 1) { currentState = 4; checkState=0;}
		break;
case 4: lastState=4;
		if (sw1Value == 0 & blinkState == 0) {currentState = 5; checkState=0;}
		break;
case 5: if (lastState != 5) {blinkState = 3; checkState=0;}
		lastState=5;
		if (sw1Value == 1) { currentState = 6; checkState=0;}
		break;
case 6: lastState=6;
		if (sw1Value == 0 & blinkState == 0) {currentState = 1; checkState=0;}
		break;
default: lastState=6;
		currentState=1;
		checkState=0;
		blinkState = 0;
		break;
}

}

// Green State LED Management
  if ( blinkState > 0 ){
	  if ( greenLEDCount < 150 ){ P4OUT &= 0x7F; }
	  if ( greenLEDCount < 500 & greenLEDCount >= 150){ P4OUT |= 0x80; }
	  if ( greenLEDCount >= 500 ){ P4OUT &= 0x7F; blinkState=blinkState-1; greenLEDCount=0;}

  }

// Red LED Management /////////////////////////////////////////////////////////////////////////////////////////
  if (LEDtimeCount >= LEDtimeCountTarget)
	 { //should have used a switch statement
	  LEDtimeCount=LEDtimeCount-LEDtimeCountTarget;
	  // Adjust the PWM duty cycle
	  if (currentState == 1 | currentState == 2){
		  zz=zz+yy;
		  if (zz > 512-1) { zz=512-1; yy=-8; }
		  if (zz < 0) { zz=0; yy=8; }
		  TA1CCR1 = zz;
		  TA2CCR1 = zz;
	  }
	  if (currentState == 3 | currentState == 4){
		playbackIndex=playbackIndex+1;
	 		if (playbackIndex >= NumOfResults) { playbackIndex = 0;}
		TA1CCR1 = results[playbackIndex]>>3;
		TA2CCR1 = results[playbackIndex]>>3;
	  }
	  if (currentState == 5 | currentState == 6){
 		playbackSpeedCounter=playbackSpeedCounter+1;
		if ( playbackSpeedCounter >= playbackSpeed ) {
			playbackSpeedCounter=0;
playbackIndex=playbackIndex+1;
	 			if (playbackIndex >= NumOfResults) { playbackIndex = 0;}
			TA1CCR1 = results[playbackIndex]>>3;
			TA2CCR1 = results[playbackIndex]>>3;
		}
	  }
//	   P4OUT ^= 0x80;                            // Toggle P4.7
  	  }
  diag6 &= diag6_L;  // Set diag6 L to indicate completion of loop

  if (ADCtimeCount >= ADCtimeCountTarget)
  {
	// check to see if Sw is being pressed.  If so, then capture
diag8 |= diag8_H;		//starting interrupt conversion
	  resultsIndex=resultsIndex+1;   // Increment results index, modulo; Set Breakpoint1 here
	 ADCtimeCount=ADCtimeCount-ADCtimeCountTarget;
	 if (resultsIndex >= NumOfResults)
	 	 {
		 	 resultsIndex = 0;
	 	 }
	 results[resultsIndex]=latestADCresult;
	 ADC12CTL0 |= ADC12SC;                     // Start next conversion
  }
 //Collect timer stats
 //timerStats[0]=TA0R;
 //updateStats ( &timerStats );


}
}
//  __no_operation();                         // For debugger





//ADC interrupt routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void)
#else
#error Compiler not supported!
#endif
{
	diag8 &= diag8_L;    //start of analog interrupt routine ... hence interrupt is complete
	switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
    latestADCresult = ADC12MEM0;             // Move results
    ADCconversionComplete = 1;
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}


// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	diag9 ^= diag9_H ;                     // Toggle diag9 bit  -- 1 msec
	timerInterruptFlag=1;

//	timerInterruptFlag=timerInterruptFlag+1;
//	if ( foundtIntFlag==1) { foundtIntFlag=0; timerInterruptFlag=1;}
}



