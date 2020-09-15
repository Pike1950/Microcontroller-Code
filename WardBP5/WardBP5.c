//-------------------------------------------------------------------------------
//       ECE 3362-002 Project 5 - 4 digit 7 segment display kitchen timer program (WRITTEN IN C CODE)
//               The assembly code in Project 3 was converted to C code for this program.
//               This program utilizes the same 4 digit 7 segment display from the
//               previous projects. The display will be used to show a kitchen timer
//               that will countdown by 100 milliseconds from 3.6 seconds based on 
//               the unique two-digit number given from the professor. The buttons
//               will be wired to work as START, RESET, and LOAD on the left, middle,
//               and right buttons respectively. The START button will initiate the
//               countdown only when the time value is loaded in, the RESET button 
//               will always reset the display and time value back to its initial
//               state, and the LOAD button will only load the time value in if it
//               is in its initial state. Once the countdown completes, the display 
//               will blink 00:00 until the user presses the reset button at which
//               point it will constantly display 00:00 or 03:60 after pressing the 
//               LOAD button. 
//                     
//       Purpose: This program is designed to introduce the student into using C
//              code to recreate the results of the assembly instructions from 
//              Project 3 in which a 4 digit display, timer interrupts, port 
//              interrupts, and DADD instructions were implemented. 
//
//       Target: TI LaunchPad development board with MSP430G2553 device with the
//               custom Display Board installed
//
//       Date:           November 26, 2018
//       Last Revision:  1.0                      
//       Written by:     Bradley Ward, ECE dept, Texas Tech University
//       Adapted from:   Provided code examples; Dr. M. Helm, October - November 2018   
//                       DemoStopWatchFramework.s43
//                       C_vs_AssyExamplesJuly5_2016v3.pdf
//                       DemoC_PushButtonPolling.c
//                       C_CodeExamples.c
//                       DemoCProjWithPBIntAndADCrevised.c
//                       DemoWithTwoTimersC_codeRevised.c
//       Assembler/IDE:  IAR Embedded Workbench 6.5
//
//       HW I/O assignments:
//       P1.0    (output) Segment A (active low) drives display board
//       P1.1    (output) Segment B (active low) drives display board
//       P1.2    (output) Segment C (active low) drives display board
//       P1.3    (output) Segment D (active low) drives display board
//       P1.4    (output) Segment E (active low) drives display board
//       P1.5    (output) Segment F (active low) drives display board
//       P1.6    (output) Segment G (active low) drives display board
//       P1.7    (output) Segment DP (active low) drives display board
//
//       P2.0    (output) Digit 3 (active low) MSdigit (leftmost)
//       P2.1    (output) Digit 2 (active low)  
//       P2.2    (output) Digit 1 (active low)  
//       P2.3    (output) Digit 0 (active low) LSdigit (rightmost)
//       P2.4    (output) Other - (dots - colon)
//       P2.5    (input)  Pushbutton 0 (active low) (rightmost)
//       P2.6    (input)  Pushbutton 1 (active low) (leftmost)
//       P2.7    (input)  Pushbutton 2 (active low) (middle)
//
//
//

#include  <msp430g2553.h>

//-------------------------------------------------------------------------------
// CONSTANT DEFINITIONS
//-------------------------------------------------------------------------------

#define LONG_DELAY    (0x7FFF) 
#define DISPLAY_DELAY  (0x0032u)

#define TIMER_A0_COUNT_1  (0x07D0u)   // set count value for TimerA_0  
// results in a 2 mS interrupt rate for updating each digit position in the
// display based on 1 MHz SMCLK/1 and counting 2000 of the 1 uS clock events
#define TIMER_A1_COUNT_1  (0xC350u)  // set count value for TimerA_1  
// results in a 100 mS basic interrupt rate based on 1 MHz SMCLK/2
// this will be a clock rate of one event per 2 uS, counting 50000 of those
// results in one interrupt every 100 mS from this timer

#define MAX_TIMER_COUNT (0x000Au) // number of TimerA1 Interrupts that have to occur
// to equal 1 second of elapsed time.

#define SEG_A_N  ~(0x01u) //inverse of %00000001 Port pin position P1.0
#define SEG_B_N  ~(0x02u) //inverse of %00000010 Port pin position P1.1
#define SEG_C_N  ~(0x04u) //inverse of %00000100 Port pin position P1.2
#define SEG_D_N  ~(0x08u) //inverse of %00001000 Port pin position P1.3
#define SEG_E_N  ~(0x10u) //inverse of %00010000 Port pin position P1.4
#define SEG_F_N  ~(0x20u) //inverse of %00100000 Port pin position P1.5
#define SEG_G_N  ~(0x40u) //inverse of %01000000 Port pin position P1.6
#define SEG_DP_N ~(0x80u) //inverse of %10000000 Port pin position P1.7

#define DIG_3 ~(0x01)          //inverse of %00000001 Port pin position P2.0 (MSdigit)
#define DIG_2 ~(0x02)          //inverse of %00000010 Port pin position P2.1
#define DIG_1 ~(0x04)          //inverse of %00000100 Port pin position P2.2
#define DIG_0 ~(0x08)          //inverse of %00001000 Port pin position P2.3(LSdigit)
#define COL_DG_COM ~(0x10)     //inverse of %00010000 Port pin position P2.4

// Pushbutton assignments CORRECTED to compensate for board layout swap
#define PB_0 (0x20u) //%00100000 Port pin position P2.5  RightMost button
#define PB_1 (0x80u) //%10000000 Port pin position P2.7  Middle button
#define PB_2 (0x40u) //%01000000 Port pin position P2.6  LeftMost button

#define SEG_PORT (P1OUT)
#define DIG_PORT (P2OUT)
#define PB_PORT  (P2IN)

// NOTE: display bd requires the INVERSE of these patterns due to Active LOW
#define ONE ~(0x06u)            //inverse of %00000110
#define TWO ~(0x5Bu)            //inverse of %01011011
#define THREE ~(0x4Fu)          //inverse of %01001111
#define FOUR ~(0x66u)           //inverse of %01100110
#define FIVE ~(0x6Du)           //inverse of %01101101
#define SIX ~(0x7Du)            //inverse of %01111101
#define SEVEN ~(0x07u)          //inverse of %00000111
#define EIGHT ~(0x7Fu)          //inverse of %01111111
#define NINE ~(0x67u)           //inverse of %01100111
#define ZERO ~(0x3Fu)           //inverse of %00111111
#define LETTER_C ~(0x39u)       //inverse of %00111001
#define NO_LED_N ~(0x00u)       //inverse of %00000000

//----------------------------------------------------------------------
// GLOBAL VARIABLE definition
//----------------------------------------------------------------------
int TimerCompareBool = 0; // boolean value for checking if timer needs to
                          // be compared
int FlashTimerBool = 0; //boolean value for flashing the timer display with
                        // 00:00
int TimerVarAdd = 0; // variable that is used to store result of DADD
                     // instruction in TimerA1

unsigned int DisplayValue = (0x0000u); // contains 4 digit value to display in BCD format
                                       // BCDdig3 | BCDdig2  | BCDdig1  | BCDdig0
                                       // xxxx      xxxx       xxxx       xxxx
                            
int CurrentDigitPos = 0; // global variable used by WriteDigitToDisplay ISR
                         // holds digit position of current digit to write
                            
int CurrentDigitValue = 0; // global variable used by WriteDigitToDisplay ISR
                           // holds digit value of next digit to write  

unsigned int TempTimeVar = 0; // Temp variable for holding display value

int ColonTestValue = 0; // variable to check if colon is displayed

int ControlVar1, ControlVar2 = 0; // control variables for affecting button operation
                                  // ControlVar1 -> Load Button
                                  // ControlVar2 -> Reset Button

int TestMask = 0; // variable for masking lower nibble

int PB_0_Mode = 0; // boolean values for checking which pushbutton
int PB_1_Mode = 0; // was pressed
int PB_2_Mode = 0;

int SegPatternTable[12] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, LETTER_C, NO_LED_N};

//----------------------------------------------------------------------
// FUNCTIONs
//----------------------------------------------------------------------

void WriteNextDigitToDisplay()
{
  DIG_PORT |= (DIG_0+DIG_1+DIG_2+DIG_3+COL_DG_COM); // this turns off any 
                                                  // digits that are already ON
                                                  // eliminates the "ghosting" problem
  if(CurrentDigitPos==0) // if Digit 0
  {
    TempTimeVar = DisplayValue; // make a copy
    TempTimeVar &= (0x000Fu); // mask out all but Dig 0
    TempTimeVar = SegPatternTable[TempTimeVar]; // use temp var to find right var in array
    SEG_PORT = TempTimeVar; // set up pattern to write
    DIG_PORT = DIG_0;
    CurrentDigitPos++; // increment current digit
  }
  
  else if(CurrentDigitPos==1) // if Digit 1
  {
    TempTimeVar = DisplayValue; // make a copy
    TempTimeVar &= (0x00F0u); // mask out all but Dig 1
    TempTimeVar = TempTimeVar >> 4; // get value into LSnibble position
    TempTimeVar = SegPatternTable[TempTimeVar]; // use temp var to find right var in array
    SEG_PORT = TempTimeVar; // set up pattern to write
    DIG_PORT = DIG_1; 
    CurrentDigitPos++; // increment current digit
  }
  
  else if(CurrentDigitPos==2) // if Digit 2
  {
    TempTimeVar = DisplayValue; // make a copy
    TempTimeVar &= (0x0F00u); // mask out all but Dig 2
    TempTimeVar = TempTimeVar >> 8; // get value into LSnibble position
    TempTimeVar = SegPatternTable[TempTimeVar]; // use temp var to find right var in array
    SEG_PORT = TempTimeVar; // set up pattern to write
    DIG_PORT = DIG_2; 
    CurrentDigitPos++; // increment current digit
  }
  
  else if(CurrentDigitPos==3) // if Digit 3
  {
    TempTimeVar = DisplayValue; // make a copy
    TempTimeVar &= (0xF000u); // mask out all but Dig 3
    TempTimeVar = TempTimeVar >> 12; // get value into LSnibble position
    TempTimeVar = SegPatternTable[TempTimeVar]; // use temp var to find right var in array
    SEG_PORT = TempTimeVar; // set up pattern to write
    DIG_PORT = DIG_3;
    CurrentDigitPos++; // increment current digit
  }
  
  else if(CurrentDigitPos==4) // if Colon
  {
    DIG_PORT |= (0xFFu); // clear out digit port
    if(ColonTestValue == 1) // check if colon test var is 1
    {
      SEG_PORT = 0x04; // if so, move to colon port
      DIG_PORT = COL_DG_COM; // turn on led for colon
    }
    CurrentDigitPos++; // increment current digit
  }
  
  else
    CurrentDigitPos = 0; // if current digit position is anything else, set it to
                         // zero to restart this process
  
  return;
};

void Delay_Long(int DelayTime)
{
  for(int i = 0;i < DelayTime;i++); // decrement loop counter
  
  for(int i = 0;i < DelayTime;i++);
}

// MAIN PROGRAM 
void main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT  
  
  //Set up Port 1 & Port 2
  P1DIR = (0xFFu); // all as outputs
  P2DIR = (0x1Fu); // all but 3 MSbits as outputs
  
  // The PushButtons are wired to P2.5,6,7 (active LOW), so we need to  
  // turn on the internal Pullup Resistors  - this is done with P2REN
  // register
           
  P2REN = (PB_0+PB_1+PB_2);    // turn on the internal resistor
  P2OUT = (PB_0+PB_1+PB_2);    // set the resistor to Pullup mode
        
  // activate the General Purpose Digital I/O mode for P2.6 and P2.7
  P2SEL &= ~(PB_1+PB_2);
           
  // setup Port 2 interrupts for the pushbuttons
  P2IE |= (PB_0+PB_1+PB_2); // enable interrput for Pushbutton            
  P2IES |= (PB_0+PB_1+PB_2); // set edge select for high to low trans
    
  // turn off all the segments and digits
  SEG_PORT = (0xFFu);
  DIG_PORT = (0xFFu);

  // Set up Timers
  
  // Set up the clock (calibrated mode at 1 MHz)
  // Get the calibrated data for the DCO clock
  // Set DCO to 1 MHz:  (this directly from TI Family Guide page283 and 284
  DCOCTL = 0x00;
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
  
  // TimerA0
  TA0CCR0 = TIMER_A0_COUNT_1;              // load a count "up to"value into timer
  TA0CTL = TASSEL_2+ID_0+MC_1;          // select SMCLK/1, up mode
  TA0CCTL0 = CCIE;                      // interrupt enabled for Timer0

  // TimerA1
  TA1CCR0 = TIMER_A1_COUNT_1;              // load a count "up to"value into timer
  TA1CTL = TASSEL_2+ID_1+MC_0;         // select SMCLK/2, up mode  
  TA1CCTL0 = CCIE;                      //  interrupt enabled for Timer1
  
  // clear out any garbage stored in these variables
  PB_0_Mode = 0; 
  PB_1_Mode = 0;
  PB_2_Mode = 0;
  
  P1IFG = 0; // clear the Int flag register for Port 1 
  P2IFG = 0; // clear the Int flag register for Port 2 
  
  _BIS_SR(GIE); // enable general interrupts
  
  DisplayValue = 0xABBB; // display C on leftmost digit with other
                         // digits left blank
  
  Delay_Long(LONG_DELAY);
  Delay_Long(LONG_DELAY);
  
  DisplayValue = 0x0000; // display 00:00 (colon added on next line)
  
  ColonTestValue = 0x01; // put value in test var for drawing the colon
    
   while(1)  // forever idle loop - Timer interrupts control the action
   {
   
   if (PB_0_Mode == 1) // LOAD Button - Right
   {
     if (DisplayValue == 0x0000) // check if time value is currently 00:00
       ControlVar1 = 1; // if yes, set var to 1
     if (FlashTimerBool == (0x0001u)) // check to see if display is flashing
       ControlVar1 = 0; // if no, set var to 0
     
     if (ControlVar1 == 1)
     {
        TA1CTL = TASSEL_2+ID_1+MC_0; // set TimerA1 to no count mode
        DisplayValue = (0x0360); // display 03:60 on board
     }
     
     ControlVar2 = 1; // enable start button
     
     PB_0_Mode = 0;  // clear the Boolean flag
   }
   
   else if (PB_1_Mode == 1) // ; RESET Button - Middle
   {
     TA1CTL = TASSEL_2+ID_1+MC_0; // set TimerA1 to no count mode
     TimerCompareBool = 0; // disable timer compare
     FlashTimerBool = 0; // disable flashing
     ControlVar1 = 1; // enable load button
     ControlVar2 = 0; // disable start button
     DisplayValue = 0x0000; // display 00:00 on board
     PB_1_Mode = 0 ; //clear the Boolean flag
   }
   
   else if (PB_2_Mode == 1) // ; START Button - Left
   {
     if (DisplayValue == 0x0000) // check to see if time is 00:00
        ControlVar2 = 0; // if yes, set var to 0
     if (DisplayValue == 0xBBBBu) // check to see if display is flashing
        ControlVar2 = 0; // if yes, set var to 0
        
     if (ControlVar2 == 1)
     {   
        TA1CTL = TASSEL_2+ID_1+MC_1; // set TimerA1 clock mode to up
        TimerCompareBool = 0x0001 ; // set boolean flag to compare time value
        ControlVar1 = 0; // disable start button
     }
     PB_2_Mode = 0; // clear the Boolean flag
   }
   
   if (FlashTimerBool == 0x0001) // test flash timer boolean
   {
      ColonTestValue = 0x00; // clear test boolean for colon
      DisplayValue = (0xBBBBu); // display nothing on board
      Delay_Long(LONG_DELAY);
      Delay_Long(LONG_DELAY);
      ColonTestValue = 0x01; // put value in test register for drawing the colon
      DisplayValue = 0x0000; // display 00:00 on board
      Delay_Long(LONG_DELAY);
      Delay_Long(LONG_DELAY);
   }
   
   if (TimerCompareBool == 0x0001) // test timer compare boolean
   {
      if (DisplayValue == 0x0000) // see if time is 00:00
      { 
          TA1CTL = TASSEL_2+ID_1+MC_0; // set TimerA1 to no count mode
          TimerCompareBool = 0; // clear boolean flag for comparing the time
          FlashTimerBool = 0x0001; // set boolean flag for flashing the display
      }
   }
   }
}  // end of MAIN

//ISR's
//Timer0_A0 ISR
#pragma vector=TIMER0_A0_VECTOR   // this line tells the C compiler to put
                                  // the start address of the following ISR
                                  // into the Interupt Vector table

__interrupt void Timer_A0_ISR (void)   // required syntax for first line of ISR
{
  WriteNextDigitToDisplay();
}

//Timer0_A1 ISR
#pragma vector=TIMER1_A0_VECTOR   // this line tells the C compiler to put
                                  // the start address of the following ISR
                                  // into the Interupt Vector table

__interrupt void Timer_A1_ISR (void)   // required syntax for first line of ISR
{
    TestMask = DisplayValue; // mask out low order byte
    TestMask &= 0x00FF; 
    
    if(TestMask == 0) // check low order byte to see if equal to 00
    {
        DisplayValue = DisplayValue - 0x0100; // if so, subtract 1 second
        DisplayValue = DisplayValue + 0x0090; // add 90 milliseconds
    }
    else
    {
        DisplayValue = DisplayValue - 0x0010; // if not, subtract 10 milliseconds
    }
}


// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & PB_0) // PB_0 Pushbutton?  (if 1 it is pressed)
      PB_0_Mode = 1; // set var to 1
    
    if (P2IFG & PB_1) // PB_1 Pushbutton?  (if 1 it is pressed)
      PB_1_Mode = 1; // set var to 1
    
    if (P2IFG & PB_2) // PB_2 Pushbutton?  (if 1 it is pressed)
      PB_2_Mode = 1; // set var to 1
    
    P2IFG = 0; // Clear Port 2 Interrupt Flags
}

