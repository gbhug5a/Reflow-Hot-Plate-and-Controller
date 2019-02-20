/*

                        REFLOW CONTROLLER
                               v1.0

This is the firmware for a duty-cycle reflow controller.  It does not use feedback,
but rather executes a sequence of AC mains duty-cycle stages previously determined
through experimentation to approximate the standard temperature profile for leaded
solder paste.  The stages defined here apply to the hot plate and saw blade
combination described in the Hotplate folder.  But of course the controller can be
used with anything that operates in a similar way - a toaster oven, or whatever - 
since it basically just switches mains power on and off per a defined sequence.

This sketch was compiled using the Arduino IDE v1.8.8 for the Arduino Nano with the
ATmega328p (old bootloader) processor.

Please see the pdf file and the schematic in this folder for detailed information.

*/

#include <avr/sleep.h>
#include <avr/wdt.h>

const byte segA  = bit(0);                 // Bit values of segments
const byte segB  = bit(1);
const byte segC  = bit(2);
const byte segD  = bit(3);
const byte segE  = bit(4);
const byte segF  = bit(5);
const byte segG  = bit(6);

   //Definitions of the characters used in the "ProfileDisplay" array

const byte charP = segA + segB + segE + segF + segG;
const byte charb = segC + segD + segE + segF + segG;
const byte charL = segD + segE + segF;
const byte charF = segA + segE + segF + segG;
const byte char2 = segA + segB + segD + segE + segG;

//add any others needed for additional profiles

const byte Profiles = 3;                   //Number of reflow profiles defined

   //Array of the two-character display names of the profiles
   //   Pb = leaded, LF = lead-free, P2 = simplified leaded

byte ProfileDisplay[Profiles*2] = {

charP,charb,                               //"Pb" = leaded profile
charL,charF,                               //"LF" = lead-free profile
charP,char2,                               //"P2" = simplified leaded profile

   //add others needed for additional profiles - two characters per profile

};                                         //end array with this

   //Duty cycle steps - seconds, percent  (seconds should be an even number)
   //This is a two-dimensional array of each profile number and its steps
   //Profiles are numbered starting with #1

byte  AutoSteps[Profiles][40]  = {         //allows 20 steps per profile - increase if needed

   //Profile #1 - for leaded paste

{68,100,                                   //number of seconds, duty cycle percentage
20,75,
20,50,
28,25,
76,0,
10,16,
44,100,                                    //ramp up for reflow stage - starts early
100,0,                                     //won't cool fast enough - remove from burner
0,0},                                      //must be last item  - sound buzzer here

   //Profile #2 - for lead-free paste - these values are just place holders.  A lead-free
   //  profile has not yet been developed.  

{10,10,
10,20,
10,16,
10,8,
6,0,
0,0},

   //Profile #3 - this is the "P2" simplified version of Profile #1 for leaded paste.  It only
   // requires the power to be turned on twice, at full power, at specific times, so it does not
   // really need a controller at all, just a stopwatch.

{102,100,                                  //number of seconds, duty cycle percentage
150,0,
46,100,
100,0,                                     //won't cool fast enough - remove from burner
0,0},                                      //must be last item  - sound buzzer here
    
   //Add more as needed for additional profiles
   //   Also change Profiles and ProfileDisplay

};                                         //End Autosteps array with this

   // Set default profile and auto/manual mode on boot

byte PROFILENUM = 1;                       //Select default leaded profile on boot
byte AUTOMODE   = 0;                       //Must start in manual mode on boot
                                           //   Unsafe to start in Automode

   // Pin assignments for I/O (16 in total) - need not be sequential or adjacent

const byte  SSRpin    = A0;                //solid state relay control line

const byte  REDpin    = 9;                 //red LED
const byte  GRNpin    = 10;                //green LED
const byte  BUZZpin   = 11;                //buzzer

   // Display pins

const byte SEGApin    = A5;                //Segments
const byte SEGBpin    = A4;
const byte SEGCpin    = 2;
const byte SEGDpin    = 4;
const byte SEGEpin    = 5;
const byte SEGFpin    = A3;
const byte SEGGpin    = A2;

const byte CACC0pin   = 3;                 //Common anodes or common cathodes
const byte CACC1pin   = 6;

const byte UNUSEDpin0 = 12;                //initialize so they won't float
const byte UNUSEDpin1 = 13;                //D0 and D1 already intialized

   // Encoder pins

const byte CWPIN      = 8;                 //Bit 0 - This pin leads on CW rotation
const byte CCWPIN     = 7;                 //Bit 1 - This pin leads on CCW rotation
const byte SWITCHpin  = A1;                //encoder's momentary switch

   // Definitions for Displays

const byte SEGMENTS  = 7;                  //Change to 8 if using decimal point
const byte UNUSED    = 2;                  //Number of unused pins

   // Array allows pins to be addressed in A-G sequence regardless of pin numbers

byte SEGARRAY[]    = {SEGApin, SEGBpin, SEGCpin, SEGDpin, SEGEpin,SEGFpin, SEGGpin};
byte UNUSEDARRAY[] = {UNUSEDpin0,UNUSEDpin1};

   //  Use these defs for common cathode displays
const byte SEGON     = HIGH;
const byte SEGOFF    = LOW;
const byte CACCON    = LOW;
const byte CACCOFF   = HIGH;

   // Use these defs for common anode displays
//const byte SEGON     = LOW;
//const byte SEGOFF    = HIGH;
//const byte CACCON    = HIGH;
//const byte CACCOFF   = LOW;

   //Segment patterns for numbers

const byte char0 = segA + segB + segC + segD + segE + segF;
const byte char1 = segB + segC;
const byte char3 = segA + segB + segC + segD + segG;
const byte char4 = segB + segC + segF + segG;
const byte char5 = segA + segC + segD + segF + segG;
const byte char6 = segA + segC + segD + segE + segF + segG;
const byte char7 = segA + segB + segC;
const byte char8 = segA + segB + segC + segD + segE + segF + segG;
const byte char9 = segA + segB + segC + segD + segF + segG;
const byte char10  = segA + segD + segG;                     //used to display "100"

   // Array links a value to its character

byte charArray[] = {char0, char1, char2, char3, char4, char5,
                    char6, char7, char8, char9, char10};

byte SEGCOUNT;                             //Segment counter - count up to SEGMENTS value
byte CURSEG;                               //Current segment bit position
byte DIGIT0;                               //Current segment pattern of Digit0
byte DIGIT1;                               //Current segment pattern of Digit1


   //Definitions for Rotary Encoder

const byte ENCODERTYPE   = 0;              //Encoder has pulses = detents per rev
//const byte  ENCODERTYPE  = 1;              //Encoder has pulses = detents/2 per rev
                                             //If tick every other detent, try 1 here
                                             //If two ticks per detent, set it to 0

   // Accumulated change to recognize a tick = number of transitions between detents

const int THRESH  = (4-(2*ENCODERTYPE));   //THRESH = 4 for type 0, 2 for type 1

int  TOTAL;                                //Cumulative transitions: 8 = none
byte INDEX;                                //Index into lookup state table
byte CURCW;                                //Current state of CW encoder pin
byte CURCCW;                               //Current state of CCW encoder pin

   // Encoder state table

int ENCTABLE[]  = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

   //Definitions for SSR duty cycle

const byte ONESTEP = 10;                   //One duty cycle percent step
byte  CHECKCNT;                            //Count before next duty cycle check
byte  DUTY;                                //Duty cycle percentage setting
byte  DUTYCOUNT;                           //Count down from 100% - compare to DUTY
byte  AUTOINDEX;                           //Pointer to next AutoSteps entry
byte  AUTOCOUNT;                           //Count down of current Step time


   //Definitions for encoder's momentary switch

const int   ONESEC    = 500;               //number of 2ms iterations
const int   TENTHSEC  = 50;
const int   QUARTSEC  = 125;
int   BUZZTIME        = 0;                 //time buzzer will be ON - init to zero

const byte  SHORT     = 1;                 //Flag value for short press
const byte  LLONG     = 128;               //Flag value for long press, zero if neither

byte  SWNOW;                               //Current state of switch
byte  SWITCHMODE;                          //Processing mode for switch - 1, 2 or 3
byte  LSFLAG;                              //Press was long or short
int   LONGCNT;                             //Countdown for long press test
int   RELEASECNT;                          //Countdown for release test


   //Definitions for blinking green - profile selected

byte PROCYCLES;                            //profile number = number of flashes
int  PROCOUNT;                             //flash time
int  OFFCOUNT;                             //off time between flash bursts
byte ONFLAG;                               //flash on (1) or off(0)
byte GOBACK = 1;                           //after enter AUTOMODE, short press no longer works

   //Other Definitions

int TEMP;
byte i;

void setup() {

  PROFILENUM  -= 1;                        //Convert selected profile number to zero-based
  OFFCOUNT = ONESEC;                       //setup for blinking green

     //Setup for displays

  for (i = 0; i < SEGMENTS; ++i) {         //Initialize segment pins to OUTPUT, off
    pinMode(SEGARRAY[i], OUTPUT);
    digitalWrite(SEGARRAY[i], SEGOFF);
  }

  for (i = 0; i < UNUSED; ++i)  {          //Initialize unused pins to OUTPUT, LOW
    pinMode(UNUSEDARRAY[i],OUTPUT);
    digitalWrite(UNUSEDARRAY[i],LOW);
  }

  pinMode(CACC0pin, OUTPUT);               //Same for CACC pins
  digitalWrite(CACC0pin, CACCOFF);

  pinMode(CACC1pin, OUTPUT);
  digitalWrite(CACC1pin, CACCOFF);

     //Set at end so first refresh will re-initialize everything

  SEGCOUNT   = SEGMENTS - 1;               //Segment counter - set to end
  CURSEG     = bit(SEGMENTS - 1);          //Bit position of last segment

  DIGIT0    = ProfileDisplay[1+(PROFILENUM*2)]; //Set display to current profile
  DIGIT1    = ProfileDisplay[PROFILENUM*2];

     //Setup for rotary encoder

  pinMode(CWPIN,INPUT_PULLUP);             //Encoder pins = input with pullups
  pinMode(CCWPIN,INPUT_PULLUP);

  TOTAL       = 8;                         //Value at detent with no movement

  if(digitalRead(CWPIN) | digitalRead(CCWPIN)) INDEX = 15;
  else  INDEX = 0;                         //Initialize INDEX to current state

     //Setup for SSR duty cycle

  pinMode(SSRpin,OUTPUT);                  //Init SSRpin to OUTPUT, LOW
  digitalWrite(SSRpin,LOW);

  CHECKCNT    = ONESTEP;                   //Init countdown
  DUTY        = 0;                         //Duty cycle = 0%
  DUTYCOUNT   = 100;                       //Beginning of 2 second cycle
  AUTOINDEX   = 0;                         //Point to beginning of AutoSteps
  AUTOCOUNT   = 1;                         //First interation will load first AutoStep

     //Setup for momentary switch - short, long, repeating long if held down

  pinMode(SWITCHpin,INPUT_PULLUP);         //Init SWITCHpin to input with pullups
  SWITCHMODE  =  1;                        //Mode 1 = waiting for initial keypress
  LSFLAG      =  0;                        //No presses pending

     //Setup for LEDs and buzzer

  pinMode(REDpin,OUTPUT);                  //All these outputs, low
  digitalWrite(REDpin,LOW);

  pinMode(GRNpin,OUTPUT);
  digitalWrite(GRNpin,LOW);

  pinMode(BUZZpin,OUTPUT);
  digitalWrite(BUZZpin,LOW);

  if(AUTOMODE) {
    GOBACK = 0;                            //if boot in automode, disable short press
    BUZZTIME  = QUARTSEC*2;                //also do buzzer
    digitalWrite(BUZZpin, HIGH);
    digitalWrite(GRNpin, HIGH);
  }

     //Set up various power-saving options

  ADCSRA = 0;                              //Turn off ADC
  ACSR = 0x80;                             //Turn off comparator
  SPCR = 0;                                //Turn off SPI
  wdt_disable();                           //Turn off watchdog
  PRR = 0xCF;                              //Turn off all periph clocks except timer0
     

     //Set up alternate interrupt = at 249 on timer0  = 2ms = 500Hz

  cli();                                   // disable interrupts while doing this
  CLKPR = 0x80;                            // change system clock prescaler to 16
  CLKPR = 0x04;                            //  so now 1MHz

  TCCR0A = 0;                              // set entire TCCR0A register to 0
  TCCR0B = 0;                              // same for TCCR0B
  TCNT0  = 0;                              // initialize counter value to 0

  OCR0A = 249;                             // set top of counter
  TIMSK0 &= ~bit(TOIE0);                   // disable overflow interrupt
  TCCR0A |= bit(WGM01);                    // turn on CTC mode
  TCCR0B |= bit(CS01);                     // Set CS01 bit for counter prescaler = 8
  TIMSK0 |= bit(OCIE0A);                   // enable timer compare interrupt
  sei();                                   // enable interrupts

}


   // Loop has only the go-to-sleep code

void loop() {
  set_sleep_mode (SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_cpu();
}

   // All activity carried out in the interrupt service routine

ISR(TIMER0_COMPA_vect) {

     //Process display update
 
     //This section updates the segment driver selection

  digitalWrite(SEGARRAY[SEGCOUNT],SEGOFF); //turn off to make changes - no ghosting
  CURSEG       =  CURSEG << 1;             //shift to next bit position
  SEGCOUNT     += 1;                       //used as index into SEGARRAY
  if (SEGCOUNT == SEGMENTS) {              //if done with last segment, start over
    SEGCOUNT   =  0;                       //re-initialize
    CURSEG     =  1;
  }
     //This section turns the CA/CC pins ON/OFF per the character segment patterns
     //If the CURSEG bit of the DIGITn segment pattern is 1, turn on that CACCn pin

  if (DIGIT0 & CURSEG) digitalWrite(CACC0pin, CACCON);
  else digitalWrite(CACC0pin, CACCOFF);

  if (DIGIT1 & CURSEG) digitalWrite(CACC1pin, CACCON);
  else digitalWrite(CACC1pin, CACCOFF);

  digitalWrite(SEGARRAY[SEGCOUNT], SEGON); //now turn new segment driver on

     //Process rotary encoder

  CURCW     = digitalRead(CWPIN);          //Read the current state of the encoder pins
  CURCCW    = digitalRead(CCWPIN);
  
  INDEX     = INDEX << 2;                  //Shift previous state left 2 bits (0 in)
  if(CURCW) bitSet(INDEX,0);               //If CW is high, set INDEX bit 0
  if(CURCCW) bitSet(INDEX,1);              //If CCW is high, set INDEX bit 1
  INDEX     = INDEX & 15;                  //Mask out all but prev and current

     //INDEX is now a four-bit index into the ENCTABLE state table

  TOTAL     += ENCTABLE[INDEX];

     //Change is +1 for CW, -1 for CCW, 0 for no change or illegal
     //If no change or illegal, move prev back to original position

  if(ENCTABLE[INDEX] == 0)  {
    INDEX    = INDEX >> 2;
    goto     NewDuty;                      //Nothing further to do on encoder
  }

  if(CURCW & CURCCW) goto Tick;            //If now both high, process tick
  if(ENCODERTYPE == 0) goto NewDuty;       //Recognize tick only on both high
  if(CURCW | CURCCW) goto NewDuty;         //No tick if one is high

Tick:                                      //Both high, or both low and Enc-type = 1

  if(TOTAL >= (8 + THRESH)) {              //If TOTAL has the required transitions
    if(DUTY < 100) DUTY  += 1;             //   in either direction, change DUTY
  }

  if(TOTAL <= (8 - THRESH)) {
    if(DUTY > 0)  DUTY  -= 1;
  }

  DIGIT1       = 0;                        //Translate DUTY into 2 chars for display
  DIGIT0       = DUTY;
  while (DIGIT0 > 9) {                     //Subtract 10 until less than 10 remains
    DIGIT0  -= 10;                         //(OK for small numbers - up tp 100 here)
    DIGIT1  += 1;
  }
  DIGIT0    =  charArray[DIGIT0];          //Convert to characters
  DIGIT1    =  charArray[DIGIT1];

  TOTAL = 8;                               //Always reset TOTAL to 8 at detent

     //Process SSR Duty cycle

NewDuty:

  CHECKCNT   -=  1;                        //Decrement 10-count
  if(CHECKCNT > 0) goto ButtonPress;       //If not timed out, leave duty cycle alone
  CHECKCNT    = ONESTEP;                   //Else re-init CHECKCNT
  if(DUTYCOUNT > DUTY)  {                  //If countdown hasn't reached DUTY, turn OFF
    digitalWrite(SSRpin,LOW);
    digitalWrite(REDpin,LOW);
  }
  else {                                   //If at or below DUTY, turn ON
    digitalWrite(SSRpin,HIGH);
    digitalWrite(REDpin,HIGH);
  }

  DUTYCOUNT  -=  1;
  if(DUTYCOUNT > 0) goto ButtonPress;      //If not zero yet, we're done here
  DUTYCOUNT   = 100;                       //Re-init on zero

  if(AUTOMODE == 0) goto ButtonPress;      //If not auto mode, we're done here

  AUTOCOUNT   -= 1;                        //Decrement count for this step
  if(AUTOCOUNT > 0) goto ButtonPress;      //Nothing to be done till zero
  if(AutoSteps[PROFILENUM][AUTOINDEX] == 0) {        //If no further steps in AutoSteps array
    AUTOMODE  = 0;                         //Turn off auto mode
    digitalWrite(GRNpin,LOW);              //Turn off green LED
                                           //But leave duty cycle running at last setting
    BUZZTIME  = (ONESEC*5);                //also do long buzzer
    digitalWrite(BUZZpin, HIGH);
  }

  else  {
    AUTOCOUNT = AutoSteps[PROFILENUM][AUTOINDEX]>>1; //Load next Step data into count and duty
    DUTY      = AutoSteps[PROFILENUM][AUTOINDEX+1];
    AUTOINDEX += 2;                        //Increment pointer for next time
    if(DUTY > 100)  DUTY = 100;            //Limit duty cycle to 100%

    DIGIT1       = 0;                      //Translate DUTY into 2 chars for display
    DIGIT0       = DUTY;
    while (DIGIT0 > 9) {                   //Subtract 10 until less than 10 remains
      DIGIT0  -= 10;                       //(OK for small numbers - up tp 100 here)
      DIGIT1  += 1;
    }
    DIGIT0    =  charArray[DIGIT0];        //Convert to characters
    DIGIT1    =  charArray[DIGIT1];
  }
ButtonPress:

  SWNOW     = digitalRead(SWITCHpin);      //Read switch pin (low means pressed)
  if(SWITCHMODE==2) goto PressDetected;    //Mode 2 = waiting for release or Long
  if(SWITCHMODE==3) goto Wait4Release;     //Mode 3 = wait for release after long
                                           //            or repeat

  if(SWNOW == 0) {                         //Mode 1 = wait for initial press
    LONGCNT    = ONESEC;                   //Long press if not released by 1 sec
    RELEASECNT = TENTHSEC;                 //Short press if released for 0.1 sec
    SWITCHMODE = 2;                        //Now in Mode 2
  }
  goto ServiceBuzz;                        //Nothing more to do

PressDetected:                             //Mode 2 - check for release or long press

  if(SWNOW) {                              //If not pressed now
    RELEASECNT  -= 1;                      //Is release long enough?
    if(RELEASECNT == 0) {                  //Yes - set LSFLAG and restore Mode 1
      LSFLAG   = SHORT;
      SWITCHMODE = 1;
      goto ButtonDone;                     //Process short flag
    }
  }
  else RELEASECNT = TENTHSEC;              //If pressed - reinit release count

  LONGCNT   -=  1;                         //Check for long press
  if(LONGCNT > 0) goto ButtonDone;         //Not yet

  LSFLAG     = LLONG;                      //Recongnize long press and repeat if held
  SWITCHMODE = 3;
  goto   ButtonDone;

Wait4Release:                              //Mode 3 - wait for release after long press

  if(SWNOW == 0) {                         //If still pressed
    RELEASECNT   = TENTHSEC+1;             //Init release count
  }
  RELEASECNT  -= 1;                        //If not pressed, test release count
  if(RELEASECNT == 0) SWITCHMODE = 1;

ButtonDone:                                //Process short or long press if any

  if(LSFLAG == 0)  goto ServiceBuzz;       //If no press, we're done here
  if(LSFLAG == 128) {
    LSFLAG  = 0;
    goto  ItWasLong;
  }

ItWasShort:                                //Short flag = 1

  LSFLAG  = 0;
  if(GOBACK == 0) goto ServiceBuzz;        //Only change profile before entering automode
  PROFILENUM  +=1;                         //Cycle to next profile
  if(PROFILENUM == Profiles) PROFILENUM=0;
  DIGIT0=ProfileDisplay[1+(PROFILENUM*2)]; //Set display to new profile
  DIGIT1=ProfileDisplay[PROFILENUM*2];
  goto  ServiceBuzz;

ItWasLong:                                 //Long flag = 128
                                           //Long press toggles auto mode setting

  if((AUTOMODE == 1)                       //if mode is auto,
    || (AutoSteps[PROFILENUM][AUTOINDEX])  //or there are more steps
    || (AUTOCOUNT)) {                      //or the current step isn't finished

    AUTOMODE ^= 1;                         //then toggle automode
    if(AUTOMODE) {
      digitalWrite(GRNpin,HIGH);           //and set green LED as needed
      GOBACK = 0;                          //short press and blinking green no longer work
    }
    else digitalWrite(GRNpin,LOW);
    BUZZTIME  = QUARTSEC*2;                //also do buzzer
    digitalWrite(BUZZpin, HIGH);
  }

ServiceBuzz:

  if(BUZZTIME)  {                          //Leave buzzer on for the count
    BUZZTIME   -= 1;
    if(BUZZTIME == 0) digitalWrite(BUZZpin,LOW);
  }

     //Blinking green LED to indicate selected profile

  if(GOBACK) {
    if(OFFCOUNT) {
      OFFCOUNT -= 1;
      if(OFFCOUNT == 0) {
        PROCYCLES = PROFILENUM + 1;
        PROCOUNT = QUARTSEC;
        ONFLAG = 1;
        digitalWrite(GRNpin,HIGH);
      }
    }
    else {
      PROCOUNT -= 1;                       //if OFFCOUNT == 0, PROCOUNT > 0
      if(PROCOUNT == 0) {
        if(ONFLAG == 0) PROCYCLES -= 1;
        if(PROCYCLES == 0) {
          OFFCOUNT = ONESEC;
        }
        else {
          ONFLAG ^= 1;
          PROCOUNT = QUARTSEC;
          digitalWrite(GRNpin, !digitalRead(GRNpin));
        }
      }
    }
  }
}
