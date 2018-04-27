// ATTINY "Smart" Wake-up routine
// Adjustable active time
// Triggerable Shutdown
// Activatable by either watchdog (Time elapsed) or PB0 or PB3 Pin Change
// Code copied and modified from:
// 1. http://blog.onlinux.fr
// 2. Johannes S. (joh.raspi - forum-raspberrypi.de)
// 
//+  Pinout:                                             
//+                   +-------------+                     
//+                 +-+ (PB5)   VCC +-+  VBAT +           
//+                   | (Reset)     |                     
//+                   |             |                     
//+  WAKEUP2        +-+ PB3     PB2 +-+  VREG/LDO Enable (OUTPUT)    
//+  (INPUT)          |                     
//+                   |             |                     
//+  VREG Shutdown  +-+ PB4     PB1 +-+  WAKEUP Switch State (OUTPUT)
//+  (INPUT)          |             |                     
//+                   |             |                     
//+  GND            +-+ GND     PB0 +-+  WAKEUP Switch (INPUT)       
//+                   +-------------+                     
//+  
//+  Pin Function:
//+  PB0 - WAKEUP Switch       - Wakes ATTINY from Deep Sleep when state changes
//+  PB1 - WAKEUP Switch State - Forward Wakeup Switch over this pin for upstream components
//+  PB2 - VREG Enable         - Connect to Power Regulator (LDO) to enable power to upstream components
//+  PB3 - WAKEUP2             - Second Wakeup pin
//+  PB4 - VREG Shutdown       - A short signal on this pin will turn of the power regulator
//+
//+  Functies:
//+  1. -> The ATTINY is woken up after a Pin Change Interrupt from Pin PB0 or PB3, or if the Watchdog Timer expires
//+  2. -> ATTINY sets PB2 HIGH to activate the Power Regulator (And thereby powers the ESP
//+  3. -> Now ATTINY will wait for a signal from the ESP on pin PB4 to shutdown the Power Regulator.
//         In the meantime the state from PB0 is still being outputted on PB1.
//+  4. -> When after the expiration of "timerInterval" number of seconds no signal is received on PB4 the Power Regulator will be shut down (PB2 goes LOW)
//+        When the power is turned off, also PB1 is set to LOW again.
//+  5. -> Now the ATTINY goes to deep sleep. waiting for the next interrupt to wake the chip.
//+        The power consumption of the ATTiny lies around ~300nA & 3.8V. Please check the datasheet for exact details.
//+
//**************************************************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//
// W H Y  V O L A T I L E ?
//
// Why volatile?
// ---->
// https://www.arduino.cc/en/Reference/Volatile
// volatile keyword:
// Specifically, it directs the compiler to load the variable from RAM and not from a storage register,
// which is a temporary memory location where program variables are stored and manipulated. 
// Under certain conditions, the value for a variable stored in registers can be inaccurate.
 
volatile boolean f_wdt = 1;


// How many seconds does the Power Regulator need to stay on?
const unsigned long timerInterval = 1800;
// How many seconds does the ATTINY sleep if there is no pin change on PB0 and PB3 (Multiple of 8 seconds)?
const unsigned long sleepDuration = 16;
 
// Setup Vatiables
const int WAKEUP_SWITCH = 0;
const int WAKEUP_SW2 = 3;
const int WAKEUP_SWITCH_STATE = 1;
const int VREG_ENABLE = 2;
const int VREG_SHTDN = 4;
unsigned long start_time;
boolean goto_sleep = true;
 

/******************************************************************/
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
 
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
} 
 
void system_sleep() {
  //cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF to save power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here, waiting for interrupt
  sleep_disable();                     // System continues execution here when watchdog timed out 
  //sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON to make it available again in the code.
}
 
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag to regocnize watchdog had timed out
}
 
 
void setup()
{
  setup_watchdog(9);
  // Configure GPIO's
  pinMode(VREG_ENABLE, OUTPUT);
  pinMode(VREG_SHTDN, INPUT);
  pinMode(WAKEUP_SWITCH_STATE, OUTPUT);
  pinMode(WAKEUP_SWITCH, INPUT);
  digitalWrite(WAKEUP_SWITCH, HIGH);  // Activate internal pullup resistor (Prefer to use NO/NC Reed Switch because this lowers power consumption) 
  PCMSK |= ((1<<PCINT0) | (1<<PCINT3));  //                     Pin Change Mask Register
  ADCSRA &= ~(1<<ADEN);    // Turn off AD Converter as I do not use it in this project
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts  General Interrupt Flag Register
  GIMSK  |= bit (PCIE);    // enable pin change interrupts      General Interrupt Mask Register
  sei();                   // enable interrupts
}
 
ISR (PCINT0_vect){}
 
void loop()
{
  if (goto_sleep == true){  
    // Disable power switch
    digitalWrite(VREG_ENABLE, LOW);
    // Turn off Forward for REED Switch
    digitalWrite(WAKEUP_SWITCH_STATE, LOW);
    for (int i = 0; i < (sleepDuration/8); i++){
      system_sleep();
      if (f_wdt < 1) {
        i = (sleepDuration/8);
      }
    }
    //Woke up from Pin Change interrupt or Watchdog Timed Out
    // Spannungsregler einschalten
    digitalWrite(VREG_ENABLE, HIGH);
    // (Neue) Startzeit einlesen
    start_time = millis();
    // Genug geschlafen
    goto_sleep = false;   
    if ( f_wdt > 0) {  // watchdog signal
      f_wdt=0;         //reset flag
    }
  }

  //++++++++++++++++++++++++++++++++++++++++
  // Forward PB0 state to PB1
  //++++++++++++++++++++++++++++++++++++++++
  digitalWrite(WAKEUP_SWITCH_STATE, digitalRead(WAKEUP_SWITCH));
  
  //++++++++++++++++++++++++++++++++++++++++
  // Was the system on long enough?
  //++++++++++++++++++++++++++++++++++++++++
  if (millis()-start_time >= timerInterval*1000) 
  {
    // Need to go to sleep again
    goto_sleep = true;
  }
  
  //++++++++++++++++++++++++++++++++++++++++
  // Shutdown signal received?
  //++++++++++++++++++++++++++++++++++++++++
  if (digitalRead(VREG_SHTDN) == 1) 
  {
    // Try to detect false positive
    delay(100);
    // Wait if the High signal is still detected.
    while (digitalRead(VREG_SHTDN) == 1) {};
    goto_sleep = true;
  }  
}
