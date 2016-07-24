/*
  * drafted by Jeremy VanDerWal ( jjvanderwal@gmail.com ... www.jjvanderwal.com )
  * drafted by Dylan VanDerWal (dylanjvanderwal@gmail.com)
  * code is writen to work with the stalker 2.3 board with a mdot lorawan module on the xbee 
  * GNU General Public License .. feel free to use / distribute ... no warranties
*/
//common libraries
#include <LoRaAT.h>                     //Include LoRa AT libraray
#include <SoftwareSerial.h>             //Software serial for debug
#include <avr/sleep.h>                  // this is for low power sleep
#include <avr/power.h>                  // this is for low power sleep
#include <Wire.h>                       // required for sleep/low power
#include "DS3231.h"                     // RTC
#include <math.h>                       // required for rounding some of the data from the analog readout of the battery

//sensor specific libraries



//define and initialize some of the pins/types and setup variables
/*generic requirements*/
SoftwareSerial debugSerial(10, 11);     // RX, TX
LoRaAT mdot(0, &debugSerial);           //Instantiate a LoRaAT object
int POWER_BEE = 5;                      // power_bee pin is 5 to turn on and off radio
int responseCode;                       //define the responsecode for joining the lora network
DS3231 RTC;                             // Create the DS3231 RTC interface object
static DateTime interruptTime;          // this is the time to interupt sleep

/* sensor specific preparation */


// setup the start
void setup() {
                                                 
   /* CHANGE THIS SECTION TO EDIT SENSOR INITIALIZATION */ 

  /* END OF SECTION TO EDIT SENSOR INITIALIZATION */ 
                                                 
  /*setup serial ports for sending data and debugging issues*/
  debugSerial.begin(38400);             //Debug output. Listen on this ports for debugging info
  mdot.begin(38400);                    //Begin (possibly amongst other things) opens serial comms with MDOT

  /*misc setup for low power sleep*/
  pinMode(POWER_BEE, OUTPUT);           //set the pinmode to turn on and off the power to the radio

  /*Initialize INTR0 for accepting interrupts */
  PORTD |= 0x04; 
  DDRD &=~ 0x04;
  
  Wire.begin();    
  RTC.begin();

  attachInterrupt(0, INT0_ISR, FALLING); //Only LOW level interrupt can wake up from PWR_DOWN
    
  DateTime start = RTC.now();                   //get the current time
  debugSerial.println(String(start.hour())+":"+String(start.minute())+":"+String(start.second()));
  interruptTime = DateTime(start.get() + 60);  //Add 5 mins in seconds to start time

  JoinLora(); //start and join the lora network
}

//start the application
void loop () 
{
  ///////////////// START the application ////////////////////////////
  
  String postData;                                           //define the initial post data

  /* CHANGE THIS SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

  /* END OF SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

  int BatteryValue = analogRead(A7);                                // read the battery voltage
  float voltage = BatteryValue * (3.7 / 1024)* (10+2)/2;            //Voltage devider
  String Volts = String(round(voltage*100)/100);                    //get the voltage 
  postData = ("BT:" + String(RTC.getTemperature())+ ",CH:" + String(read_charge_status()) +",V:" + Volts);   //append it to the post data -- internal temperature, charging status, & voltage
  debugSerial.println(postData);                                    //for debugging purposes, show the data
  responseCode = mdot.sendPairs(postData);                      // post the data

  ////////////////// Application finished... put to sleep ///////////////////
   //setup the interupt for sleep
  RTC.clearINTStatus();                                                                         //This function call is  a must to bring /INT pin HIGH after an interrupt.
  RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());     // set the interrupt at (h,m,s)
  attachInterrupt(0, INT0_ISR, FALLING);                                                            //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt

  debugSerial.println("Free ram:"+String(freeRam()));
  debugSerial.println(String(interruptTime.hour())+":"+String(interruptTime.minute())+":"+String(interruptTime.second()));
  SleepNow();
  debugSerial.println(String(interruptTime.hour())+":"+String(interruptTime.minute())+":"+String(interruptTime.second()));
  DateTime start = RTC.now();                   //get the current time
  debugSerial.println(String(start.hour())+":"+String(start.minute())+":"+String(start.second()));
  interruptTime = DateTime(interruptTime.get() + 60);  //decide the time for next interrupt, configure next interrupt  

} 

//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
  detachInterrupt(0); 
}


//define the sleepnow function
void SleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  //Power Down routines
  digitalWrite(POWER_BEE, LOW);                //turn the xbee port oFF -- turn on the radio
  delay(1000);                                  // allow radio to power down
  
  cli(); 
  sleep_enable();                           // Set sleep enable bit
  sleep_bod_disable();                      // Disable brown out detection during sleep. Saves more power
  sei();
    
  debugSerial.println("\nSleeping");        //debug: print the system is going to sleep
  delay(10);                                //This delay is required to allow print to complete
  
  //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
  power_all_disable();                      //This shuts down ADC, TWI, SPI, Timers and USART
  sleep_cpu();                              // Sleep the CPU as per the mode set earlier(power down) 
  
  /* WAIT FOR INTERUPT */
 
  //wake up the system
  sleep_disable();                          // Wakes up sleep and clears enable bit. Before this ISR would have executed
  power_all_enable();                       //This shuts enables ADC, TWI, SPI, Timers and USART
  delay(1000);                                //This delay is required to allow CPU to stabilize
  JoinLora(); //start and join the lora network
  delay(1000);
  debugSerial.println("Awake from sleep");  //debug: print the system is awake 

}

// check ram remaining
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


//start and join the lora network
void JoinLora() {
  /* start the radio */
  digitalWrite(POWER_BEE, HIGH);                //turn the xbee port on -- turn on the radio
  delay(1000);                                  // allow radio to power up
  do {                                          //join the lora network
    responseCode = mdot.join();                 //join the network and get the response code
  } while (responseCode != 0);                  //continue if it joins
}

//get the charging status
unsigned char read_charge_status(void) {
  unsigned char CH_Status=0;
  unsigned int ADC6=analogRead(6);
  if(ADC6>900) {
    CH_Status = 0;//sleeping
  } else if(ADC6>550) {
    CH_Status = 1;//charging
  } else if(ADC6>350) {
    CH_Status = 2;//done
  } else {
    CH_Status = 3;//error
  }
  return CH_Status;
}
