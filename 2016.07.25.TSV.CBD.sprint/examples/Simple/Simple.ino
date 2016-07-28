/*
  * drafted by Jeremy VanDerWal ( jjvanderwal@gmail.com ... www.jjvanderwal.com )
  * drafted by Dylan VanDerWal (dylanjvanderwal@gmail.com)
  * code is writen to work with the stalker 2.3 board with a mdot lorawan module on the xbee 
  * GNU General Public License .. feel free to use / distribute ... no warranties
  * 
  * Pin D5 - powers up/down xbee slot
  * Pin D2 - interupt to allow sleep
  * Pin A6 - defines the charging state
  * Pin A7 - define the battery voltage
*/

//common libraries
#include <LoRaAT.h>                     //Include LoRa AT libraray
#include <SoftwareSerial.h>             //Software serial for debug
#include <avr/sleep.h>                  //For low power sleep
#include <avr/power.h>                  //For low power sleep
#include <Wire.h>
#include "DS3231.h"                     //RTC
#include <math.h>                       //For rounding some of the data from the analog readout of the battery

//define and initialize some of the pins/types and setup variables
SoftwareSerial debugSerial(10, 11);     //RX, TX
LoRaAT mdot(0, &debugSerial);           //Instantiate a LoRaAT object
int POWER_BEE = 5;                      //POWER_BEE pin is 5 to turn on and off radio
DS3231 RTC;                             //Create the DS3231 RTC interface object
static DateTime interruptTime;          //Time to interupt sleep

// setup the start
void setup() {                                               
  /*setup serial ports for sending data and debugging issues*/
  debugSerial.begin(38400);             //Debug output. Listen on this ports for debugging info
  mdot.begin(38400);                    //Begin (possibly amongst other things) opens serial comms with MDOT

  /*misc setup for low power sleep*/
  pinMode(POWER_BEE, OUTPUT);           //Set the pinmode to turn on and off the power to the radio

  /*Initialize INTR0 for accepting interrupts */
  PORTD |= 0x04; 
  DDRD &=~ 0x04;
  
  Wire.begin();    
  RTC.begin();

  //start the lora network
  JoinLora();                            //start and join the lora network
}

//start the application
void loop () 
{
  int responseCode = -1;
  String postData = "";                                         //define the initial post data

  //Build the message to send:
  String rtcTemp = F("BT:");
  postData = rtcTemp;
  RTC.convertTemperature();                                     //Convert current RTC temperature into registers
  postData += RTC.getTemperature();

  String atTemp = F(",AT:");
  postData += atTemp;
  postData += GetTemp();                                        //Get the temperature from the ATmega

  int BatteryValue = analogRead(A7);                            // read the battery voltage
  float voltage = BatteryValue * (3.7 / 1024)* (10+2)/2;        //Voltage devider
  
  String volts = F(",V:");
  postData += volts;
  postData += String(round(voltage*100)/100);

  String charge = F(",CH:");
  postData += charge;
  postData += String(read_charge_status());
  
  
  debugSerial.println(postData);                                //for debugging purposes, show the data
  responseCode = mdot.sendPairs(postData);                      // post the data

  //Debug feedback for the developer to double check what the result of the send
  debugSerial.println("MAIN  : send result: " + String(responseCode));
  
  ////////////////// Application finished... put to sleep ///////////////////
   //setup the interupt for sleep
  RTC.clearINTStatus();                                                                             //This function call is  a must to bring /INT pin HIGH after an interrupt.
  RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());         // set the interrupt at (h,m,s)
  attachInterrupt(0, INT0_ISR, FALLING);                                                            //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt

  //Debug feedback for the user to double check how memory usage is being handled
  debugSerial.print(F("Free ram: "));
  debugSerial.println(String(freeRam()));
    
  DateTime start = RTC.now();                                                                       //Get the current time
  interruptTime = DateTime(start.get() + 60);                                                       //Set the alarm clock, based on current time

  //Debug feedback for the user to double check what the time is, and when Arduino will wake up
  debugSerial.print(F("Time now     : "));
  debugSerial.println(String(start.hour())+":"+String(start.minute())+":"+String(start.second()));
  debugSerial.print(F("Alarm set for: "));
  debugSerial.println(String(interruptTime.hour())+":"+String(interruptTime.minute())+":"+String(interruptTime.second()));
  
  SleepNow();
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
  digitalWrite(POWER_BEE, LOW);             //turn the xbee port oFF -- turn on the radio
  delay(1000);                              // allow radio to power down
  
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
  int joinLimit = 0;
  int responseCode = -1;
  /* start the radio */
  digitalWrite(POWER_BEE, HIGH);                //turn the xbee port on -- turn on the radio
  delay(1000);                                  // allow radio to power up
  do {                                          //join the lora network
    responseCode = mdot.join();                 //join the network and get the response code
    
    //Debug feedback for the developer to double check the result of the join() instruction
    debugSerial.print(F("SETUP : Join result: "));
    debugSerial.println(String(responseCode));

    delay(900);
    
  } while (responseCode != 0 && joinLimit++ < 5);                  //continue if it joins
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

/*--- GetTemp() ------------------------------------------------------------------------
  Most new AVR chips (used in the Arduino) have an internal temperature sensor. It is not
  often used, since it is not accurate.

  For more information see: http://playground.arduino.cc/Main/InternalTemperatureSensor

  This function returns the temperature in degrees Celsius.
  --------------------------------------------------------------------------------------*/
double GetTemp(void) {
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}
