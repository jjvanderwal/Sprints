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
#include "DHT.h"


//define and initialize some of the pins/types and setup variables
/*generic requirements*/
SoftwareSerial debugSerial(10, 11);     // RX, TX
LoRaAT mdot(0, &debugSerial);           //Instantiate a LoRaAT object
int POWER_BEE = 5;                      // power_bee pin is 5 to turn on and off radio
int responseCode;                       //define the responsecode for joining the lora network
DS3231 RTC;                             // Create the DS3231 RTC interface object
static DateTime interruptTime;          // this is the time to interupt sleep

//DHT requirements
#define DHTPIN0 8       //Defines where the dht is located
#define DHTPIN1 9       //Defines where the dht is located
#define DHTPIN2 12      //Defines where the dht is located
#define DHTPIN3 13      //Defines where the dht is located

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht0(DHTPIN0, DHTTYPE);
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);

// setup the start
void setup() {

   /* CHANGE THIS SECTION TO EDIT SENSOR INITIALIZATION */ 
    dht0.begin();
    dht1.begin();
    dht2.begin();
    dht3.begin();
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
    
  DateTime start = RTC.now();                   //get the current time
  interruptTime = DateTime(start.get() + 60);  //Add 5 mins in seconds to start time

  JoinLora(); //start and join the lora network
}

//start the application
void loop () 
{
  ///////////////// START the application ////////////////////////////
  
  String postData;                                           //define the initial post data

  /* CHANGE THIS SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

      postData = ("T0:" + String(dht0.readTemperature()) + ",H0:" + String(dht0.readHumidity()));
      postData += (",T1:" + String(dht1.readTemperature()) + ",H1:" + String(dht1.readHumidity()));
      debugSerial.println(postData);                                    //for debugging purposes, show the data
      responseCode = mdot.sendPairs(postData);                      // post the data
      
      postData = ("T2:" + String(dht2.readTemperature()) + ",H2:" + String(dht2.readHumidity()));
      postData += (",T3:" + String(dht3.readTemperature()) + ",H3:" + String(dht3.readHumidity()));
      debugSerial.println(postData);                                    //for debugging purposes, show the data
      responseCode = mdot.sendPairs(postData);                      // post the data
      
  /* END OF SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

  postData = ("BT:" + String(RTC.getTemperature()));                //append the temp in the box
  postData += (",CH:" + String(read_charge_status()));              //append the charge status
  int BatteryValue = analogRead(A7);                                // read the battery voltage
  float voltage = BatteryValue * (3.7 / 1024)* (10+2)/2;            //Voltage devider
  String Volts = String(round(voltage*100)/100);                    //get the voltage 
  postData += (",V:" + Volts);                                      //append it to the post data voltage
  debugSerial.println(postData);                                    //for debugging purposes, show the data
  responseCode = mdot.sendPairs(postData);                      // post the data

  ////////////////// Application finished... put to sleep ///////////////////
  SleepNow();

} 

//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
  detachInterrupt(0); 
  DateTime start = RTC.now();                   //get the current time
  interruptTime = DateTime(start.get() + 60); //decide the time for next interrupt, configure next interrupt  
}

//define the sleepnow function
void SleepNow() {
  debugSerial.println("\nSleeping");        //debug: print the system is going to sleep
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // sleep mode is set here
  sleep_enable();          // enables the sleep bit in the mcucr register
  //setup the interupt for sleep
  RTC.clearINTStatus();                                                                         //This function call is  a must to bring /INT pin HIGH after an interrupt.
  RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());     // set the interrupt at (h,m,s)
  attachInterrupt(0, INT0_ISR, LOW);                                                            //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
  sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
  //detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                             // wakeUpNow code will not be executed
                             // during normal running time.
  debugSerial.println("Awake from sleep");  //debug: print the system is awake
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
