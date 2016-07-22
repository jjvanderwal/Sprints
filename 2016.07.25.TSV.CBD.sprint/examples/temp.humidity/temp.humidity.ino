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
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


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
  pinMode(4,INPUT);                     //extern power
  
  Wire.begin();    
  RTC.begin();
  attachInterrupt(0, INT0_ISR, LOW); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  DateTime start = RTC.now();                   //get the current time
  interruptTime = DateTime(start.get() + 300);  //Add 5 mins in seconds to start time

}

//start the application
void loop () 
{
  ///////////////// START the application ////////////////////////////
  
  String postData = ("");                                           //define the initial post data

  /* CHANGE THIS SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 
      float h0 = dht0.readHumidity();
      float t0 = dht0.readTemperature();
      float h1 = dht1.readHumidity();
      float t1 = dht1.readTemperature();
      float h2 = dht2.readHumidity();
      float t2 = dht2.readTemperature();
      float h3 = dht3.readHumidity();
      float t3 = dht3.readTemperature();

      postData += ("Temp 0:" + String(t0) + ",Humid 0:" + String(h0));
      postData += (,"Temp 1:" + String(t1) + ",Humid 1:" + String(h1));
      postData += (,"Temp 2:" + String(t2) + ",Humid 2:" + String(h2));
      postData += (,"Temp 3:" + String(t3) + ",Humid 3:" + String(h3));
  /* END OF SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

  String CHstatus = String(read_charge_status());                   //read the charge status
  int BatteryValue = analogRead(A7);                                // read the battery voltage
  float voltage = BatteryValue * (1.1 / 1024)* (10+2)/2;            //Voltage devider
  String Volts = String(round(voltage*100)/100);                    //get the voltage 
  postData += (",V:" + Volts + ",CH:" + CHstatus);                  //append it to the post data

  debugSerial.println(postData);                                    //for debugging purposes, show the data

  PostData(postData);                                               // post the data to the lora network
  
  RTC.clearINTStatus();                                                                         //This function call is  a must to bring /INT pin HIGH after an interrupt.
  RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());     // set the interrupt at (h,m,s)
  attachInterrupt(0, INT0_ISR, LOW);                                                            //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt

  ////////////////// Application finished... put to sleep ///////////////////
  
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
        
  //Power Down routines
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
  delay(10);                                //This delay is required to allow CPU to stabilize
  debugSerial.println("Awake from sleep");  //debug: print the system is awake 
  
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
 
} 

//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
  detachInterrupt(0); 
  interruptTime = DateTime(interruptTime.get() + 300);  //decide the time for next interrupt, configure next interrupt  
}


//this is the setup to post data
void PostData(String str2post) {
  digitalWrite(POWER_BEE, HIGH);                //turn the xbee port on -- turn on the radio
  delay(1000);                          // allow radio to power up
  do {                                  //join the lora network
    responseCode = mdot.join();         //join the network and get the response code
    //delay(10000);
  } while (responseCode != 0);          //continue if it joins

  char postDataChar[100];                       // initialize a string array for posting data
  str2post.toCharArray(postDataChar,99);        //convert string to char array
  responseCode = mdot.sendPairs(postDataChar);  // post the data

  debugSerial.println("posted");                // debugging: desplay the data was posted
  
  digitalWrite(POWER_BEE, LOW);         //turn the xbee port off -- turn the radio off
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
