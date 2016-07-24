/*
  * drafted by Jeremy VanDerWal ( jjvanderwal@gmail.com ... www.jjvanderwal.com )
  * drafted by Dylan VanDerWal (dylanjvanderwal@gmail.com)
  * code is writen to work with the stalker 2.3 board with a mdot lorawan module on the xbee 
  * GNU General Public License .. feel free to use / distribute ... no warranties
  * 
  * pin D5 is used to power on and off the radio
  * 
*/
//common libraries
#include <LoRaAT.h>                     //Include LoRa AT libraray
#include <SoftwareSerial.h>             //Software serial for debug
#include <avr/sleep.h>                  // this is for low power sleep
#include <Wire.h>                       // required for sleep/low power
#include "DS3231.h"                     // RTC
#include <math.h>                       // required for rounding some of the data from the analog readout of the battery

//sensor specific libraries
#include <OneWire.h>
#include <DallasTemperature.h>

//define and initialize some of the pins/types and setup variables
/*generic requirements*/
SoftwareSerial debugSerial(10, 11);     // RX, TX
LoRaAT mdot(0, &debugSerial);           //Instantiate a LoRaAT object
int POWER_BEE = 5;                      // power_bee pin is 5 to turn on and off radio
int responseCode;                       //define the responsecode for joining the lora network
DS3231 RTC;                             // Create the DS3231 RTC interface object
static DateTime interruptTime;          // this is the time to interupt sleep

/*one wire temperature*/
OneWire oneWire0(7);                    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(8);                    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire2(12);                    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)

DallasTemperature sensor0(&oneWire0);    // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensor1(&oneWire1);    // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensor2(&oneWire2);    // Pass our oneWire reference to Dallas Temperature.

/* these are the addresses to the DS18b20s */
DeviceAddress Add0 = {0x28, 0x6A, 0x37, 0x08, 0x03, 0x00, 0x00, 0x3F}; //closest sensor
DeviceAddress Add1 = {0x28, 0xD6, 0xE4, 0xE1, 0x06, 0x00, 0x00, 0xBA};
DeviceAddress Add2 = {0x28, 0x7B, 0xD8, 0xCA, 0x06, 0x00, 0x00, 0xED};
DeviceAddress Add3 = {0x28, 0xDB, 0x6D, 0xCB, 0x06, 0x00, 0x00, 0xA3};
DeviceAddress Add4 = {0x28, 0x27, 0x48, 0xCC, 0x06, 0x00, 0x00, 0x00};
DeviceAddress Add5 = {0x28, 0xBC, 0x95, 0xCA, 0x06, 0x00, 0x00, 0xF2};
DeviceAddress Add6 = {0x28, 0xB0, 0x58, 0xCC, 0x06, 0x00, 0x00, 0x48};
DeviceAddress Add7 = {0x28, 0x64, 0x18, 0xCC, 0x06, 0x00, 0x00, 0xB9}; //furthest sensor

// setup the start
void setup() {
                                                 
  /*setup serial ports for sending data and debugging issues*/
  debugSerial.begin(38400);             //Debug output. Listen on this ports for debugging info
  mdot.begin(38400);                    //Begin (possibly amongst other things) opens serial comms with MDOT

  /*misc setup for low power sleep*/
  pinMode(POWER_BEE, OUTPUT);           //set the pinmode to turn on and off the power to the radio

  /*start sensors*/
  sensor0.begin();
  sensor1.begin();
  sensor2.begin();

  /*Initialize INTR0 for accepting interrupts */
  PORTD |= 0x04; 
  DDRD &=~ 0x04;
  //pinMode(4,INPUT);                     //extern power
 
  Wire.begin();    
  RTC.begin();
  //attachInterrupt(0, INT0_ISR, LOW); 
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  DateTime start = RTC.now();                   //get the current time
  interruptTime = DateTime(start.get() + 60);  //Add 5 mins in seconds to start time

  JoinLora(); //start and join the lora network
}
  
//start the application
void loop () 
{
 
  ///////////////// START the application ////////////////////////////
  char postDataChar[100];                       // initialize a string array for posting data
  String postData;                              // this is what we use to hold data to post
  
  /* CHANGE THIS SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 
  //Read Sensor 0
  sensor0.requestTemperatures();
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  
  //Collect data
  postData = ("T0:" + String(sensor0.getTempC(Add0)) + ",T1:" + String(sensor0.getTempC(Add1)));
  postData += (",T2:" + String(sensor0.getTempC(Add2)) + ",T3:" + String(sensor1.getTempC(Add3)));
  debugSerial.println(postData);                                    //for debugging purposes, show the data
  postData.toCharArray(postDataChar,99);                            //convert string to char array
  responseCode = mdot.sendPairs(postDataChar);                      // post the data
  
  postData = ("T4:" + String(sensor1.getTempC(Add4)) + ",T5:" + String(sensor1.getTempC(Add5)));
  postData += (",T6:" + String(sensor2.getTempC(Add6)) + ",T7:" + String(sensor2.getTempC(Add7)));
  debugSerial.println(postData);                                    //for debugging purposes, show the data
  postData.toCharArray(postDataChar,99);                            //convert string to char array
  responseCode = mdot.sendPairs(postDataChar);                      // post the data
      
  /* END OF SECTION TO EDIT SENSOR DATA BEING COLLECTED */ 

  postData = ("BT:" + String(RTC.getTemperature()));                //append the temp in the box
  postData += (",CH:" + String(read_charge_status()));              //append the charge status
  int BatteryValue = analogRead(A7);                                // read the battery voltage
  float voltage = BatteryValue * (3.7 / 1024)* (10+2)/2;            //Voltage devider
  String Volts = String(round(voltage*100)/100);                    //get the voltage 
  postData += (",V:" + Volts);                                      //append it to the post data voltage
  debugSerial.println(postData);                                    //for debugging purposes, show the data
  postData.toCharArray(postDataChar,99);                            //convert string to char array
  responseCode = mdot.sendPairs(postDataChar);                      // post the data


  ////////////////// Application finished... put to sleep ///////////////////
  SleepNow();



} 

//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
  detachInterrupt(0); 
  interruptTime = DateTime(interruptTime.get() + 60);  //decide the time for next interrupt, configure next interrupt  
}

//define the sleepnow function
void SleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
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
 


  /*
   * 
  delay(5000);
  digitalWrite(POWER_BEE, LOW);             //turn the xbee port off -- turn the radio off
  delay(10000);
  JoinLora();                               //start and join the lora network
  delay(1000); 
  */
  /*
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
        
  //Power Down routines
  cli(); 
  sleep_enable();                           // Set sleep enable bit
  sleep_bod_disable();                      // Disable brown out detection during sleep. Saves more power
  sei();
    
  debugSerial.println("\nSleeping");        //debug: print the system is going to sleep
  delay(10);                                //This delay is required to allow print to complete
  
  //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
  delay(5000);
  power_all_disable();                      //This shuts down ADC, TWI, SPI, Timers and USART
  sleep_cpu();                              // Sleep the CPU as per the mode set earlier(power down) 
  
  /// WAIT FOR INTERUPT
 
  //wake up the system
  sleep_disable();                          // Wakes up sleep and clears enable bit. Before this ISR would have executed
  power_all_enable();                       //This shuts enables ADC, TWI, SPI, Timers and USART
  delay(1000);                                //This delay is required to allow CPU to stabilize
  debugSerial.println("Awake from sleep");  //debug: print the system is awake 
  
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
 */

  
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
