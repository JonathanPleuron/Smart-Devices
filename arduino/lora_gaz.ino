/*

 LoRa gas sensor

 * Measures the analog value of two sensors :
    - gaz sensor (0-1.1V)
    - temperature sensor (0-3.3V)
 * Send value (4 bytes) by a LoRa module (Microchip RN2483A)to an application 
 on TheThingsNetwork (https://www.thethingsnetwork.org/) identifyed by :
  - a Device address : devAddr
  - a Network key : nwkSKey
  - an application key : appSKey
 * A heat() function permit turn on a pin to drive a heater circuit 
 for the gas sensor.
 * Device's sleeping most of the time to save energy and wakes up
 periodically to take these measures

 The circuit:
 - see schematic.jpg 
 
 created 22 Dec 2017
 by Jonathan Pleuron <pleuron@etud.insa-toulouse.fr>
    Killian Tessier  <tessier@etud.insa-toulouse.fr>

 This work is licensed under the Creative Commons Attribution 
 3.0 Unported License. To view a copy of this license, visit 
 https://creativecommons.org/licenses/by-nc/4.0/.

*/

#include <TheThingsNetwork.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define debugSerial       Serial
#define freqPlan          TTN_FP_EU868
#define byte_t            uint8_t
#define gas_sensor_pin    0   //Analog pin for the gas sensor
#define temp_sensor_pin   1   //Analog pin for the temperature sensor
#define DEFAULT_PORT      1   //TTN port
#define TX_PIN            2   //RN2483A RX pin
#define RX_PIN            3   //RN2483A TX pin
#define RST_PIN           4   //RN2483A Reset pin 
#define HEAT_PIN          5   //DC/DC boost enable pin

SoftwareSerial loraSerial(TX_PIN,RX_PIN); //Serial link with the LoRa module
volatile int f_wdt=0;
volatile int wdt_cpt=0;

/******* TTN app IDs ******/ 
/* We use a APB (Activation By Personalization) connection */
const char *devAddr = "2601124F";
const char *nwkSKey = "D727E89EA217AA295B32628E666E5249";
const char *appSKey = "2E1CE56CADD7F8F3D69663ACC7AEA22E";

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

/*************************************  LoRa functions ***************************************/

/* 
 * resetRN2483() - Reset the LoRa module
 */
void resetRN2483(){
    pinMode(RST_PIN, OUTPUT);
    digitalWrite (RST_PIN, LOW);
    delay(100);
    digitalWrite (RST_PIN, HIGH);  
    delay(100);
}

/* 
 * loraInit() - Initialize the LoRa module
 */
void loraInit(){
  resetRN2483();
  loraSerial.begin(57600);  //Serial link with the RN2483A
  MsTimer2::start();        //Timeout if ttn operation fails
  ttn.showStatus();         //Show RN2483A status
  MsTimer2::stop(); 
}

/* 
 * loraJoin() - Join TTN application
 */
void loraJoin(){
   ttn.personalize(devAddr,nwkSKey,appSKey);  //TTN join procedure (ABP mode)
}

/* 
 * loraSendBytes() - Send several bytes to the TTN application
 * 
 * @data: bytes to send
 * @length: numbre od byte to send
 * @port: TTN port
 */
int loraSendBytes(byte_t* data,int length, int port){ 
    return ttn.sendBytes(data, length,port);  //Send bytes to TTN
}

/* 
 * loraSendByte() - Send a single byte to the TTN application
 * 
 * @data: bytes to send
 * @length: numbre od byte to send
 * @port: TTN port
 */
int loraSendByte(byte_t* data,int port){
    return ttn.sendBytes(data,1,port);      //Send one byte to TTN
}

/* 
 * loraSendValues() - Send sensors values (gas + temperature) to the TTN application
 * 
 * @gas: gas sensor value
 * @temp: temperature sensor value
 * @port: TTN port
 */
int loraSendValues(int gas, int temp, int port){
      char data[4] ={gas >>8,gas & 0xFF,temp >>8,temp & 0xFF};
      return ttn.sendBytes(data,4,port);
}
/*********************************************************************************************/


/******************************** Timer/Sleep mode functions *********************************/

/* 
 * timeoutHandler() - Timeout handler
 */
void timeoutHandler(){
  debugSerial.println("Timeout !\n");
  MsTimer2::stop();
}

/* 
 * setupWatchdog() - Set up the watchdog
 */
void setupWatchdog(){
     cli();
     MCUSR &= ~(1<<WDRF);
     WDTCSR |= (1<<WDCE) | (1<<WDE);
     WDTCSR = 1<<WDP0 | 1<<WDP3;    //8.0 seconds
     WDTCSR |= _BV(WDIE);
     sei();
}
ISR(WDT_vect)
{
    //f_wdt=1;
    wdt_cpt++;
}

/* 
 * sleep_8s() - Turn Atemga328 into sleep mode for 8 seconds
 */
void sleep_8s(){
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    // sleeping here
    sleep_disable();
    power_all_enable();
}

/* 
 * goToSleep() - Turn Atemga328 into sleep
 * 
 * @sec: sleeping duration in seconds (shall be a multiple of 8)
 */
void goToSleep(int sec){
    sec=sec>>3; // division by 8s
    wdt_cpt=0;  // reset counter
    int real_period = 8*sec;  //compute real period (8 multiple)
    debugSerial.print("Sleep (");
    debugSerial.print(real_period, DEC);
    debugSerial.print("s) ");
    while(wdt_cpt<sec){
      debugSerial.print("* ");
      delay(100);
      sleep_8s(); //go to sleep
      delay(100);
    }
    wdt_cpt=0;
    debugSerial.println("Wake up !");
}
/***************************************************************************************/


/***************************** Sensor/Actuator functions *******************************/

/* 
 * getGasValue() - Returns gas value (0-1023)
 */
int getGasValue(){
  analogReference(INTERNAL);//1.1V
  int val = analogRead(gas_sensor_pin);
  debugSerial.print("Gas value = ");
  debugSerial.print(val, DEC);
  debugSerial.println("/1023");
  return val ;
}

/* 
 * getTempValue() - Returns temperature value (0-1023)
 */
int getTempValue(){
  analogReference(DEFAULT);//5V
  int val = analogRead(temp_sensor_pin);
  debugSerial.print("Temperature value = ");
  debugSerial.print(val, DEC);
  debugSerial.println("/1023");
  return val;
}

/* 
 *heat() - Turns the HEAT_PIN on to activate the heating circuit
 *
 * @sec: heating duration in seconds
 */
void heat(int sec){
  digitalWrite (HEAT_PIN, HIGH);
  delay(1000*sec);
  digitalWrite (HEAT_PIN, LOW);
}
/***************************************************************************************/


void setup() {  
  debugSerial.begin(9600);    //Start serial debugging
  pinMode(HEAT_PIN, OUTPUT);  //Set up heat pin to out
  MsTimer2::set(15000,timeoutHandler); //Set up timeout timer
  f_wdt=0;
  setupWatchdog();  //Setup Watchdog for timeout
  loraInit();       //Init the LoRa module
  loraJoin();       //Join to TTN
}

void loop() {
  delay(100);
  heat(5);    //heat the sensor during 5s
  loraSendValues(getGasValue(),getTempValue(), 1); //Get the sensors value and send them to TTN
  goToSleep(120);   //Go in sleep mode during 2 min
}

