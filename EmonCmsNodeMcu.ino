/*=============================================================================
  Emon Cms Client
  Count IR pulses from electric meter using
  TSL250R light-to-voltage optical sensor (eBay)
  
  Average Power and Pulses are sent via Wifi every 15s to EmonCms server. 
  Needs an account. (Free) https://emoncms.org/
  Setup two feeds at EmonCms:
  1."Log to feed" on "Watt" value will show instant
  2."Wh Accumulator" on "pulseCount" will show daily consumption in Wh. Needs at least two days of data.
  Use emoncms andoid app, or web site to show result
  
  NodeMCU V3  (AliExpress) http://www.instructables.com/id/Quick-Start-to-Nodemcu-ESP8266-on-Arduino-IDE/
  Connect IR sensor to 3.3V, GND and D1 (GPIO5)  
  Arduino IDE 1.8.0
 
  Athour: Anders Wilhelmsson 2017
  
  Open source
  
*/
//-----------------------------------------------------------------------------------
// Firmware version shown at startup
const char FWR_VERSION[6] = "v1.2";
#define DEBUG
#define LEDBLINK
//-----------------------------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
// provide text for the WiFi status
const char *str_status[]= {
  "WL_IDLE_STATUS",
  "WL_NO_SSID_AVAIL",
  "WL_SCAN_COMPLETED",
  "WL_CONNECTED",
  "WL_CONNECT_FAILED",
  "WL_CONNECTION_LOST",
  "WL_DISCONNECTED"
};
// provide text for the WiFi mode
const char *str_mode[]= { "WIFI_OFF", "WIFI_STA", "WIFI_AP", "WIFI_AP_STA" };
//-----------------------------------------------------------------------------------
const char* ssid = "xxxx";
const char* password = "xxxxxxxxxxxxx";
//-----------------------------------------------------------------------------------
// Emoncms configurations
WiFiClient emonCmsClient;
char emonCmsServer[] = "emoncms.org";      // name address for emoncms.org
//IPAddress server(80, 243, 190, 58);      // numeric IP for emoncms.org (no DNS)
char apikey[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
uint16_t node = 0; //if 0, not used        // To group values at EMONCMS
uint32_t lastSendTime = 0;
uint8_t retries = 0;
//-----------------------------------------------------------------------------------
//NodeMCU ESP8266 pins= D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10   
int PIN_INT_LED1 = D4;   // IR mirror. Output active low. Tx during startup. Do not connect to high!
int PIN_INT_LED2 = D0;   // Send to EmonCMS
int PIN_IR_DIODE = D1;   // TSL250R, interrupt pin, input pullup. No external resitors. 
//-----------------------------------------------------------------------------------
// This timer interrupt is used to turn off internal led 1
#ifdef LEDBLINK
extern "C" {
#include "user_interface.h"
}
os_timer_t myTimer;
#endif
//-----------------------------------------------------------------------------------
// Array to create a smooth average curve on Emon app.
const uint16_t numReadings = 15;           // mumber of readings to calculate average 
uint16_t readings[numReadings];            // watt reading stored
uint16_t readIndex = 0;                    // index of current reading
uint32_t total = 0;                        // Running total. Must be able to store 15000*numReadings
//-----------------------------------------------------------------------------------
const uint32_t DEBOUNCE_TIME  = 150;       // debounce interrupt input, micro seconds
const uint32_t MAX_WATT       = 15000;     // max power value to report. This filetrs outliers
const uint32_t SEND_FREQUENCY = 15000;     // minimum time between send (in milliseconds)
const uint32_t PULSE_FACTOR   = 1000;      // nummber of blinks per KWH of your meeter
const uint32_t ppwh = ((uint32_t)PULSE_FACTOR)/1000; // pulses per power hour

volatile uint32_t pulseCount = 0;
volatile uint32_t pulseCountHour = 0;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t tempPower = 0;

uint32_t oldPulseCount = 0;
uint16_t power = 0;
uint32_t whThisHour = 0;

uint32_t whLastHour = 0;
uint32_t whToday = 0;
uint32_t whYesterday = 0;
uint8_t oldHour = 0;
uint8_t oldDay = 0;
uint8_t oldMin = 0;
uint8_t oldSec = 0;
uint32_t retryCount = 0;
uint32_t uptimeCounter = 0;
volatile uint32_t deltaTime;               // Time since last interrupt

//-----------------------------------------------------------------------------------
#include <EthernetUdp.h>
#include <TimeLib.h>
// NTP Servers:
IPAddress timeServer(66, 199, 22, 67); 
//time.nist.gov               129.  6. 15. 28 NIST, Gaithersburg, Maryland  All services available
//nist1-macon.macon.ga.us        98.175.203.200 Macon, Georgia  All services available
//wolfnisttime.com               66.199. 22. 67 Wolf-Tek, Birmingham, Alabama All services available
//nist.netservicesgroup.com      64.113. 32.  5 Southfield, Michigan  All services available
//nisttime.carsoncity.k12.mi.us 198.111.152.100 Carson City, Michigan All services available
//nist1-lnk.binary.net          216.229.  0.179 Lincoln, Nebraska All services available
//wwv.nist.gov                   24. 56.178.140 WWV, Fort Collins, Colorado

  const int timeZone = 1;     // Central European Time
  //time_t prevNow = 0;         // Will trig action every second
  WiFiUDP Udp;
  unsigned int localPort = 8888;  // local port to listen for UDP packets
//-----------------------------------------------------------------------------------
void setup() 
{
  // start serial port:
  Serial.begin( 115200);
  while (!Serial); // wait until Arduino Serial Monitor opens
    yield();

  //----- NodeMCU module settings  
  //----- prepare GPIO2
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);

  // Connect to WiFi network
  // Show WiFi start on LED and in terminal
  pinMode(PIN_INT_LED1, OUTPUT);
  pinMode(PIN_INT_LED2, OUTPUT);
  digitalWrite(PIN_INT_LED1, HIGH);
  digitalWrite(PIN_INT_LED2, HIGH);
  
  Serial.println ("Connecting WiFi");
  connectWifi();
  
   // Initiate interrupt
  pinMode(PIN_IR_DIODE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_DIODE), onPulse, FALLING);
  
  #ifdef LEDBLINK
  // Initiate timer interrupt
  os_timer_setfn(&myTimer, timerCallback, NULL);
  #endif
  
  // NTP Server
  Serial.println(F("Connecting to NTP time server"));
  Udp.begin(localPort);
  
  setSyncProvider(getNtpTime);
  setSyncInterval(3600);  //Seconds
  
  showWelcomeMessage();

  Serial.println(F("Type h for help"));
  Serial.println();
  
  lastSendTime = millis();  
}
//-----------------------------------------------------------------------------------
void showWelcomeMessage() 
{
  Serial.println();
  Serial.print(F("EmonCMS Wifi Client. Ver:")); Serial.println(FWR_VERSION);
  Serial.print(F("Compiled: "));Serial.print(__DATE__);Serial.print(F(" "));Serial.println(__TIME__);
  Serial.print(F("Free Heap[B]: "));Serial.println(ESP.getFreeHeap());
  Serial.print(F("Chip ID: 0x"));Serial.println(ESP.getChipId(), HEX);
  Serial.println();
  Serial.println( dayMonthYearString(now()) + " " + timeString(now()));
}
//-----------------------------------------------------------------------------------
void printWifiStatus() 
{
  Serial.println();
  Serial.print(F("SSID: "));Serial.println( WiFi.SSID());
  IPAddress ip = WiFi.localIP();Serial.print(F("IP Address: "));Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print(F("Signal strength (RSSI): "));Serial.print( rssi);Serial.println(F("dBm"));   
  Serial.print(F("WiFi mode: "));Serial.println(str_mode[WiFi.getMode()]);
  Serial.print (F("Status: ")); Serial.println (str_status[WiFi.status()]);
  Serial.println();
}
//-----------------------------------------------------------------------------------
void connectWifi() 
{
  Serial.print(F("Connecting as wifi client to SSID: "));Serial.println(ssid);

  // use in case of mode problem
  WiFi.disconnect();
  // switch to Station mode
  if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA);
  }
 
  WiFi.begin (ssid, password );

  #ifdef DEBUG
    WiFi.printDiag(Serial);
  #endif
  
  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(250);
    Serial.print(".");
    digitalWrite(PIN_INT_LED2, LOW);
    delay(250);
    digitalWrite(PIN_INT_LED2, HIGH);
  }
  Serial.println("");
  // Check connection
  if (WiFi.status() == WL_CONNECTED) {
    printWifiStatus();
  } else {
    Serial.print("WiFi connect failed to ssid: ");
    Serial.println( ssid);
    Serial.print("WiFi password <");
    Serial.print( password);
    Serial.println(">");
    Serial.println("Check for wrong typing!");
    Serial.println();
    Serial.println("WiFi connect failed, push RESET button.");
    signalError();
  }
}
//-----------------------------------------------------------------------------------
void signalError()   // loop endless with LED blinking in case of error
{
  while(1) {
      digitalWrite(PIN_INT_LED1, HIGH);
      digitalWrite(PIN_INT_LED2, LOW);
      delay(100); // ms
      digitalWrite(PIN_INT_LED1, LOW);
      digitalWrite(PIN_INT_LED2, HIGH);
      delay(100); // ms
  }
}
//-----------------------------------------------------------------------------------
void runTerminal() 
{
  char sBuff[12];
  
  // Query exists on serial port; 
  if( Serial.available() > 0) {
    int8_t incoming_byte = Serial.read();
    Serial.flush();
    switch (incoming_byte) {
      case 'h':
        showWelcomeMessage();
        Serial.println();
        Serial.println(F("Type:"));
        Serial.println(F("d = Show Data"));
        break;        
      case 'd':
        Serial.println();
        Serial.print(F("Now :\t\t"));    Serial.print(power);      Serial.println(F(" Watt"));  
        Serial.print(F("This hour :\t"));Serial.print(whThisHour); Serial.println(F(" Wh"));
        Serial.print(F("Last hour :\t"));Serial.print(whLastHour); Serial.println(F(" Wh"));
        Serial.print(F("Today :\t\t"));  Serial.print((float)(whToday/1000),3);  Serial.println(F(" kWh"));
        Serial.print(F("Yesterday :\t"));Serial.print((float)(whYesterday/1000),3);  Serial.println(F(" kWh"));
        break;
      case 'w':
        printWifiStatus();
        break;
      default:
        break;
    }    
  }
}
//-----------------------------------------------------------------------------------
String dateString(time_t t) 
{
  char strBuff[20];
  sprintf(strBuff,"%4d-%02d-%02d %02d:%02d:%02d",year(t),month(t),day(t),hour(t),minute(t),second(t));
  return strBuff;
}
//-----------------------------------------------------------------------------
String timeString(time_t t) 
{
  char strBuff[20];
  sprintf( strBuff,"[%02d:%02d:%02d] ",hour(t),minute(t),second(t));
  return strBuff;
}
//-----------------------------------------------------------------------------
String dayMonthYearString(time_t t) 
{
  return (String) dayStr(weekday(t)) + " " + String( day(t))+ " " + (String) monthStr( month(t)) + " " + String(year(t)); 
}
//-----------------------------------------------------------------------------
String dayString(time_t t) 
{
  return (String) dayStr(weekday(t)) + " " + String( day(t)); 
}
//-----------------------------------------------------------------------------
void printDebug(String str) 
{
#ifdef DEBUG
  Serial.print(timeString( now()));
  Serial.println(str);    
#endif
}
//-----------------------------------------------------------------------------
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() 
{
  //Serial.println(F("Starting to get NTP time"));
  digitalWrite( PIN_INT_LED2, LOW);
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("NTP Tx ->"));
  sendNTPpacket( timeServer);
  
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      
      Serial.println(F("NTP Rx <-"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      digitalWrite( PIN_INT_LED2, HIGH);
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  } 
  Serial.println(F("NTP No Response :-("));
  digitalWrite( PIN_INT_LED2, HIGH);
  return 0; // return 0 if unable to get the time
}
//-----------------------------------------------------------------------------------
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) 
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write( packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
//-----------------------------------------------------------------------------------
bool okFromServer() 
{
  const uint32_t NETWORK_TIMEOUT = 1750;     // Wait timeout for receiving data
  const uint16_t NETWORK_DELAY = 50;         // Wait if no data is available before trying again
    
  uint32_t timeoutStart = millis();
  String receiveStr = "";
      
  while(emonCmsClient.connected() || emonCmsClient.available()){
    if( emonCmsClient.available()) {
      receiveStr += char(emonCmsClient.read());
      if(receiveStr.substring(0, receiveStr.length()) == "ok"){
        lastSendTime = millis();
        retries = 0;
        printDebug(F("Received OK <-"));
        emonCmsClient.stop();
        return true;
        break;
      }
    }
    if(millis() - timeoutStart > NETWORK_TIMEOUT) {
      printDebug(F("Err, Network Timeout"));
      break;
    }       
  }  
  if(receiveStr.length()>0){
    printDebug("Err, Wrong answer from server: " + receiveStr);
  }
  emonCmsClient.stop();
  return false;       
}
//-----------------------------------------------------------------------------------
bool postData() 
{
  printDebug(F("Connecting to server ->"));
  
  int8_t serverRespons = emonCmsClient.connect( emonCmsServer,80);
  if (serverRespons == 1) {
    printDebug(F("Connected, posting..."));
    emonCmsClient.flush(); 
    emonCmsClient.print(F("GET /input/post.json?"));
    if (node > 0) {
      emonCmsClient.print(F("node="));
      emonCmsClient.print(node);
      emonCmsClient.print(F("&"));
    }  
    emonCmsClient.print(F("json={"));
    
    emonCmsClient.print(F("Watt:"));
    emonCmsClient.print(power);

    emonCmsClient.print(F(",pulseCount:"));
    emonCmsClient.print(pulseCount);

    emonCmsClient.print(F(",whThisHour:"));
    emonCmsClient.print(whThisHour);
    
    emonCmsClient.print(F(",whLastHour:"));
    emonCmsClient.print(whLastHour);

    emonCmsClient.print(F(",whToday:"));
    emonCmsClient.print(whToday);

    emonCmsClient.print(F(",whYesterday:"));
    emonCmsClient.print(whYesterday);
   
    emonCmsClient.print(F(",uptime:"));
    emonCmsClient.print(uptimeCounter);
/*
    // Check if any comm problem with EmonCMS
    emonCmsClient.print(F(",retryCount:"));
    emonCmsClient.print(retryCount);
    
    // Signal srength of Wifi
    long rssi = WiFi.RSSI(); 
    emonCmsClient.print(F(",RSSI:"));
    emonCmsClient.print(rssi);
*/
    emonCmsClient.print(F("}&apikey="));
    emonCmsClient.println(apikey); 
       
    emonCmsClient.println(F(" HTTP/1.1"));
    emonCmsClient.println(F("Host: emoncms.org"));
    emonCmsClient.println(F("User-Agent: Arduino-ethernet"));
    emonCmsClient.println(F("Connection: close"));
    emonCmsClient.println();
    
    return true;
  } else {
    printDebug("Error connecting to server. Response = " + serverRespons);
  }  
  emonCmsClient.stop(); 
  return false; 
}
//-----------------------------------------------------------------------------------
void runEmonCmsClient()
{
  const byte MAX_RETRIES = 3;
  
  // Post data to server
  // Indicate on internal led
  digitalWrite(PIN_INT_LED2, LOW);
  if(!postData()) retries++;
  else if(!okFromServer()) retries++;
        
  // Check for max retries
  if(retries>MAX_RETRIES){
    printDebug(F("Max retries"));
    retries = 0;
    retryCount++;
    lastSendTime = millis();   
  } 
  digitalWrite( PIN_INT_LED2, HIGH);
}
//-----------------------------------------------------------------------------------
void checkReset() 
{
  if (oldSec != second()) {
    oldSec = second();
  }

  // Count uptime in minutes
  if (oldMin != minute()) {
    oldMin = minute();
    uptimeCounter++;
  }
  
  // Save data and reset new hour
  if (oldHour != hour()) {
    printDebug(F("New hour"));    
    whToday += whThisHour;
    whLastHour = whThisHour;
    whThisHour = 0;
    pulseCountHour = 0;
    oldHour = hour();
  }
  // Save data and reset new day  
  if (oldDay != day()) {    
    printDebug(F("New day"));
    whYesterday = whToday;
    whToday = 0;
    oldDay = day();    
  }  
}
//-----------------------------------------------------------------------------------
void loop() 
{
 // Do some work if we have more than one puls after startup 
 // and at least one new pulse since last scan
 if ( pulseCount != oldPulseCount && pulseCount > 1) {
    oldPulseCount = pulseCount; 
    
    // Cut off wrong values
    tempPower = constrain( tempPower, 0, MAX_WATT);

    // Average filter. Makes a smoother curve in Emon.
    total = total - readings[readIndex];
    readings[readIndex] = tempPower;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
    // calculate the average:
    power = total / numReadings;
    
    // Calculate Wh. Puls factor will give kWh.
    whThisHour = pulseCountHour*1000/PULSE_FACTOR;

    // Show what's going on...
    printDebug("Power= "+String(power)+" W.\tThis hour= "+String(whThisHour)+" Wh.\tPulses= "+String(pulseCount)+"\tDelta= "+String(deltaTime)+" microsec");
  }
 
  // Send result to EmonCms every 15s
  if (millis() - lastSendTime > SEND_FREQUENCY) {
    if (!emonCmsClient.connected()) {
      runEmonCmsClient();
    } else Serial.println("Emon server still connected");      
  }
  
  void();
  checkReset();
  void();
  runTerminal();
  void();
}
//-----------------------------------------------------------------------------------
void onPulse()  //ISR
{  
  uint32_t pulseTime = micros();
  if (lastPulseTime != 0 ) {     
    if (pulseTime - lastPulseTime > DEBOUNCE_TIME*1000) { // IR diode makes a slope when chaning state. Ignore this!
      if (lastPulseTime < pulseTime) { 
     
        // Calculate time between pulses
        deltaTime = pulseTime - lastPulseTime;   
            
        // Calculate consumption right now
        tempPower = (3600000000.0 /deltaTime) / ppwh;
                
        #ifdef LEDBLINK
        // Show at internal LED
        digitalWrite( PIN_INT_LED1, LOW);

        // Arm timer to turn LED off
        os_timer_arm(&myTimer, DEBOUNCE_TIME, false);
        #endif
        
        // Count raw pulses. Will be reset every hour.
        pulseCountHour++;
        
        // Count raw pulses. Will not be reset. 
        // It is used with EmonCMS ACC Value to build day and weekly diagram.
        pulseCount++;
        
      } else printDebug(F("Millis wrap")); 
    } //else Serial.println("Debounce ignored");
  } else printDebug(F("First pulse detected"));
  lastPulseTime = pulseTime;  
}      
//-----------------------------------------------------------------------------------
// start timerCallback
void timerCallback(void *pArg) 
{
    digitalWrite( PIN_INT_LED1, HIGH); // High= OFF
} 



