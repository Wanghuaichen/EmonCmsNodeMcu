# EmonCmsNodeMcu
Energy monitor
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
