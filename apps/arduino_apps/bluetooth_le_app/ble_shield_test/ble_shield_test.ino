/**
 Just a testing application / quick prototype
 Setup:
 * Arduino Uno
 * BleShield (http://imall.iteadstudio.com/im130704001.htm)
 * Mount BleShield ontop of the Arduino which looks as 
   follows (from top)
 
______________________________________________________________
 
  0°   |       |       _   _                        °
   °   |       |      | |_| |_|         |      |    °
   °   |    ###|       Antenna          |      |    °
   °   |###    |                        |      |    °
   °   |       |        H M - 1 0       |      |    °
   °   |       |        C H I P         |      |    °
   °   |       |                        |      |
  7°   |       |                                    °
        Jumpers                                     °
  8°                                                °
   °                                                °
   °                                        _       °
   °                 Status *         #    |*|5V    °
   °                   Pwr  *         #    | |
 13°                                Reset  |_|3.3V
   °
   °
       #####
       #####
       #####
         #
         Usb Cable
         
___________________________________________________________________

 * Setup a the jumpers on the left so that TX is connected to
   D2 and RX is connected to D3 (if you want to choose other
   pins update the RX_PIN and TX_PIN constants)
 * select 5V at the switch on the right
 * Connect the Arduino to USB; the red 'Pwr' LED should be on and
   the yellow 'Status' LED should blink
 * Upload this sketch onto the Arduino
 * Open the SerialMonitor, select 9600 Baud and No Line Ending
 * Now you can type AT-Commands in the SerialMonitor which will
   be send to the Shield. You can find them in the Shield's
   datasheet:
   ftp://imall.iteadstudio.com/Shield/IM130704001_ITEAD_BLE_Shield/DS_IM130704001_ITEAD_BLE_Shield.pdf
 * For Example: Type "AT" and the chip will respond "OK"
 
   HAVE FUN :)
   
**/

#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 3

SoftwareSerial ble(RX_PIN, TX_PIN);

void setup()
{
  Serial.begin(9600);
  ble.begin(9600);
  Serial.println("Send commands to shield via Serial");
}
 
void loop()
{
  while(Serial.available())
  {
    ble.write(Serial.read());
  }
  while(ble.available())
  {
    Serial.write(ble.read());
  }
}
