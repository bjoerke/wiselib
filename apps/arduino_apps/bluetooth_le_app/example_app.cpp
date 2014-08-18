/**
 This app demonstrates the use of Bluetooth Low Energy (BLE) for arduino.
 It scans for nearby iBeacons and uses them for localization.
 Furthermore it sends advertisements to act as an iBeacon.

 To run this app you need this BLE shield:
   http://imall.iteadstudio.com/development-platform/arduino/shields/im130704001.html
 Then you have to flash my custom firmware on the shield's SoC (requires soldering):
   https://github.com/bjoerke/HM-10-Firmware
 
 For further information about hardware, see
    wiselib.testing/externel_interfaces/arduino/arduino_ble_radio.

 You may want to try this app for Android first:
    apps/android/bluetooth_le_app
*/

#include "external_interface/arduino/arduino_application.h"
#include "external_interface/arduino/arduino_os.h"
#include "external_interface/arduino/arduino_debug.h"
#include "external_interface/arduino/arduino_ble_radio.h"
#include "algorithms/localization/link_metric_based/link_metric_based_localization.h"
#include "algorithms/localization/link_metric_based/distance_estimation/ibeacon.h"
#include "algorithms/localization/link_metric_based/coordinate3d.h"

#include <avr/sleep.h>
#include <avr/wdt.h>

typedef wiselib::ArduinoOsModel Os;
typedef Os::BleRadio BleRadio;
typedef Os::Debug Debug;
typedef Os::Clock Clock;

typedef float Arithmetic;
typedef wiselib::IBeaconDistanceEstimation<Os, BleRadio, Arithmetic> DistanceEstimation;
typedef wiselib::LinkMetricBasedLocalization<Os, BleRadio, DistanceEstimation, Arithmetic> Localization;

typedef BleRadio::node_id_t node_id_t;
typedef BleRadio::block_data_t block_data_t;

uint8_t advertData[30] = {
  2,    //Length1
  0x01, //ADTYPE_FLAGS
  0x06, //FLAGS_BREDR_NOT_SUPPORTED | FLAGS_GENERAL
  26,   //Length2
  0xFF, //ADTYPE_MANUFACTURER_SPECIFIC
  0x4C, 0x00,   //Manufacturer ID (Apple)
  0x02, 0x15,   //0x0215 (Advertisement)
  0x11, 0x22, 0x33, 0x44,  //UUID
  0x55, 0x66, 0x77, 0x88,  // "
  0x99, 0xAA, 0xBB, 0xCC,  // "
  0xDD, 0xEE, 0xFF, 0x00,  // "
  0xAA, 0xAA,   //Major number
  0xBB, 0xBB,   //Minor number
  -70  //txPower
};

class ExampleApplication
{
public:
   // --------------------------------------------------------------------
   void init(Os::AppMainParameter& amp)
   {
      Debug debug; BleRadio radio; Clock clock; Localization localization;
      debug_ = &debug;
      radio_ = &radio;
      localization_ = &localization;
      debug_->debug("Hello World");

      radio_->enable_radio();
      radio_->send(BleRadio::BROADCAST_ADDRESS, sizeof(advertData), advertData);

      localization_->init(radio_, debug_, &clock);
      localization_->add_anchor( 0x00DEE531, wiselib::coordinate3d<Arithmetic>(  0.0,   0.0,  0.0) );
      localization_->add_anchor( 0x04717875, wiselib::coordinate3d<Arithmetic>(130.0,   0.0,  0.0) );
      localization_->add_anchor( 0x0471783B, wiselib::coordinate3d<Arithmetic>(  0.0, 130.0,  0.0) );
      localization_->add_anchor( 0xD68CD17A, wiselib::coordinate3d<Arithmetic>(130.0, 130.0, 10.0) );
      localization_->register_state_callback<ExampleApplication, &ExampleApplication::state_cb>(this);
      for(;;) radio_->poll_serial(NULL);  //TODO: this should be replaced by an periodic timer!
   }

   Debug* debug_;
   BleRadio* radio_;
   Localization* localization_;

private:
   void state_cb(int state)
   {
      if(state == Localization::READY)
      {
         wiselib::coordinate3d<Arithmetic> pos = (*localization_)();
         debug_->debug("Pos: %d, %d, %d", (int) (pos.x*10), (int) (pos.y*10), (int) (pos.z*10));
      }
   }

};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, ExampleApplication> example_app;
// --------------------------------------------------------------------------
int main(int argc, const char** argv)
{
   init();
#if defined(USBCON)
   USBDevice.attach();
#endif
   ::Serial.begin(9600);
   wiselib::ArduinoOsModel amp;
   example_app.init(amp);
   for(;;)
   {
       if(serialEventRun) serialEventRun();
       if(wiselib::ArduinoTask::tasks_.empty())
          ;
       else
       {
          wiselib::ArduinoTask t = wiselib::ArduinoTask::tasks_.front();
          wiselib::ArduinoTask::tasks_.pop();
          t.callback_(t.userdata_);
          delay(10);
      }
   }
}

