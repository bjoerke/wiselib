#include "external_interface/arduino/arduino_application.h"
#include "external_interface/arduino/arduino_os.h"
#include "external_interface/arduino/arduino_debug.h"
#include "external_interface/arduino/arduino_ble_radio.h"
#include "algorithms/localization/link_metric_based/link_metric_based_localization.h"
#include "algorithms/localization/link_metric_based/distance_estimation/static.h"
#include "algorithms/localization/link_metric_based/distance_estimation/ibeacon.h"
#include "algorithms/localization/link_metric_based/coordinate3d.h"

typedef wiselib::ArduinoOsModel Os;
typedef Os::BleRadio BleRadio;
typedef Os::Debug Debug;

typedef float Arithmetic;
typedef wiselib::IBeaconDistanceEstimation<Os, BleRadio, Arithmetic> DistanceEstimation;
//typedef wiselib::StaticDistanceEstimation<Os, BleRadio, Arithmetic> DistanceEstimation;
typedef wiselib::LinkMetricBasedLocalization<Os, BleRadio, DistanceEstimation, Arithmetic> Localization;

typedef BleRadio::node_id_t node_id_t;
typedef BleRadio::block_data_t block_data_t;

typedef struct iBeaconAdvertData
{
  uint8_t len1;
  uint8_t type1;
  uint8_t flags;
  
  uint8_t len2;
  uint8_t type2;
  uint8_t manufacturerID_lo;
  uint8_t manufacturerID_hi;
  uint16_t advertisement;
  uint8_t uuid[16];
  uint8_t major_hi;
  uint8_t major_lo;
  uint8_t minor_hi;
  uint8_t minor_lo;
  uint8_t txPower;
}__attribute__((packed)) iBeaconAdvertData_t;

iBeaconAdvertData_t advertData;

class ExampleApplication
{
public:
   // --------------------------------------------------------------------
   void init(Os::AppMainParameter& amp)
   {
      Debug debug; BleRadio radio; Localization localization;
      radio_ = &radio;
      debug_ = &debug;
      localization_ = &localization;
      advertData.len1 = 2;
      advertData.type1 = 0x01; //GAP_ADTYPE_FLAGS
      advertData.flags = 0x06; //GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL
      advertData.len2 = 26;
      advertData.type2 = 0xFF; //GAP_ADTYPE_MANUFACTURER_SPECIFIC,
      advertData.manufacturerID_lo = 0x4C; //Apple
      advertData.manufacturerID_hi = 0x00;
      advertData.advertisement = 0x1502;
      advertData.major_lo = 0xBB;
      advertData.major_hi = 0xBB;
      advertData.minor_lo = 0xAA;
      advertData.minor_hi = 0xAA;
      advertData.txPower = 0x90;
      for(int i=0; i<16; i++) advertData.uuid[i] = i;

      debug_->debug("Hello World");

      radio_->enable_radio();
      radio_->send(BleRadio::BROADCAST_ADDRESS, sizeof(iBeaconAdvertData_t), (block_data_t*) &advertData);

      localization_->init(radio_, debug_);
      localization_->add_anchor( 0x00DEE531, wiselib::coordinate3d<Arithmetic>(0.0, 0.0, 0.0) );
      localization_->add_anchor( 0x04717875, wiselib::coordinate3d<Arithmetic>(1.3, 0.0, 0.0) );
      localization_->add_anchor( 0x0471783B, wiselib::coordinate3d<Arithmetic>(0.0, 1.3, 0.0) );
      localization_->add_anchor( 0xD68CD17A, wiselib::coordinate3d<Arithmetic>(1.3, 1.3, 0.1) );
      localization_->register_state_callback<ExampleApplication, &ExampleApplication::state_cb>(this);
      for(;;) radio_->poll();  //needed for periodic polling of serial TODO remove this
   }

private:

private:
   Debug* debug_;
   BleRadio* radio_;
   Localization* localization_;

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
   Serial.begin(9600);  //TODO: this should go into Debug shouldn't it?
   wiselib::ArduinoOsModel amp;
   example_app.init(amp);
   return 0;
}

