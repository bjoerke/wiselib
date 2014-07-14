#include "external_interface/arduino/arduino_application.h"
#include "external_interface/arduino/arduino_os.h"
#include "external_interface/arduino/arduino_debug.h"
#include "external_interface/arduino/arduino_ble_radio.h"

typedef wiselib::ArduinoOsModel Os;
typedef wiselib::ArduinoBleRadio<Os> Radio;

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
      Os::Debug debug_;
      Os::Radio radio_;

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

      debug_.debug("Hello World");

      radio_.reg_recv_callback<ExampleApplication, &ExampleApplication::on_receive>(this);
      radio_.enable_radio();
      radio_.send(Radio::BROADCAST_ADDRESS, sizeof(iBeaconAdvertData_t), (Radio::block_data_t*) &advertData);
      for(;;) radio_.poll();  //needed for periodic polling of serial TODO remove this
   }

private:

   void on_receive(Os::BleRadio::node_id_t node, Os::BleRadio::size_t len, Os::BleRadio::block_data_t* data, const Os::BleRadio::ExtendedData& extended_data)
   {
      if(len >= 30)
      {
         int major = data[25] << 8 | data[26];
         int minor = data[27] << 8 | data[28];
         int txPower = data[29];
         if (txPower < 0) txPower = data[29] + 256;  //2s complement
         Serial.println("DEV!");
/*         debug_.debug(" Header: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", (int) data[0], (int) data[1], (int) data[2], (int) data[3], (int) data[4], (int) data[5], (int) data[6], (int) data[7], (int) data[8]);
         debug_.debug("   UUID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", (int) data[9], (int) data[10], (int) data[11], (int) data[12], (int) data[13], (int) data[14], (int) data[15], (int) data[16], (int) data[17], (int) data[17], (int) data[18], (int) data[19], (int) data[20], (int) data[21], (int) data[22], (int) data[23], (int) data[24]);
         debug_.debug("  Major: %d, Minor: %d", major, minor); 
         debug_.debug("TxPower: %d", txPower);
         debug_.debug("   RSSI: %d", extended_data.link_metric() ); */
      }
//      radio_.disable_radio();
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

