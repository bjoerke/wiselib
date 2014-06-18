#include "external_interface/arduino/arduino_application.h"
#include "external_interface/arduino/arduino_os.h"
#include "external_interface/arduino/arduino_debug.h"
#include "external_interface/arduino/arduino_ibeacon_radio.h"

typedef wiselib::ArduinoOsModel Os;
typedef wiselib::ArduinoIBeaconRadio<Os> Radio;

class ExampleApplication
{
public:
   // --------------------------------------------------------------------
   void init(Os::AppMainParameter& amp)
   {
      Os::Debug debug;
      Os::Radio radio;
      debug.debug("Hello World");
      radio.enable_radio();
      Radio::block_data_t data[] = "Hallo";
      radio.send(Radio::BROADCAST_ADDRESS, 0, data);
      for(;;);
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

