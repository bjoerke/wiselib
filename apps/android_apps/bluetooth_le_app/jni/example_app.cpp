/*
* Wiselib example demonstrating the use of
* Bluetooth Low Energy radio
*/

#include "external_interface/android/android_application.h"
#include "external_interface/android/android_os.h"
#include "external_interface/android/android_debug.h"
#include "external_interface/android/android_clock.h"
#include "external_interface/android/android_timer.h"
#include "external_interface/android/android_ble_radio.h"

typedef wiselib::AndroidOsModel Os;

typedef Os::Radio::node_id_t node_id_t;
typedef Os::Radio::block_data_t block_data_t;

class ExampleApplication
{
public:

   void init(Os::AppMainParameter& amp)
   {
      debug_ = new Os::Debug();
      radio_ = new Os::Radio();
      radio_->setup(amp.jni_env, amp.wiselib_activity); //TODO remove this. AndroidBleRadio should get the env from somewhere else! but where???
      Os::Radio::block_data_t message[] = "Test\0";
      debug_->debug( "Hello World from Example Android Application!\n" );
      idx_ = radio_->reg_recv_callback<ExampleApplication, &ExampleApplication::on_receive>(this);
      radio_->enable_radio();
   }

   ~ExampleApplication()
   {
      delete debug_;
      delete radio_;
   }

private:
   Os::Debug* debug_;
   Os::BleRadio* radio_;
   int idx_;

   void on_receive(Os::BleRadio::node_id_t node, Os::BleRadio::size_t len, Os::BleRadio::block_data_t* data, const Os::BleRadio::ExtendedData& extended_data)
   {
      if(len >= 30)
      {
         int major = data[25] << 8 | data[26];
         int minor = data[27] << 8 | data[28];
         int txPower = data[29];
         if (txPower < 0) txPower = data[29] + 256;  //2s complement
         debug_->debug(" Header: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", (int) data[0], (int) data[1], (int) data[2], (int) data[3], (int) data[4], (int) data[5], (int) data[6], (int) data[7], (int) data[8]);
         debug_->debug("   UUID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", (int) data[9], (int) data[10], (int) data[11], (int) data[12], (int) data[13], (int) data[14], (int) data[15], (int) data[16], (int) data[17], (int) data[17], (int) data[18], (int) data[19], (int) data[20], (int) data[21], (int) data[22], (int) data[23], (int) data[24]);
         debug_->debug("  Major: %d, Minor: %d", major, minor); 
         debug_->debug("TxPower: %d", txPower);
         debug_->debug("   RSSI: %d", extended_data.link_metric() );
      }
      // radio_->disable_radio(); TODO
   }

};

// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, ExampleApplication> example_app;
// --------------------------------------------------------------------------
extern "C" {
   JNIEXPORT void JNICALL Java_com_ibralg_wiselib_WiselibActivity_exampleapp(JNIEnv* env, jobject object, jobject wiselib_activity )
   {
      Os os(env, wiselib_activity);
      example_app.init(os);
   }
}


