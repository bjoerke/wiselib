/**
 * This example demonstrates the use of Bluetooth Low Energy
 * for Android. Furthermore it shows how to use iBeacons for
 * localization. You need an Bluetooth 4.0 capable smartphone
 * with Android 4.3+.
 */

#include "external_interface/android/android_application.h"
#include "external_interface/android/android_os.h"
#include "external_interface/android/android_debug.h"
#include "external_interface/android/android_clock.h"
#include "external_interface/android/android_timer.h"
#include "external_interface/android/android_ble_radio.h"
#include "algorithms/localization/link_metric_based/link_metric_based_localization.h"
#include "algorithms/localization/link_metric_based/distance_estimation/ibeacon.h"

typedef wiselib::AndroidOsModel Os;
typedef Os::BleRadio BleRadio;
typedef Os::Debug Debug;

typedef BleRadio::node_id_t node_id_t;
typedef BleRadio::block_data_t block_data_t;

typedef wiselib::IBeaconDistanceEstimation<Os, BleRadio> DistanceEstimation;
typedef wiselib::LinkMetricBasedLocalization<Os, BleRadio, DistanceEstimation> Localization;

class ExampleApplication
{
public:

   void init(Os::AppMainParameter& amp)
   {
      debug_ = new Debug();
      radio_ = new BleRadio(amp);
      localization_ = new Localization();

      localization_->init(radio_, debug_);
      localization_->register_state_callback<ExampleApplication, &ExampleApplication::state_cb>(this);
   }

   ~ExampleApplication()
   {
      delete debug_;
      delete radio_;
      delete localization_;
   }

private:
   Debug* debug_;
   BleRadio* radio_;
   Localization* localization_;

   void state_cb(int state)
   {
      debug_->debug("new state %d", state);
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


