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
#include "algorithms/localization/link_metric_based/coordinate3d.h"

typedef wiselib::AndroidOsModel Os;
typedef Os::BleRadio BleRadio;
typedef Os::Debug Debug;
typedef Os::Timer Timer;
typedef Os::Clock Clock;

typedef BleRadio::node_id_t node_id_t;
typedef BleRadio::block_data_t block_data_t;

typedef double Arithmetic;
typedef wiselib::IBeaconDistanceEstimation<Os, BleRadio, Arithmetic> DistanceEstimation;
typedef wiselib::LinkMetricBasedLocalization<Os, BleRadio, DistanceEstimation, Arithmetic> Localization;

class ExampleApplication
{
public:

   void init(Os::AppMainParameter& amp)
   {
      debug_ = new Debug();
      timer_ = new Timer();
      Clock* clock_ = new Clock();
      radio_ = new BleRadio(amp);
      localization_ = new Localization();

      localization_->init(radio_, debug_, clock_);
      localization_->add_anchor(0xFF9400DEE531L, wiselib::coordinate3d<Arithmetic>(0.0, 0.0, 0.0) );
      localization_->add_anchor(0xFC2A3BABA5A6L, wiselib::coordinate3d<Arithmetic>(1.3, 0.0, 0.0) );
      localization_->add_anchor(0x0017EA92D1C5L, wiselib::coordinate3d<Arithmetic>(0.0, 1.3, 0.0) );
      localization_->add_anchor(0xE6BBD68CD17AL, wiselib::coordinate3d<Arithmetic>(1.3, 1.3, 0.1) );
      localization_->register_state_callback<ExampleApplication, &ExampleApplication::state_cb>(this);
   }

   ~ExampleApplication()
   {
      delete debug_;
      delete radio_;
      delete timer_;
      delete localization_;
   }

private:
   Debug* debug_;
   BleRadio* radio_;
   Timer* timer_;
   Localization* localization_;

   void state_cb(int state)
   {
      if(state == Localization::READY)
      {
         wiselib::coordinate3d<Arithmetic> pos = (*localization_)();
         debug_->debug("Pos: %.1f, %.1f, %.1f", pos.x, pos.y, pos.z);
      }
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


