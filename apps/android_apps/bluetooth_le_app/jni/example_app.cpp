/**
 * This example demonstrates the use of Bluetooth Low Energy (BLE)
 * for Android. Furthermore it shows how to use iBeacons for
 * localization. You need a Bluetooth 4.0 capable smartphone
 * with Android 4.3+.
 * There is also an Arduino version of this app at
 *    apps/arduino/bluetooth_le_app
 */

// set the LOG_PATH if you want to dump the positions into a file
//#define LOG_PATH "/storage/emulated/0/poslog.csv"

#include "external_interface/android/android_application.h"
#include "external_interface/android/android_os.h"
#include "external_interface/android/android_debug.h"
#include "external_interface/android/android_clock.h"
#include "external_interface/android/android_timer.h"
#include "external_interface/android/android_ble_radio.h"
#include "algorithms/localization/link_metric_based/link_metric_based_localization.h"
#include "algorithms/localization/link_metric_based/distance_estimation/ibeacon.h"
#include "algorithms/localization/link_metric_based/coordinate3d.h"

#ifdef LOG_PATH
#include <stdio.h>
#endif

typedef wiselib::AndroidOsModel Os;
typedef Os::BleRadio BleRadio;
typedef Os::Debug Debug;
typedef Os::Timer Timer;
typedef Os::Clock Clock;
typedef BleRadio::node_id_t node_id_t;
typedef BleRadio::block_data_t block_data_t;

// 'Arithmetic' is the data type used by the localization algorithm when calculating the position
// So this basically determines the precision. You could also use 'float' or 'int' which may
// lead to less precision.
typedef double Arithmetic;

// The LinkMetricBasedLocalization algorithm needs two modules: an ExtendedRadio and an DistanceEstmator.
// In this app we use BLE and iBeacons for distance estimation.
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

#ifdef LOG_PATH
      log = fopen (LOG_PATH,"w");
      if(log == NULL) debug_->debug("cannot open poslog :(");
#endif

      localization_->init(radio_, debug_, clock_);

      // The LinkMetricBasedLocalization algorithm needs to know the anchors' positions. In our case
      // we use iBeacons as anchors. Bluetooth addresses are used as node ids.
      localization_->add_anchor(0xFF9400DEE531L, wiselib::coordinate3d<Arithmetic>(1.0,  1.3, 2.3) );
      localization_->add_anchor(0xFC2A3BABA5A6L, wiselib::coordinate3d<Arithmetic>(4.1,  0.8, 2.75) );
      localization_->add_anchor(0xE6BBD68CD17AL, wiselib::coordinate3d<Arithmetic>(3.5, 4.65, 2.7) );
      localization_->add_anchor(0x0017EA92D1C5L, wiselib::coordinate3d<Arithmetic>(0.6, 3.35, 1.95) );

      localization_->register_state_callback<ExampleApplication, &ExampleApplication::state_cb>(this);

      // Now the algorithm will run. It collects the distances to nearby anchors by using the DistanceEstimator.
      // When it has enough distances (i.e. 4) it calculates our position using trilateration. Then it calls our
      // 'state_cb', where the position is printed.
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

#ifdef LOG_PATH
   FILE* log;
#endif

   /**
    * \brief state callback
    * This callback is called by the localization algorithm when its state has
    * changed (i.e. it has found or lost our position).
    * \param state new state
    */
   void state_cb(int state)
   {
      if(state == Localization::READY)
      {
         wiselib::coordinate3d<Arithmetic> pos = (*localization_)();
         debug_->debug("Pos: %.1f, %.1f, %.1f", pos.x, pos.y, pos.z);
#ifdef LOG_PATH
         if (log!=NULL)
         {
            fprintf(log, "%.3f\t%.3f\t%.3f\n", pos.x, pos.y, pos.z);
            fflush(log);
         }
#endif
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


