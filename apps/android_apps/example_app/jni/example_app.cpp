/*
* Simple Wiselib Example
*/
#include "external_interface/android/android_application.h"
#include "external_interface/android/android_os.h"
#include "external_interface/android/android_debug.h"
#include "external_interface/android/android_clock.h"
#include "external_interface/android/android_timer.h"
#include "external_interface/android/android_wlan_radio.h"
#include "external_interface/android/android_accelerometer.h"
#include "external_interface/android/android_battery.h"
#include "external_interface/android/android_light_sensor.h"
#include "external_interface/android/android_proximity_sensor.h"
#include "external_interface/android/android_magnetic_sensor.h"
#include "algorithms/neighbor_discovery/echo_jmdns.h"


typedef wiselib::AndroidOsModel Os;

class ExampleApplication
{
public:
   // --------------------------------------------------------------------
   void init(wiselib::AndroidOsModel::AppMainParameter amp)
   {
      debug_ = new Os::Debug();
      clock_ = new Os::Clock();
      timer_ = new Os::Timer();
      radio_ = new Os::Radio();
      accelerometer_ = new Os::Accelerometer();
      battery_ = new Os::Battery();
      light_ = new Os::LightSensor();
      proximity_ = new Os::ProximitySensor();
      magnetic_ = new Os::MagneticSensor();

      wiselib::AndroidAccelerometer<Os>::value_t data;
      wiselib::AndroidBattery<Os>::value_t capacity;
      wiselib::AndroidLightSensor<Os>::value_t light_data;
      wiselib::AndroidProximitySensor<Os>::value_t proximity_data;
      wiselib::AndroidMagneticSensor<Os>::value_t magnetic_data;

      Os::Radio::block_data_t message[] = "Test\0";

      debug_->debug( "Hello World from Example Android Application!\n" );
      // --------------------------------------------------------------------
      data = ( *accelerometer_ )();
      debug_->debug( "Data from your accelerometer %f, %f, %f\n", data.x, data.y, data.z );

      debug_->debug( "Android accelerometer enabled, I'm going to sleep for 5 seconds please move your phone around\n" );
      sleep ( 5 );
      data = ( *accelerometer_ )();
      debug_->debug( "Data from your accelerometer %f, %f, %f\n", data.x, data.y, data.z );
      // --------------------------------------------------------------------
      light_data = ( *light_ )();
      debug_->debug( "Data from your light sensor %f\n", light_data );

      debug_->debug( "I'm going to sleep for 5 seconds please make sure your phone is better lit\n" );
      sleep ( 5 );
      light_data = ( *light_ )();
      debug_->debug( "Data from your light sensor %f\n", light_data );
      // --------------------------------------------------------------------
      proximity_data = ( *proximity_ )();
      debug_->debug( "Data from your proximity sensor %f\n", proximity_data );

      debug_->debug( "I'm going to sleep for 5 seconds please hold your hand on top of your phone\n" );
      sleep ( 5 );
      proximity_data = ( *proximity_ )();
      debug_->debug( "Data from your proximity sensor %f\n", proximity_data );
      // --------------------------------------------------------------------
      magnetic_data = ( *magnetic_ )();
      debug_->debug( "Data from your magnetic field sensor %f %f %f\n", magnetic_data.x, magnetic_data.y, magnetic_data.y );

      debug_->debug( "I'm going to sleep for 5 seconds please move your phone around\n" );
      sleep ( 5 );
      magnetic_data = ( *magnetic_ )();
      debug_->debug( "Data from your magnetic field sensor %f %f %f\n", magnetic_data.x, magnetic_data.y, magnetic_data.y );
      // --------------------------------------------------------------------
      capacity = ( *battery_ )();
      debug_->debug( "Your battery capacity is %u\n", capacity );
      // --------------------------------------------------------------------
      timer_->set_timer<ExampleApplication, &ExampleApplication::on_time>( 300, this, ( void* )timer_ );

      radio_->enable_radio();

      idx_ = radio_->reg_recv_callback<ExampleApplication, &ExampleApplication::receive_radio_message>( this );

      debug_->debug( "Android Radio Broadcast" );
      radio_->send( Os::Radio::BROADCAST_ADDRESS, 4, message );

      wiselib::EchoJmDNS<Os, Os::Radio, Os::Timer, Os::Debug> nb_t;
      nb_t.init( *radio_, *clock_, *timer_, *debug_ );

      uint8_t flags =  wiselib::EchoJmDNS<Os, Os::Radio, Os::Timer, Os::Debug>::NEW_NB | wiselib::EchoJmDNS<Os, Os::Radio, Os::Timer, Os::Debug>::DROPPED_NB;
      nb_t.reg_event_callback<ExampleApplication, &ExampleApplication::callback>( flags, 7, this );

      nb_t.enable();

      while ( 1 ) pause();

   }
   // --------------------------------------------------------------------
   void callback( uint8_t event, Os::Radio::node_id_t from, uint8_t len, uint8_t* data )
   {
      struct sockaddr_in ip;
      ip.sin_addr.s_addr = from;

      if ( wiselib::EchoJmDNS<Os, Os::Radio, Os::Timer, Os::Debug>::NEW_NB == event )
      {
         debug_->debug( "NEW_NB; %s \n" , inet_ntoa( ip.sin_addr ) );
      }
      else
      {
         debug_->debug( "DROPPED_NB; %s \n" , inet_ntoa( ip.sin_addr ) );
      }

   }
   // --------------------------------------------------------------------
   void on_time( void* )
   {
      Os::Clock::time_t t = clock_->time();
      debug_->debug( "Unix timestamp: %d", clock_->seconds ( t ) );
   }
   // --------------------------------------------------------------------
   void receive_radio_message( Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t* buf )
   {
      debug_->debug( "received radio at %u from %u\n", radio_->id(), from );
      debug_->debug( "msg len %u, first byte %c\n", len, buf[0] );
      radio_->unreg_recv_callback( idx_ );
   }
   // --------------------------------------------------------------------
   ~ExampleApplication()
   {
      delete radio_;
      delete clock_;
      delete debug_;
      delete timer_;
      delete accelerometer_;
      delete battery_;
      delete light_;
      delete proximity_;
      delete magnetic_;
   }
private:
   int idx_;
   Os::Accelerometer* accelerometer_;
   Os::Battery* battery_;
   Os::Clock* clock_;
   Os::Debug* debug_;
   Os::Timer* timer_;
   Os::Radio* radio_;
   Os::LightSensor* light_;
   Os::ProximitySensor* proximity_;
   Os::MagneticSensor* magnetic_;
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
