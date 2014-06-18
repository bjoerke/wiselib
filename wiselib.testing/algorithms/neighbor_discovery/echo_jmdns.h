/***************************************************************************
 ** This file is part of the generic algorithm library Wiselib.           **
 ** Copyright (C) 2012 by the Wisebed (www.wisebed.eu) project.           **
 **                                                                       **
 ** The Wiselib is free software: you can redistribute it and/or modify   **
 ** it under the terms of the GNU Lesser General Public License as        **
 ** published by the Free Software Foundation, either version 3 of the    **
 ** License, or (at your option) any later version.                       **
 **                                                                       **
 ** The Wiselib is distributed in the hope that it will be useful,        **
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of        **
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
 ** GNU Lesser General Public License for more details.                   **
 **                                                                       **
 ** You should have received a copy of the GNU Lesser General Public      **
 ** License along with the Wiselib.                                       **
 ** If not, see <http://www.gnu.org/licenses/>.                           **
 ***************************************************************************/

#ifndef ECHO_JMDNS_H
#define ECHO_JMDNS_H

#include <pthread.h>
#include <arpa/inet.h>

//wiselib includes
#include "util/delegates/delegate.hpp"
#include "util/pstl/vector_static.h"
#include "util/pstl/pair.h"
#include "pgb_payloads_ids.h"

#include "echomsg.h"

#define MAX_PG_PAYLOAD 30
#define ECHO_MAX_NODES 60
#define ECHO_TIMES_ACC_NEARBY 2

//TODO: CLEANUP unused echo.h garbage.

namespace wiselib
{
   /**
    * \brief EchoJmDNS
    *
    *  \ingroup neighbourhood_discovery_concept
    *  \ingroup basic_algorithm_concept
    *  \ingroup neighbourhood_discovery_algorithm
    *
    */
   template < typename OsModel_P, typename Radio_P, typename Timer_P,
            typename Debug_P >
   class WrapperEchoJmDNS;

   pthread_mutex_t neighbor_mutex_;
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   template < typename OsModel_P, typename Radio_P, typename Timer_P,
            typename Debug_P >
   class EchoJmDNS
   {
   public:
      typedef EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P> self_type;
      typedef self_type* self_pointer_t;

      // Type definitions
      typedef OsModel_P OsModel;

      typedef Radio_P Radio;
      typedef Timer_P Timer;
      typedef Debug_P Debug;
      typedef typename OsModel_P::Clock Clock;

      typedef typename Radio::node_id_t node_id_t;
      typedef typename Radio::size_t size_t;
      typedef typename Radio::block_data_t block_data_t;
      typedef typename Radio::message_id_t message_id_t;
      typedef typename Clock::time_t time_t;

      typedef EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P> self_t;

      typedef delegate4<void, uint8_t, node_id_t, uint8_t, uint8_t*>
      event_notifier_delegate_t;
      // --------------------------------------------------------------------
      struct neighbor_entry
      {
         uint16_t last_lqi;
         uint16_t avg_lqi;
         uint8_t beacons_in_row;
         node_id_t id;
         uint32_t total_beacons;
         uint8_t inverse_link_assoc;
         uint16_t stability;
         bool active;
         bool stable;
         bool bidi;
      };
      // --------------------------------------------------------------------
      struct reg_alg_entry
      {
         uint8_t alg_id;
         uint8_t data[MAX_PG_PAYLOAD];
         uint8_t size;
         event_notifier_delegate_t event_notifier_callback;
         uint8_t events_flag;
      };
      // --------------------------------------------------------------------
      typedef struct reg_alg_entry reg_alg_entry_t;
      typedef wiselib::vector_static<OsModel, reg_alg_entry_t, TOTAL_REG_ALG>
      reg_alg_vector_t;
      typedef typename reg_alg_vector_t::iterator reg_alg_iterator_t;

      /**
       * Actual Vector containing callbacks for all the register applications.
       */
      reg_alg_vector_t registered_apps;

      /**
       * Type of the structure that hold the relevant neighbor data.
       */
      typedef struct neighbor_entry neighbor_entry_t;

      /**
       * Type of the Vector Containing information for all nodes in the Neighborhood.
       */
      typedef wiselib::vector_static<OsModel, neighbor_entry_t, ECHO_MAX_NODES>
      node_info_vector_t;
      typedef typename node_info_vector_t::iterator iterator_t;

      /**
       * Actual Vector containing the nodes in the neighborhood
       */
      node_info_vector_t neighborhood;
      node_info_vector_t neighborhood_cache;

      typedef node_info_vector_t Neighbors;
      // --------------------------------------------------------------------

      enum error_codes
      {
         SUCCESS = OsModel::SUCCESS, /*!< The method return with no errors */
         RGD_NUM_INUSE = 1, /*!< This app number is already registered */
         RGD_LIST_FULL = 2, /*!< The list with the registered apps is full*/
         INV_ALG_ID = 3,    /*!< The alg id is invalid*/
         AVAHI_THREAD_FAIL = 4 /*!< The avahi thread failed publishing the
	                           service or creating browser*/
      };

      enum event_codes
      {
         NEW_NB = 1, /*!< Event code for a newly added stable neighbor */
         DROPPED_NB = 2 /*!< Event code for a neighbor removed from nb list */
      };

      /**
       * Constructor.
       *
       */
      EchoJmDNS();

      /**
       * Destructor.
       *
       */
      ~EchoJmDNS();

      /**
       * Enable the Echo system enable radio
       * and register receive callback
       * initialize vectors
       * */
      void enable();

      // --------------------------------------------------------------------

      /**
       * \brief Disable the Echo protocol.
       *
       * Disables the Echo protocol. The
       * module will unregister the receive
       * callback, and will stop send beacons.
       * The timer will keep triggering every
       * beacon_period Millis.
       * All the existing neighbors will timeout
       * at most after timeout_period and the
       * NB_DROPPED events will be generated.
       * */
      void disable();

      /**
       * \brief Initialize vectors and variables
       * for Neighborhood Discovery.
       *
       * It will initialize all the the relevant
       * structures and variables of the module:
       * clear neighborhood vector
       * set the node stability to zero.
       * */
      void init_echo()
      {
         neighborhood.clear();
      }
      ;

      /**
       * \brief Initialize the module.
       */
      void init( Radio& radio, Clock& clock, Timer& timer, Debug& debug )
      {
         radio_ = &radio;
         clock_ = &clock;
         timer_ = &timer;
         debug_ = &debug;
      };
      // --------------------------------------------------------------------
      Neighbors& topology();
      // --------------------------------------------------------------------
      template<class T, void( T::*TMethod )( uint8_t, node_id_t, uint8_t, uint8_t* )>
      uint8_t reg_event_callback( uint8_t alg_id, uint8_t events_flag, T* obj_pnt )
      {

         for ( reg_alg_iterator_t it = registered_apps.begin(); it
               != registered_apps.end(); it++ )
         {
            if ( it->alg_id == alg_id )
            {
               it->event_notifier_callback
               = event_notifier_delegate_t::template from_method < T,
               TMethod > ( obj_pnt );
               it->events_flag = events_flag;
               return 0;
            }
         }

         reg_alg_entry_t entry;
         entry.alg_id = alg_id;
         entry.size = 0;
         entry.event_notifier_callback
         = event_notifier_delegate_t::template from_method<T, TMethod>(
            obj_pnt );
         entry.events_flag = events_flag;
         registered_apps.push_back( entry );

         return 0;
      }
      // --------------------------------------------------------------------
      void unreg_event_callback( uint8_t alg_id )
      {
         for ( reg_alg_iterator_t it = registered_apps.begin(); it
               != registered_apps.end(); it++ )
         {
            if ( it->alg_id == alg_id )
            {
               it->event_notifier_callback = event_notifier_delegate_t();
               return;
            }
         }
      }
      // --------------------------------------------------------------------
      Radio& radio()
      {
         return *radio_;
      }
      // --------------------------------------------------------------------
      Clock& clock()
      {
         return *clock_;
      }
      // --------------------------------------------------------------------
      Timer& timer()
      {
         return *timer_;
      }
      // --------------------------------------------------------------------
      Debug& debug()
      {
         return *debug_;
      }

   private:

      friend class WrapperEchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>;
      // --------------------------------------------------------------------
      void notify_listeners( uint8_t event, node_id_t from, uint8_t len,
                             uint8_t* data )
      {

         for ( reg_alg_iterator_t ait = registered_apps.begin(); ait
               != registered_apps.end(); ++ait )
         {

            if ( ( ait->event_notifier_callback != 0 ) && ( ( ait->events_flag
                  & ( uint8_t ) event ) == ( uint8_t ) event ) )
            {

               ait->event_notifier_callback( event, from, len, data );

            }
         }

      }
      // --------------------------------------------------------------------
      void send_resolver ( node_id_t node_id, char action )
      {

         bool found = false;
         iterator_t it;

         neighbor_entry_t entry;
         entry.id = node_id;

         for ( it = neighborhood.begin();
               it != neighborhood.end(); ++it )
         {
            if ( it->id == node_id )
            {
               found = true;

               if ( action == 'D' )
               {
                  neighborhood.erase( it );
                  notify_listeners( DROPPED_NB, node_id, 0, 0 );
               }

               break;

            }

         }

         if ( !found && action == 'A' )
         {
            neighborhood.push_back( entry );
            notify_listeners( NEW_NB, node_id, 0, 0 );
         }

         for ( it = neighborhood.begin();
               it != neighborhood.end(); ++it )
         {
            char id[16];
            snprintf( id, 16, "%ld", it->id );
            __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug nodes [] : ", "%s", id );

         }

         fprintf( stderr, "\n\n\n" );

      }

      /**
       * Callback for receive function
       */
      int recv_callback_id_;

      jobject zeroconf_obj_;
      jclass zeroconf_class_;

      pthread_t zeroconf_thread_;

      Radio* radio_;
      Clock* clock_;
      Timer* timer_;
      Debug* debug_;
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   class BaseWrapperEchoJmDNS
   {
   public:
      virtual void send_resolver( unsigned long, char ) = 0;

      virtual ~BaseWrapperEchoJmDNS() {};
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P>
   class WrapperEchoJmDNS : public BaseWrapperEchoJmDNS
   {
   protected:
      EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P> *zeroconf_;
   public:
      WrapperEchoJmDNS () {};
      WrapperEchoJmDNS ( EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P> *zeroconf ) : zeroconf_( zeroconf ) {};

      void send_resolver( unsigned long node_id, char action )
      {
         zeroconf_->send_resolver( node_id, action );
      }
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   void parse_jni_nodes ( const char* nodes, BaseWrapperEchoJmDNS* obj )
   {

      char ip[16];
      char action;
      int i = 0, j = 0;
      struct in_addr addr;

      pthread_mutex_lock ( &neighbor_mutex_ );

      while ( nodes[j] != '\0' )
      {
         if ( nodes[j] != ';' )
         {
            if ( ( nodes[j] == 'A' ) || ( nodes[j] == 'D' ) )
            {
               action = nodes[j];
            }
            else
            {

               ip[i] = nodes[j];
               i++;
            }

            j++;
         }
         else
         {
            ip[i] = '\0';
            inet_aton ( ip, &addr );
            obj->send_resolver( addr.s_addr, action );
            i = 0;
            j++;
         }

      }

      pthread_mutex_unlock ( &neighbor_mutex_ );

   }
   // -----------------------------------------------------------------------
   typedef struct jni_wrapper
   {
      jobject* zeroconf_object;
      jclass* zeroconf_class;
      BaseWrapperEchoJmDNS* obj;
   } jni_wrapper;
   // -----------------------------------------------------------------------
   static jni_wrapper jni_data_wrapper;
   // -----------------------------------------------------------------------
   void* zeroconf_loop_thread ( void* data )
   {
      JavaVM* jvm = __GLOBAL_JAVA_VM__;
      jmethodID get_method, clear_method;
      jstring nodes;

      jni_wrapper* jni_data = ( jni_wrapper* ) data;
      JavaVMAttachArgs jargs;
      jargs.version = JNI_VERSION_1_6;
      jargs.name = "ZeroconfLoopThread";
      jargs.group = NULL;

      JNIEnv* env = NULL;

      if ( jvm->AttachCurrentThread( &env, &jargs ) != JNI_OK )
      {
         return NULL;
      }

      get_method = env->GetMethodID( *jni_data->zeroconf_class, "getNodes", "()Ljava/lang/String;" );
      clear_method = env->GetMethodID( *jni_data->zeroconf_class, "clearNodes", "()V" );

      jvm->DetachCurrentThread();

      while( 1 )
      {
         sleep ( 3 ); //maybe more

         if ( jvm->AttachCurrentThread( &env, &jargs ) != JNI_OK )
         {
            return NULL;
         }

         nodes = ( jstring ) env->CallObjectMethod( *jni_data->zeroconf_object, get_method );

         const char* str = env->GetStringUTFChars( nodes, 0 );

         parse_jni_nodes( str, jni_data->obj );

         __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug string : ", "%s", str );

         env->ReleaseStringUTFChars( nodes, str );

         env->CallVoidMethod ( *jni_data->zeroconf_object, clear_method );
         jvm->DetachCurrentThread();

      }

      return NULL;
   }
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P >
   EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::EchoJmDNS()
   {

      JavaVM* jvm_ = __GLOBAL_JAVA_VM__;


      JNIEnv* env = NULL;
      jvm_->GetEnv( ( void** ) &env, JNI_VERSION_1_6 );

      jmethodID constructor = NULL;
      jclass local_class;
      jobject local_object;
      jstring service = env->NewStringUTF( "_wiselib._udp.local." );
      jint port = 1337;


      local_class = env->FindClass( "org/wiselib/zeroconf/Zeroconf" );

      if ( local_class == NULL )
      {
         __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug: ", "Class not found" );
      }

      zeroconf_class_ = ( jclass ) env->NewGlobalRef( local_class );
      env->DeleteLocalRef( local_class );

      constructor = env->GetMethodID( zeroconf_class_, "<init>", "(Ljava/lang/String;I)V" );

      if ( constructor == NULL )
      {
         __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug: ", "Method not found" );
      }

      local_object = env->NewObject( zeroconf_class_, constructor, service, port );

      if ( local_object == NULL )
      {
         __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug: ", "Create new object failed" );
      }

      zeroconf_obj_ = env->NewGlobalRef( local_object );
      env->DeleteLocalRef( local_object );

      WrapperEchoJmDNS< OsModel_P, Radio_P, Timer_P, Debug_P > *obj_wrapper = new WrapperEchoJmDNS< OsModel_P, Radio_P, Timer_P, Debug_P > ( this );
      jni_data_wrapper.obj = obj_wrapper;

      pthread_mutex_init ( &neighbor_mutex_, NULL );

   }
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P >
   EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::~EchoJmDNS()
   {
      pthread_mutex_destroy ( &neighbor_mutex_ );
   }
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P >
   typename EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::Neighbors&
   EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::topology()
   {
      neighborhood_cache.clear();
      pthread_mutex_lock ( &neighbor_mutex_ );

      neighborhood_cache = neighborhood;

      pthread_mutex_unlock ( &neighbor_mutex_ );

      return &neighborhood_cache;

   }
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P >
   void EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::enable()
   {

      /**
       * Enable normal radio and register the receive callback.
       */
      radio().enable_radio();

      /**
       * Initialize vectors and variables.
       */
      init_echo();

      jni_data_wrapper.zeroconf_class = &zeroconf_class_;
      jni_data_wrapper.zeroconf_object = &zeroconf_obj_;

      pthread_create ( &zeroconf_thread_, NULL, zeroconf_loop_thread, &jni_data_wrapper );

   }
   // -----------------------------------------------------------------------
   template <typename OsModel_P, typename Radio_P, typename Timer_P, typename Debug_P >
   void EchoJmDNS<OsModel_P, Radio_P, Timer_P, Debug_P>::disable()
   {
      radio().template unreg_recv_callback( recv_callback_id_ );
      pthread_join( zeroconf_thread_, NULL );

   }

}

#endif	/* ECHO_AVAHI_H */
