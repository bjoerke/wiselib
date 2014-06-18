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
#ifndef __ANDROID_LIGHT_SENSOR__
#define __ANDROID_LIGHT_SENSOR__

#include <android/sensor.h>
#include <android/looper.h>

#define LOOPER_ID_LIGHT 68

namespace wiselib
{
   /** \brief Android Implementation of \ref request_sensor_concept "Request
    *  Sensor Concept"
    *
    * Android implementation of the \ref request_sensor_concept "Request
    * Sensor Concept" ...
    */

   static ASensorEventQueue* light_sensor_event_queue_ = NULL;

   template<typename OsModel_P>
   class AndroidLightSensor
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidLightSensor<OsModel> self_type;
      typedef self_type* self_pointer_t;

      typedef float value_t;
      //------------------------------------------------------------------------
      enum StateData
      {
         READY = OsModel::READY,
         NO_VALUE = OsModel::NO_VALUE,
         INACTIVE = OsModel::INACTIVE
      };
      //------------------------------------------------------------------------
      ///@name Constructor/Destructor
      ///
      /** Default constructor
       *
       */
      AndroidLightSensor();
      ~AndroidLightSensor();
      //------------------------------------------------------------------------
      ///@name Getters and Setters
      ///
      /** Returns the current state of the sensor
       *
       *  \return The current state
       */
      int state()
      {
         return state_;
      }
      //------------------------------------------------------------------------
      /** Returns data from accelerometerV
       *
       *  \returns Returns sturcture of floats for x,y and z coordinatesV
       */
      value_t operator()( void );

   private:
      /// The current state
      StateData state_;
      ASensor const* sensor_;
      ALooper* looper_;
      ASensorManager* sensor_manager_;
   };
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   template < typename OsModel_P>
   AndroidLightSensor<OsModel_P>::AndroidLightSensor()
   {
      sensor_manager_ = ASensorManager_getInstance();

      looper_ = ALooper_forThread();

      if( looper_ == NULL )
      {
         looper_ = ALooper_prepare( ALOOPER_PREPARE_ALLOW_NON_CALLBACKS );
      }

      sensor_ = ASensorManager_getDefaultSensor( sensor_manager_, ASENSOR_TYPE_LIGHT );

      state_ = READY;
   }
   //------------------------------------------------------------------------
   template <typename OsModel_P>
   AndroidLightSensor<OsModel_P>::~AndroidLightSensor()
   {
   }
   //------------------------------------------------------------------------

   static int get_light_sensor_data( int fd, int events, void* data )
   {
      ASensorEvent event;

      while ( ASensorEventQueue_getEvents( light_sensor_event_queue_, &event, 1 ) > 0 )
      {

         if( event.type == ASENSOR_TYPE_LIGHT )
         {
            memcpy ( data, &event.light, sizeof ( event.light ) );
         }

         return 0;
      }
   }
   //------------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidLightSensor<OsModel_P>::value_t AndroidLightSensor<OsModel_P>::operator ()( void )
   {
      value_t data = 0;

      light_sensor_event_queue_ = ASensorManager_createEventQueue( sensor_manager_, looper_, LOOPER_ID_LIGHT, get_light_sensor_data, &data );

      if ( ASensorEventQueue_enableSensor( light_sensor_event_queue_, sensor_ ) < 0 )
      {
         state_ = INACTIVE;
         return data;
      }

      int events;

      ALooper_pollOnce( -1, NULL, &events, NULL );

      ASensorEventQueue_disableSensor( light_sensor_event_queue_, sensor_ );

      ASensorManager_destroyEventQueue( sensor_manager_, light_sensor_event_queue_ );

      return data;

   }
};


#endif