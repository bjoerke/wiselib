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
#ifndef __ANDROID_ACCELEROMETER__
#define __ANDROID_ACCELEROMETER__

#include <android/sensor.h>
#include <android/looper.h>

#define LOOPER_ID_USER 67

namespace wiselib
{
   /** \brief Android Implementation of \ref request_sensor_concept "Request
    *  Sensor Concept"
    *
    * Android implementation of the \ref request_sensor_concept "Request
    * Sensor Concept" ...
    */

   static ASensorEventQueue* sensor_event_queue_ = NULL;

   template<typename OsModel_P>
   class AndroidAccelerometer
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidAccelerometer<OsModel> self_type;
      typedef self_type* self_pointer_t;

      typedef struct value_t
      {
         float x;
         float y;
         float z;
      } value_t;
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
      AndroidAccelerometer();
      ~AndroidAccelerometer();
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
      ASensor const* accelerometer_;
      ALooper* looper_;
      ASensorManager* sensor_manager_;
   };
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   template < typename OsModel_P>
   AndroidAccelerometer<OsModel_P>::AndroidAccelerometer()
   {
      sensor_manager_ = ASensorManager_getInstance();

      looper_ = ALooper_forThread();

      if( looper_ == NULL )
      {
         looper_ = ALooper_prepare( ALOOPER_PREPARE_ALLOW_NON_CALLBACKS );
      }

      accelerometer_ = ASensorManager_getDefaultSensor( sensor_manager_, ASENSOR_TYPE_ACCELEROMETER );

      state_ = READY;
   }
   //------------------------------------------------------------------------
   template <typename OsModel_P>
   AndroidAccelerometer<OsModel_P>::~AndroidAccelerometer()
   {
   }
   //------------------------------------------------------------------------

   static int get_sensor_data( int fd, int events, void* data )
   {
      ASensorEvent event;

      while ( ASensorEventQueue_getEvents( sensor_event_queue_, &event, 1 ) > 0 )
      {

         if( event.type == ASENSOR_TYPE_ACCELEROMETER )
         {
            memcpy ( data, &event.acceleration, sizeof ( event.acceleration ) );
         }

         return 0;
      }
   }
   //------------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidAccelerometer<OsModel_P>::value_t AndroidAccelerometer<OsModel_P>::operator ()( void )
   {
      value_t data = {0, 0, 0};

      sensor_event_queue_ = ASensorManager_createEventQueue( sensor_manager_, looper_, LOOPER_ID_USER, get_sensor_data, &data );

      if ( ASensorEventQueue_enableSensor( sensor_event_queue_, accelerometer_ ) < 0 )
      {
         state_ = INACTIVE;
         return data;
      }

      int events;

      ALooper_pollOnce( -1, NULL, &events, NULL );

      ASensorEventQueue_disableSensor( sensor_event_queue_, accelerometer_ );

      ASensorManager_destroyEventQueue( sensor_manager_, sensor_event_queue_ );

      return data;

   }
};


#endif