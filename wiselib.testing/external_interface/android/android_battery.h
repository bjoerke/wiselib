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
#ifndef __ANDROID_BATTERY__
#define __ANDROID_BATTERY__

#include <stdio.h>
#include "android_types.h"
#include <android/log.h>


namespace wiselib
{
   /** \brief Android Implementation of \ref request_sensor_concept "Request
    *  Sensor Concept"
    *
    * Android implementation of the \ref request_sensor_concept "Request
    * Sensor Concept" ...
    */

   template<typename OsModel_P>
   class AndroidBattery
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidBattery<OsModel> self_type;
      typedef self_type* self_pointer_t;

      typedef unsigned int value_t;
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
      AndroidBattery();
      ~AndroidBattery();
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
      const char* filename_;

      StateData state_;
   };
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   //------------------------------------------------------------------------
   template < typename OsModel_P>
   AndroidBattery<OsModel_P>::AndroidBattery(): filename_( BATTERY_CAPACITY )
   {
      state_ = READY;
   }
   //------------------------------------------------------------------------
   template <typename OsModel_P>
   AndroidBattery<OsModel_P>::~AndroidBattery()
   {
   }
   //------------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidBattery<OsModel_P>::value_t AndroidBattery<OsModel_P>::operator ()( void )
   {
      value_t data = 0;
      FILE* fp;

      fp = fopen ( filename_, "r" );

      if ( fp == NULL )
      {
         state_ = NO_VALUE;
         return data;
      }

      fscanf ( fp, "%u", &data );
      fclose ( fp );

      return data;

   }
};


#endif