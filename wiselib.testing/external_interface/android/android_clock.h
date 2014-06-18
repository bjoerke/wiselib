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

#ifndef ANDROID_CLOCK_H
#define ANDROID_CLOCK_H

// hack to get around the time_t prerequisite for the wiselib facet
#define time_t posix_time_t
#include <time.h>
#undef time_t

namespace wiselib
{
   /** \brief Android Implementation of \ref clock_concept "Clock Concept"
    *  \ingroup clock_concept
    *  \ingroup android_facets
    *
    * Android implementation of the \ref clock_concept "Clock Concept" ...
    */
   template < typename OsModel_P > class AndroidClock
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidClock < OsModel > self_type;
      typedef self_type* self_pointer_t;

      typedef struct timespec time_t;
      typedef time_t value_t;
      typedef uint16_t micros_t;
      typedef uint16_t millis_t;
      typedef uint32_t seconds_t;
      // --------------------------------------------------------------------
      enum States
      {
         READY = OsModel::READY,
         NO_VALUE = OsModel::NO_VALUE,
         INACTIVE = OsModel::INACTIVE
      };
      // --------------------------------------------------------------------
      AndroidClock ();

      time_t time ();
      //void set_time (time_t); // not supported
      micros_t microseconds ( time_t );
      millis_t milliseconds ( time_t );
      seconds_t seconds ( time_t );
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------

   template < typename OsModel_P > AndroidClock < OsModel_P >::AndroidClock ()
   {
   }
   // --------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidClock < OsModel_P >::time_t AndroidClock <
   OsModel_P >::time ()
   {
      time_t cur_time;

      clock_gettime ( CLOCK_REALTIME, &cur_time );
      return cur_time;
   }
   // --------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidClock < OsModel_P >::seconds_t AndroidClock <
   OsModel_P >::seconds ( time_t cur_time )
   {
      return cur_time.tv_sec;
   }
   // --------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidClock < OsModel_P >::millis_t AndroidClock <
   OsModel_P >::milliseconds ( time_t cur_time )
   {
      return ( ( cur_time.tv_nsec / 1000000 ) % 1000 );
   }
   // --------------------------------------------------------------------
   template < typename OsModel_P >
   typename AndroidClock < OsModel_P >::micros_t AndroidClock <
   OsModel_P >::microseconds ( time_t cur_time )
   {
      return ( ( cur_time.tv_nsec / 1000 ) % 1000 );
   }

}

#endif
