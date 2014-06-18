/***************************************************************************
** This file is part of the generic algorithm library Wiselib.            **
** Copyright (C) 2012 by the Wisebed (www.wisebed.eu) project.            **
**                                                                        **
** The Wiselib is free software: you can redistribute it and/or modify    **
** it under the terms of the GNU Lesser General Public License as         **
** published by the Free Software Foundation, either version 3 of the     **
** License, or (at your option) any later version.                        **
**                                                                        **
** The Wiselib is distributed in the hope that it will be useful,         **
** but WITHOUT ANY WARRANTY; without even the implied warranty of         **
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the           **
** GNU Lesser General Public License for more details.                    **
**                                                                        **
** You should have received a copy of the GNU Lesser General Public       **
** License along with the Wiselib.                                        **
** If not, see <http://www.gnu.org/licenses/>.                            **
***************************************************************************/

// vim: set noexpandtab ts=4 sw=4:

#ifndef ANDROID_TIMER_H
#define ANDROID_TIMER_H

#include <err.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include "util/delegates/delegate.hpp"

namespace wiselib
{
   /** \brief Android Implementation of \ref timer_concept "Timer Concept".
    *
    *  \ingroup timer_concept
    *  \ingroup android_facets
    *
    *  Android implementation of the \ref timer_concept "Timer Concept" ...
    */
   typedef delegate1<void, void*> timer_delegate_t;

   template<typename OsModel_P>
   class AndroidTimer
   {
   public:
      typedef OsModel_P OsModel;
      typedef suseconds_t millis_t;
      typedef AndroidTimer<OsModel_P> self_t;
      typedef self_t* self_pointer_t;
      // --------------------------------------------------------------------
      enum { SUCCESS = OsModel::SUCCESS, ERR_UNSPEC = OsModel::ERR_UNSPEC };
      // --------------------------------------------------------------------
      AndroidTimer();

      template<typename T, void ( T::*TMethod )( void* )>
      int set_timer( millis_t millis, T* obj, void* userdata );

   private:
      // make use of posix timers and thread notification
      pthread_attr_t attr;
      struct sigevent sev;
      static void timer_handler_( union sigval sv );

   }; // class AndroidTimerModel
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   struct thread_data_wrapper
   {
      timer_delegate_t callback;
      void* userdata;
      timer_t* timer_id;
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   AndroidTimer<OsModel_P>::AndroidTimer()
   {
      pthread_attr_init ( &attr );
      pthread_attr_setdetachstate ( &attr, PTHREAD_CREATE_DETACHED );
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   template<typename T, void ( T::*TMethod )( void* )>
   int AndroidTimer<OsModel_P>::
   set_timer( millis_t millis, T* obj, void* userdata )
   {

      struct itimerspec ts;
      timer_t* timer_id;

      timer_id = ( timer_t* )malloc( sizeof( timer_t ) );

      thread_data_wrapper* data;

      data = ( struct thread_data_wrapper* ) malloc( sizeof( struct thread_data_wrapper ) );
      data->callback = timer_delegate_t::from_method<T, TMethod>( obj );
      data->userdata = userdata;

      sev.sigev_notify = SIGEV_THREAD;
      sev.sigev_notify_function = &timer_handler_;
      sev.sigev_notify_attributes = &attr;
      sev.sigev_value.sival_ptr = data;

      if ( timer_create ( CLOCK_REALTIME, &sev, timer_id ) == -1 )
         return ERR_UNSPEC;

      data->timer_id = timer_id;

      ts.it_interval.tv_sec = 1;
      ts.it_interval.tv_nsec = 0;

      ts.it_value.tv_sec = millis / 1000;
      ts.it_value.tv_nsec = ( millis % 1000 ) * 1000000;

      if ( timer_settime( *timer_id, 0, &ts, NULL ) == -1 )
         return ERR_UNSPEC;

      return SUCCESS;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   void AndroidTimer<OsModel_P>::
   timer_handler_( union sigval sv )
   {
      int save_errno = errno;

      struct thread_data_wrapper* data = ( thread_data_wrapper* ) sv.sival_ptr;

      data->callback( data->userdata );

      timer_delete ( *data->timer_id );
      free ( data );

      errno = save_errno;
   }

} // namespace wiselib

#endif // ANDROID_TIMER_H
