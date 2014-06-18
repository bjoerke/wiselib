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

#ifndef ANDROID_DEBUG_H
#define ANDROID_DEBUG_H

#include <stdarg.h>
#include <stdio.h>
#include <jni.h>
#include <android/log.h>

namespace wiselib
{
   /** \brief Android Implementation of \ref debug_concept "Debug Concept".
    *
    * \ingroup debug_concept
    * \ingroup android_facets
    *
    * Android implementation of the \ref debug_concept "Debug Concept" ...
    */
   template < typename OsModel_P > class AndroidDebug
   {

   public:
      typedef OsModel_P OsModel;

      typedef AndroidDebug < OsModel > self_type;
      typedef self_type* self_pointer_t;
      // --------------------------------------------------------------------
      //
      void debug( const char* msg, ... );
      AndroidDebug () {}

   private:

      static const unsigned int maxline = 1024;
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   template < typename OsModel_P >
   void AndroidDebug < OsModel_P >::debug ( const char* msg, ... )
   {

      va_list fmtargs;
      char buffer[maxline];

      va_start ( fmtargs, msg );
      vsnprintf ( buffer, sizeof ( buffer ), msg, fmtargs );
      va_end( fmtargs );
      __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug", "%s", buffer );

   }
}

#endif
