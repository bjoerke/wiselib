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
#ifndef ANDROID_RADIO_H
#define ANDROID_RADIO_H

#include "android_types.h"
#include "util/delegates/delegate.hpp"
#include "util/serialization/simple_types.h"
#include "util/pstl/vector_static.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

namespace wiselib
{

   /** \brief Android Implementation of \ref radio_concept "Radio concept".
    *  \ingroup radio_concept
    *  \ingroup android_facets
    *
    * Android implementation of the \ref radio_concept "Radio concept" ...
    *
    * @tparam OsModel_P Has to implement @ref os_concept "Os concept".
    */

   template <typename OsModel_P>
   class WrapperThread;

   template<typename OsModel_P>
   class AndroidRadio
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidRadio<OsModel> self_type;
      typedef self_type* self_pointer_t;

      //assuming ipv4
      typedef unsigned long node_id_t;
      typedef uint8_t  block_data_t;
      typedef uint8_t  size_t;
      typedef uint8_t  message_id_t;

      typedef delegate3<void, unsigned long, uint8_t, uint8_t*> android_radio_delegate_t;
      typedef android_radio_delegate_t radio_delegate_t;


      // --------------------------------------------------------------------
      enum ErrorCodes
      {
         SUCCESS = OsModel::SUCCESS,
         ERR_UNSPEC = OsModel::ERR_UNSPEC
      };
      // --------------------------------------------------------------------
      enum { MAX_INTERNAL_RECEIVERS = 10 };
      // --------------------------------------------------------------------
      enum SpecialNodeIds
      {
         BROADCAST_ADDRESS      = 0       ///< Unknown/No node id
      };
      // --------------------------------------------------------------------
      enum Restrictions
      {
         MAX_MESSAGE_LENGTH = 255 ///< Maximal number of bytes in payload
      };
      // --------------------------------------------------------------------
      AndroidRadio();
      ~AndroidRadio() {};

      int send( node_id_t id, size_t len, block_data_t* data );

      int enable_radio();

      int disable_radio();

      node_id_t id();

      template<class T, void ( T::*TMethod )( node_id_t, size_t, block_data_t* )>
      int reg_recv_callback( T* obj_pnt );

      int unreg_recv_callback( int idx );
      void received( unsigned char* data, size_t len, node_id_t from );

   private:
      // need to access object properties C linkage
      friend void* read_poll_thread ( void* data );
      friend class WrapperThread<OsModel_P>;
      WrapperThread< OsModel_P > *wrapped_this_;

      // the node is identified by the ipv4 address
      node_id_t id_;
      // see android_types
      const char* lan_interface_;

      unsigned long broadcast_;

      int socket_;
      struct sockaddr_in server_, client_;
      const int port_;

      pthread_t receive_thread_;

      android_radio_delegate_t android_radio_callbacks_[MAX_INTERNAL_RECEIVERS];

   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   class Wrapper
   {
   public:
      virtual int* get_socket() const = 0;
      virtual int get_size() const = 0;
      virtual void received( unsigned char* data, uint8_t len, unsigned int from ) = 0;

      virtual ~Wrapper() {};
   };
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   // -----------------------------------------------------------------------
   template <typename OsModel_P>
   class WrapperThread : public Wrapper
   {
   protected:
      AndroidRadio<OsModel_P> *radio_;
   public:
      WrapperThread () {};
      WrapperThread ( AndroidRadio<OsModel_P> *radio ) : radio_( radio ) {}

      int get_size() const
      {
         return AndroidRadio<OsModel_P>::MAX_MESSAGE_LENGTH;
      }

      int* get_socket() const
      {
         return &radio_->socket_;
      }

      void received( unsigned char* data, uint8_t len, unsigned int from )
      {
         radio_->received( data, static_cast< typename AndroidRadio<OsModel_P>::size_t > ( len ), from );
      }
   };
   // -----------------------------------------------------------------------
   static void* read_poll_thread ( void* obj_this )
   {
      int recv, *sock, status = 0;
      unsigned char* data;
      uint8_t len;
      struct sockaddr_in addr;
      socklen_t addr_len;
      struct timeval waitd;

      fd_set readfds;

      Wrapper* radio = reinterpret_cast<Wrapper*>( obj_this );

      len = radio->get_size();

      data = ( unsigned char* ) malloc ( sizeof ( unsigned char ) * len );

      if ( data == NULL )
      {
         goto out;
      }

      memset ( data, 0, sizeof( data ) );
      sock = radio->get_socket();

      addr_len = sizeof ( struct sockaddr );

      while ( *sock )
      {
         waitd.tv_sec = 5;
         waitd.tv_usec = 0;


         FD_ZERO( &readfds );
         FD_SET( *sock, &readfds );

         status = select ( *sock + 1, &readfds, NULL, NULL, &waitd );

         if ( status < 0 )
         {
            goto clean;
         }
         else if ( status == 0 )
         {
            //timeout
            continue;
         }

         if( FD_ISSET( *sock, &readfds ) )
         {

            recv = recvfrom( *sock, data, len, 0, ( struct sockaddr* )&addr, &addr_len );

            if ( recv > 0 )
            {
               radio->received ( data, len, addr.sin_addr.s_addr );
               memset ( data, 0, sizeof( data ) );
               fprintf ( stderr, "data %s\n", data );
            }
            else
            {
               goto clean;
            }
         }
      }

clean:
      free ( data );

out:
      fprintf ( stderr, "Shit happened\n" );
      return NULL;

   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   AndroidRadio<OsModel_P>::AndroidRadio():  lan_interface_( INTERFACE ), port_( PORT )
   {
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   int AndroidRadio<OsModel_P>::enable_radio()
   {
      wrapped_this_ = new WrapperThread< OsModel_P >( this );

      struct ifreq ifr;

      socket_ = socket( AF_INET, SOCK_DGRAM, 0 );
      ifr.ifr_addr.sa_family = AF_INET;
      strncpy( ifr.ifr_name, lan_interface_, IFNAMSIZ - 1 );
      ioctl( socket_, SIOCGIFADDR, &ifr );

      id_ = ( ( struct sockaddr_in* )&ifr.ifr_addr )->sin_addr.s_addr;

      ioctl( socket_, SIOCGIFBRDADDR, &ifr );
      broadcast_ = ( ( struct sockaddr_in* )&ifr.ifr_broadaddr )->sin_addr.s_addr;

      // needs super user privileges to run
      if ( setsockopt( socket_, SOL_SOCKET, SO_BINDTODEVICE, lan_interface_, sizeof( lan_interface_ ) ) < 0 )
         fprintf( stderr, "%s\n", strerror( errno ) );

      int so_broadcast = 1;

      if ( setsockopt( socket_, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof( so_broadcast ) ) < 0 )
         fprintf( stderr, "%s\n", strerror( errno ) );

      memset( &server_, 0, sizeof( struct sockaddr_in ) );
      server_.sin_family = AF_INET;
      server_.sin_addr.s_addr = INADDR_ANY;
      server_.sin_port = htons ( port_ );

      if ( bind( socket_, ( struct sockaddr* )&server_, sizeof( struct sockaddr_in ) ) < 0 )
         return ERR_UNSPEC;

      int flags;
      flags = fcntl( socket_, F_GETFL, 0 );
      fcntl( socket_, F_SETFL, flags | O_NONBLOCK );

      pthread_create ( &receive_thread_, NULL, read_poll_thread, wrapped_this_ );

      return SUCCESS;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   int AndroidRadio<OsModel_P>::disable_radio()
   {
      id_ = 0;
      broadcast_ = 0;

      shutdown ( socket_, SHUT_RDWR );
      socket_ = 0;

      pthread_join( receive_thread_, NULL );

      delete wrapped_this_;

      return SUCCESS;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   typename AndroidRadio<OsModel_P>::node_id_t AndroidRadio<OsModel_P>::id()
   {
      return id_;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   int AndroidRadio<OsModel_P>::
   send( node_id_t dest, size_t len, block_data_t* data )
   {

      if ( dest == BROADCAST_ADDRESS )
         dest = broadcast_;

      memset( ( char* ) &client_, 0, sizeof( struct sockaddr_in ) );
      client_.sin_family = AF_INET;
      client_.sin_addr.s_addr = dest;
      client_.sin_port = htons( port_ );

      if ( sendto( socket_, data, len, 0, ( struct sockaddr* )&client_, sizeof( struct sockaddr_in ) ) < 0 )
         return ERR_UNSPEC;

      return SUCCESS;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   template < class T,
            void ( T::*TMethod )( typename AndroidRadio<OsModel_P>::node_id_t,
                                  typename AndroidRadio<OsModel_P>::size_t,
                                  typename AndroidRadio<OsModel_P>::block_data_t* ) >
   int
   AndroidRadio<OsModel_P>::
   reg_recv_callback( T* obj_pnt )
   {

      for ( int i = 0; i < MAX_INTERNAL_RECEIVERS; i++ )
      {
         if ( !android_radio_callbacks_[i] )
         {
            android_radio_callbacks_[i] = android_radio_delegate_t::template from_method<T, TMethod>( obj_pnt );
            return SUCCESS;
         }
      }

      return ERR_UNSPEC;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   int AndroidRadio<OsModel_P>::
   unreg_recv_callback( int idx )
   {
      android_radio_callbacks_[idx] = android_radio_delegate_t();
      return SUCCESS;
   }
   // -----------------------------------------------------------------------
   template<typename OsModel_P>
   void AndroidRadio<OsModel_P>::
   received( unsigned char* data, size_t len, unsigned long from )
   {
      for ( unsigned int i = 0; i < MAX_INTERNAL_RECEIVERS; ++i  )
      {
         if ( android_radio_callbacks_[i] )
         {
            android_radio_callbacks_[i]( from, len, data );
         }
      }
   }
}

#endif
