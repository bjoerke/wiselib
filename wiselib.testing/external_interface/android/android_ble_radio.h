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
#ifndef ANDROID_BLE_RADIO_H
#define ANDROID_BLE_RADIO_H

#include "android_types.h"
#include "android_os.h"
#include "util/delegates/delegate.hpp"
#include "util/serialization/simple_types.h"
#include "util/pstl/vector_static.h"
#include "util/base_classes/extended_radio_base.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <jni.h>

//incomming bluetooth data from Java is passed to this delegate by the 
//'Java_com_ibralg_wiselib_WiselibActivity_onBleDataReceive' function
//It is set to wiselib::AndroidBleRadio::onBleDataRecv() where
//the data is passed to all registered callbacks.
static delegate4<void, JNIEnv*, jobject, jbyteArray, jint> onBleDataReceive;

namespace wiselib
{

   /** \brief Android Implementation of \ref extended_adio_concept "Extended Radio concept".
    *  \ingroup extended_radio_concept
    *  \ingroup android_facets
    *
    * Android implementation of the \ref extended_radio_concept "Extended Radio concept" by using
    * Bluetooth Low Energy, also knows as Bluetooth 4.
    * Extended data is received signal strength indication (RSSI)
    *
    * This class basically passes all calls to Java code which then utilises the
    * Android API to access all Bluetooth LE functions
    *
    * @tparam OsModel_P Has to implement @ref os_concept "Os concept".
    */

   template<typename OsModel_P>
   class AndroidBleRadio : public ExtendedRadioBase<OsModel_P, jobject, uint8_t, uint8_t> //OsModel, NodeId, Size, BlockData
   {
   public:
      typedef OsModel_P OsModel;

      typedef AndroidBleRadio<OsModel> self_type;
      typedef self_type* self_pointer_t;

      typedef jobject node_id_t;    //Type of node id - must be unique for a node in the network; a pointer to a Java BluetoothDevice is used for that
      typedef uint8_t block_data_t; //Data type used for raw data in message sending process. 
      typedef uint8_t size_t;       //Unsigned integer that represents length information. 
      typedef BaseExtendedData<OsModel> ExtendedData; //extended data: link quality

      enum ErrorCodes
      {
         SUCCESS = OsModel::SUCCESS,
         ERR_UNSPEC = OsModel::ERR_UNSPEC,
         ERR_NOTIMPL = OsModel::ERR_NOTIMPL
      };
      enum SpecialNodeIds
      {
         // BROADCAST_ADDRESS   // Broadcasting is not supported since sending is not supported!
         NULL_NODE_ID           = 0  // Unknown/No node id
      };
      enum Restrictions
      {
         MAX_MESSAGE_LENGTH = 0 //sending data is not supported
      };

      /**
       * Sends data to a node
       * THIS FUNCTION IS NOT IMPLEMENTED!
       */
      int send( node_id_t id, size_t len, block_data_t* data )
      {
         return ERR_NOTIMPL;
      }

      /**
       * Turn on radio.
       * Messages can be sent and received.
       */
      int enable_radio()
      {
         return jni_env_->CallBooleanMethod(wiselib_activity_, method_enable_) ? SUCCESS : ERR_UNSPEC;
      }


      /**
       * Turn off radio.
       * Messages canot be sent and received.
       */
      int disable_radio()
      {
         return jni_env_->CallBooleanMethod(wiselib_activity_, method_disable_) ? SUCCESS : ERR_UNSPEC;
      }


      /**
       * Return id of node for current radio.
       */
      node_id_t id()
      {
         return this;  //node_id_t is a pointer to jobject i.e. a memory adress. so this adress is unique
      }

      /**
       * Constructor
       */
      AndroidBleRadio()
      {
         ;
      }

      void onBleDataRecv(JNIEnv* env, jobject device, jbyteArray data, jint rssi)
      {
         jbyte* raw_data = env->GetByteArrayElements(data, NULL);
         jsize len = env->GetArrayLength(data);
         extended_data_.set_link_metric( ((uint16_t) rssi)+(2<<31));  //TODO Android passes a SIGNED int, link metric is defined as UNSINGED int
         ExtendedRadioBase<OsModel_P, node_id_t, size_t, block_data_t>
            ::notify_receivers((node_id_t) device, (size_t) len, (block_data_t*) raw_data, extended_data_);
         env->ReleaseByteArrayElements(data, raw_data, 0);
      }

      //TODO: move this to constructor
      void setup(JNIEnv* jni_env, jobject wiselib_activity)
      {
         jni_env_ = jni_env;
         wiselib_activity_ = wiselib_activity;
         jclass wiselib_class_ = jni_env_->FindClass("com/ibralg/wiselib/WiselibActivity");
         method_enable_  = jni_env_->GetMethodID(wiselib_class_, "enableBluetoothLe", "()Z");
         method_disable_ = jni_env_->GetMethodID(wiselib_class_, "disableBluetoothLe", "()Z");
         onBleDataReceive = delegate4<void, JNIEnv*, jobject, jbyteArray, jint>::
            from_method<AndroidBleRadio<OsModel>, &AndroidBleRadio<OsModel>::onBleDataRecv>(this);
	  //(Ljava/lang/String;)
      }

      /**
       * Destructor
       * Disables the radio and frees all used resoources
       */
      ~AndroidBleRadio()
      {
         disable_radio();
      }

   private:
      // Java Native Interface
      JNIEnv* jni_env_;  //TODO: jni_env_ must NOT be shared between different threads!!!
      jobject wiselib_activity_;
      jmethodID method_enable_;
      jmethodID method_disable_;

      ExtendedData extended_data_;
   };

}

extern "C" {
   JNIEXPORT void JNICALL Java_com_ibralg_wiselib_WiselibActivity_onBleDataReceive(JNIEnv* env, jobject object, jobject dev, jbyteArray data, jint rssi)
   {
      onBleDataReceive(env, dev, data, rssi);
   }
}

#endif
