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

#ifndef ANDROID_OS_MODEL_H
#define ANDROID_OS_MODEL_H

#include <stdint.h>
#include <jni.h>

#include "external_interface/default_return_values.h"

#include "android_onload.h"
#include "android_debug.h"
#include "android_timer.h"
#include "android_wlan_radio.h"
#include "android_clock.h"
#include "android_accelerometer.h"
#include "android_battery.h"
#include "android_light_sensor.h"
#include "android_proximity_sensor.h"
#include "android_magnetic_sensor.h"
#include "android_ble_radio.h"
#include "util/serialization/endian.h"

namespace wiselib
{
   /** \brief Android implementation of \ref os_concept "Os Concept".
    *
    * \ingroup os_concept
    * \ingroup basic_return_values_concept
    * \ingroup android_facets
    */
   class AndroidOsModel
      : public DefaultReturnValues<AndroidOsModel>
   {
   public:

      typedef AndroidOsModel AppMainParameter;
      typedef AndroidOsModel Os;

      typedef uint32_t size_t;
      typedef uint8_t block_data_t;

      typedef AndroidDebug<AndroidOsModel> Debug;
#ifdef ANDROID_USE_BLUETOOTH_LE_RADIO
      typedef AndroidBleRadio<AndroidOsModel> BleRadio;
      typedef BleRadio Radio;
#else
      typedef AndroidWlanRadio<AndroidOsModel> WlanRadio;
      typedef WlanRadio Radio;
#endif
      typedef AndroidTimer<AndroidOsModel> Timer;
      typedef AndroidClock<AndroidOsModel> Clock;
      typedef AndroidAccelerometer<AndroidOsModel> Accelerometer;
      typedef AndroidBattery<AndroidOsModel> Battery;
      typedef AndroidLightSensor<AndroidOsModel> LightSensor;
      typedef AndroidProximitySensor<AndroidOsModel> ProximitySensor;
      typedef AndroidMagneticSensor<AndroidOsModel> MagneticSensor;
      // ----------------------------------------------------------
      AndroidOsModel(JNIEnv* jni_env_, jobject wiselib_activity_)
      {
         jni_env = jni_env_;
         wiselib_activity = jni_env->NewGlobalRef(wiselib_activity_);  //TODO this must be released somewhere via env->DeleteGlobalRef(w_a);
      }

      //-----------------------------------------------------------
      int argc;
      const char** argv;

      JNIEnv* jni_env;          //the JavaNativeInterface is used to access Java code from native code
      jobject wiselib_activity; //the actvity hosting the native code

      // --------------------------------------------------------------------
      static const Endianness endianness = WISELIB_ENDIANNESS;
      // --------------------------------------------------------------------
   };
}

#endif
