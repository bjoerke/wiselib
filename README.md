= Wiselib: implementing iBeacon support =

This repository has been forked from:
>  https://github.com/ibr-alg/wiselib

As a result almost every source has been copied from there, expect the following:
* the Android port has been merged into it: https://github.com/rraf/wiselib (branch: android) so that these files have been copied from there:
  * wiselib.testing/external_interface/android/*
  * wiselib.testing/algorithms/neighbor_discovery/echo_jmdns.h
  * apps/generic_apps/Makefile.android
  * apps/android_apps/example_app

* new files have been added:
  * BLE for Android:
    * wiselib.testing/external_interface/android/android_ble_radio.h
    * apps/android_apps/bluetooth_le_app/*
  * BLE for Arduino:
    * wiselib.testing/external_interface/arduino/arduino_ble_radio.h
    * apps/arduino_apps/bluetooth_le_app/*
  * iBeacon based localization:
    * wiselib.testing/algorithms/localization/link_metric_based/*

# How do I test it?

## On Android

Compile apps/android_apps/bluetooth_le_app/ and run it on an BLE capable Android device (Android 4.3+ needed). This app uses iBeacons for indoor localization. It cannot turn your Android device into an iBeacon due to Api limitations.

## On Arduino
You need the ITEAD BLE SHIELD (http://imall.iteadstudio.com/development-platform/arduino/shields/im130704001.html).
Flash my custom firmware which is available at https://github.com/bjoerke/HM-10-Firmware
Now set up the Jumpers correctly as descriped in apps/arduino_apps/bluetooth_le_app/ble_shield_test/ble_shield_test.ino
Finally you can compile apps/arduino_apps/bluetooth_le_app and flash it onto an Arduino (tested with Uno)


