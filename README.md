====== Wiselib: implementing iBeacon support ======

This repository has been forked from:
  https://github.com/ibr-alg/wiselib

As a result almost every source has been copied from there, expect the following:
 * the Android port has been merged into it: https://github.com/rraf/wiselib (branch: android) so that these files have been copied from there:
     wiselib.testing/external_interface/android/*
     wiselib.testing/algorithms/neighbor_discovery/echo_jmdns.h
     apps/generic_apps/Makefile.android
     apps/android_apps/example_app

 * new file have been added to implement iBeacon support:
     wiselib.testing/external_interface/android/android_ble_radio.h
     apps/android_apps/bluetooth_le_app/*
     [wiselib.testing/external_interface/arduino/_arduino_ibeacon_radio.h (deprecated)]
     wiselib.testing/external_interface/arduino/arduino_ble_radio.h
     apps/arduino_apps/bluetooth_le_app/*

For more information, see
	http://www.wiselib.org/

-------------------
How do I test it?
-------------------

 * on Android:
	iBeacon detection:
		Compile apps/android_apps/bluetooth_le_app/ and run it on an BLE capable Android device (Android 4.3+ needed)
	Act as an iBeacon:
		(not possible due to Android API)
 * on Arduino:
	iBeacon detection + Act as an iBeacon:
		You need the ITEAD BLE SHIELD (http://imall.iteadstudio.com/development-platform/arduino/shields/im130704001.html).
		Flash my custom firmware which is available at https://github.com/bjoerke/HM-10-Firmware
		Now set up the Jumpers correctly as descriped in apps/arduino_apps/bluetooth_le_app/ble_shield_test/ble_shield_test.ino
		Finally you can compile apps/arduino_apps/bluetooth_le_app and flash it onto an Arduino (tested with Uno)

