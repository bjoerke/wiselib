# path to android SDK
SDK_PATH=~/wiselib-Android/android-sdk-linux

# path to android NDK
NDK_PATH=$(SDK_PATH)/ndk

# desired minimum priority of log messages to display
#    V — Verbose (lowest priority)
#    D — Debug
#    I — Info
#    W — Warning
#    E — Error
#    F — Fatal
#    S — Silent (highest priority, on which nothing is ever printed)
LOG_PRIORITY=D

# Android development uses ANT
# for further details see
# https://developer.android.com/tools/building/building-cmdline.html

export NDK_PATH

.PHONY: default app upload debug jni-libs logcat

default: jni-libs app debug

jni-libs:
	make -C jni

app:
	ant debug

upload:
	ant installd
	$(SDK_PATH)/platform-tools/adb shell am start -n com.ibralg.wiselib/com.ibralg.wiselib.WiselibActivity

logcat:
	#$(SDK_PATH)/platform-tools/adb logcat *:$(LOG_PRIORITY)
	$(SDK_PATH)/platform-tools/adb logcat WiselibDebug:D *:S | ./coloredLogcat.py
	#$(SDK_PATH)/platform-tools/adb logcat *:D | ./coloredLogcat.py

debug: upload logcat

clean:
	ant clean

# fixes this error when trying to upload:
# error: insufficient permissions for device
# this restarts the adb service with root rights
fix-adb:
	$(SDK_PATH)/platform-tools/adb kill-server
	sudo $(SDK_PATH)/platform-tools/adb start-server

