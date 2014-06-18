LOCAL_PATH := $(call my-dir)
WISELIB_PATH := $(LOCAL_PATH)/../../../../..

WISELIB_STABLE=$(WISELIB_PATH)/wiselib.stable
WISELIB_TESTING=$(WISELIB_PATH)/wiselib.testing

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(WISELIB_TESTING) $(WISELIB_STABLE)

LOCAL_CFLAGS    := -DOSMODEL=AndroidOsModel
LOCAL_MODULE    := android_main
LOCAL_SRC_FILES := $(wildcard  *.cpp)
LOCAL_LDLIBS    := -llog -landroid

include $(BUILD_SHARED_LIBRARY)
