LOCAL_PATH := $(call my-dir)
WISELIB_PATH := /home/wiselib/wiselib

WISELIB_STABLE=$(WISELIB_PATH)/wiselib.stable
WISELIB_TESTING=$(WISELIB_PATH)/wiselib.testing

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(WISELIB_TESTING) $(WISELIB_STABLE)

LOCAL_CPPFLAGS := -U__unix__

LOCAL_MODULE    := example_app
LOCAL_SRC_FILES := example_app.cpp
LOCAL_LDLIBS    := -llog -landroid

include $(BUILD_SHARED_LIBRARY)
