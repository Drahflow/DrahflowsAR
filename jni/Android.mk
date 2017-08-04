LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libsvo
LOCAL_SRC_FILES := ../svo/lib/libsvo.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE    := ar_jni
LOCAL_SRC_FILES := ar_gesture.cpp ar_svo.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../svo/include
LOCAL_CPPFLAGS := -std=c++11 -fexceptions
LOCAL_SHARED_LIBRARIES := libsvo
LOCAL_LDLIBS := -llog
include $(BUILD_SHARED_LIBRARY)
