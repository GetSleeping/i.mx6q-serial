LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        xtev-serial-main.c \
        xtev-serial-parse.c \

LOCAL_MODULE_TAGS := eng

LOCAL_SHARED_LIBRARIES := \


LOCAL_C_INCLUDES:= $(TARGET_HARDWARE_INCLUDE)

LOCAL_MODULE:= xtev-serial-main

include $(BUILD_EXECUTABLE)
