DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq (,$(filter userdebug eng,$(TARGET_BUILD_VARIANT)))
       KERNEL_CFLAGS += CONFIG_QM35_SPI_DEBUG_FW=y
       KBUILD_OPTIONS += CONFIG_QM35_SPI_DEBUG_FW=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := qm35.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_info.ko
include $(DLKM_DIR)/AndroidKernelModule.mk
