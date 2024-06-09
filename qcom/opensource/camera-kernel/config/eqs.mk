# Settings for compiling waipio camera architecture

# Localized KCONFIG settings
# Camera: Remove for user build
ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
CONFIG_CCI_DEBUG_INTF := y
ccflags-y += -DCONFIG_CCI_DEBUG_INTF=1
endif
CONFIG_CAM_SENSOR_PROBE_DEBUG := y
CONFIG_CAM_SENSOR_CTLE_FOR_FRONTIER := y
CONFIG_AF_NOISE_ELIMINATION := y

# Flags to pass into C preprocessor
ccflags-y += -DCONFIG_CAM_SENSOR_PROBE_DEBUG=1
ccflags-y += -DCONFIG_CAM_SENSOR_CTLE_FOR_FRONTIER=1

ccflags-y += -DCONFIG_MOT_OIS_AF_DRIFT=1
ccflags-y += -DCONFIG_AF_NOISE_ELIMINATION=1
