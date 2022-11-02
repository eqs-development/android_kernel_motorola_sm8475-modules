# Settings for compiling cape/felix camera architecture

# Localized KCONFIG settings
# Camera: Remove for user build
ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
CONFIG_CCI_DEBUG_INTF := y
ccflags-y += -DCONFIG_CCI_DEBUG_INTF=1
endif
CONFIG_CCI_ADDR_SWITCH := y
CONFIG_CAM_SENSOR_PROBE_DEBUG := y

# Flags to pass into C preprocessor
ccflags-y += -DCONFIG_CCI_ADDR_SWITCH=1
ccflags-y += -DCONFIG_CAM_SENSOR_PROBE_DEBUG=1
