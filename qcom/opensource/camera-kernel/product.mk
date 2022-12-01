ifneq ($(BOARD_USES_LEGACY_CAMERA), true)
ifneq ($(TARGET_USES_QMAA),true)
PRODUCT_PACKAGES += camera.ko
endif
endif # !BOARD_USES_LEGACY_CAMERA
