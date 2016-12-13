ifeq ("$(TARGET_OS)","linux")
ifneq (,$(filter $(TARGET_CPU),p6 p6i))

ifndef BUILD_LINUX
$(error update alchemy to version 1.0.5 or more)
endif

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := linux
LOCAL_CATEGORY_PATH := system

LINUX_EXPORTED_HEADERS := \
	$(LOCAL_PATH)/include/linux/videodev.h \
	$(LOCAL_PATH)/drivers/parrot/sound/p6_aai/aai_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/video/p6fb_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/char/dmamem_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/char/pwm/pwm_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/ultra_sound/ultra_snd.h \
	$(LOCAL_PATH)/drivers/parrot/pressure/lps22hb.h \
	$(LOCAL_PATH)/drivers/parrot/char/gpio2_ioctl.h \
	$(LOCAL_PATH)/arch/arm/plat-parrot/include/mach/delos_hwrev.h \
	$(LOCAL_PATH)/arch/arm/plat-parrot/include/mach/powerup_hwrev.h \
	$(LOCAL_PATH)/arch/arm/plat-parrot/include/mach/jpsumo_hwrev.h

# Linux configuration file
LINUX_DEFAULT_CONFIG_FILE := $(LOCAL_PATH)/lucie/config/kernel_P6.config

include $(BUILD_LINUX)

include $(CLEAR_VARS)

LOCAL_MODULE := perf
LOCAL_CATEGORY_PATH := devel

include $(BUILD_LINUX)


endif
endif
