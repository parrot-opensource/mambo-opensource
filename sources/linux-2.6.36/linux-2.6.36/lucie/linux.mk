#############################################################
#
# linux
#
#############################################################

LINUX_DIR:=$(PACKAGE_DIR)/linux
LINUX_EXPORTED_HEADERS := \
	$(LINUX_DIR)/drivers/parrot/sound/p6_aai/aai_ioctl.h \
	$(LINUX_DIR)/drivers/parrot/video/p6fb_ioctl.h \
	$(LINUX_DIR)/drivers/parrot/char/dmamem_ioctl.h \
	$(LINUX_DIR)/drivers/parrot/char/pwm/pwm_ioctl.h \
	$(LINUX_DIR)/drivers/parrot/char/gpio2_ioctl.h

ifndef LINUXCONF_MK
	$(error LINUXCONF_MK is not defined)
endif

include $(LINUXCONF_MK)
