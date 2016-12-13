obj-$(CONFIG_AAI_PARROT5P) += aai.o

aai-objs := aai_module.o aai_hw.o aai_irq.o aai_alsa.o aai_ops.o aai_mixer.o aai_ioctl.o

ifeq ($(CONFIG_AAI_PARROT5P_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
EXTRA_CFLAGS += -DDEBUG
EXTRA_CFLAGS += -O0


