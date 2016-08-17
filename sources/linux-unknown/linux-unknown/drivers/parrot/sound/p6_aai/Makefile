obj-$(CONFIG_AAI_PARROT6) += aai.o

EXTRA_CFLAGS += -I$(src)/p6 -I$(src)/p6i -I$(src)

aai-objs := \
    aai_alsa.o              \
    aai_fiq.o               \
    aai_hw.o                \
    aai_irq_dma.o           \
    aai_mixer.o             \
    aai_module.o            \
    aai_ops.o               \
    p6/aai_hw_p6.o          \
    p6/aai_hw_struct_p6.o   \
    p6/aai_ioctl_p6.o       \
    p6/aai_irq_dma_p6.o     \
    p6/aai_mixer_p6.o       \
    p6i/aai_codec_p6i.o     \
    p6i/aai_hw_p6i.o        \
    p6i/aai_hw_struct_p6i.o \
    p6i/aai_ioctl_p6i.o     \
    p6i/aai_irq_dma_p6i.o   \
    p6i/aai_mixer_p6i.o

ifeq ($(CONFIG_AAI_PARROT6_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
EXTRA_CFLAGS += -O0 -Wall
endif

