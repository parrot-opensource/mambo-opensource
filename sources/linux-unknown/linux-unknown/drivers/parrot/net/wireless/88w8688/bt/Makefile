obj-$(CONFIG_88W8688_BT) += 88w8688_bt.o

ifeq ($(CONFIG_88W8688_BT_DEBUG),y)
EXTRA_CFLAGS += -O0 -DDEBUG_LEVEL1
endif

EXTRA_CFLAGS += -DFPNUM='"44"' -I$(src)/../include

BTOBJS +=	bt_main.o	\
		bt_proc.o

BTOBJS +=	bt_sdiommc.o

88w8688_bt-objs := $(BTOBJS)
