EXTRA_CFLAGS += -DFPNUM='"144"' -DREASSOCIATION

ifeq ($(CONFIG_88W8688_WLAN_REASSOCIATION),y)
EXTRA_CFLAGS += -DREASSOCIATION
endif

ifeq ($(CONFIG_88W8688_WLAN_MFG_CMD_SUPPORT),y)
EXTRA_CFLAGS +=-DMFG_CMD_SUPPORT
endif

ifeq ($(CONFIG_88W8688_WLAN_DEBUG),y)
EXTRA_CFLAGS += -O0 -DDEBUG_LEVEL1
endif

obj-$(CONFIG_88W8688_WLAN) += 88w8688_wlan.o

WLANOBJS +=	wlan_main.o	\
		wlan_init.o	\
		wlan_wext.o	\
		wlan_rx.o	\
		wlan_tx.o	\
		wlan_cmd.o	\
		wlan_cmdresp.o	\
		wlan_scan.o	\
		wlan_join.o	\
		wlan_priv.o	\
		wlan_wmm.o	\
		wlan_11d.o	\
		wlan_11h.o	\
		wlan_meas.o

WLANOBJS +=	wlan_sdio_mmc.o

ifdef CONFIG_PROC_FS
WLANOBJS += wlan_proc.o
ifeq ($(CONFIG_88W8688_WLAN_PROC_DEBUG),y)
WLANOBJS += wlan_debug.o
EXTRA_CFLAGS += -DPROC_DEBUG
endif
endif

88w8688_wlan-objs := $(WLANOBJS)
