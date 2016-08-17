
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libnl
LOCAL_DESCRIPTION := Utility library for netlink sockets monitoring / \
	manipulation
LOCAL_CATEGORY_PATH := libs

LOCAL_EXPORT_C_INCLUDES := $(TARGET_OUT_STAGING)/usr/include/libnl3/

# mandatory because as-needed is activated by alchemy
# if activated, the .so constructors aren't called and their respective caches
# aren't registered
# the default alchemy behaviour is then reinstalled
LOCAL_EXPORT_LDLIBS := -Wl,--no-as-needed \
	-lnl-3 -lnl-route-3 -lnl-genl-3 -lnl-nf-3 -lnl-cli-3\
	-Wl,--as-needed

include $(BUILD_AUTOTOOLS)

