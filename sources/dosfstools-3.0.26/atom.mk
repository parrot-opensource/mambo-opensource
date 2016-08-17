LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE            := dosfstools
LOCAL_DESCRIPTION       :=                                          \
	The dosfstools package includes the mkdosfs (aka mkfs.dos and   \
	mkfs.vfat) and dosfsck (aka fsck.msdos and fsck.vfat) utilities,\
	which respectively make and check MS-DOS FAT filesystems on hard\
	drives or on floppies.                                          \
    See http://www.daniel-baumann.ch/software/dosfstools
LOCAL_CATEGORY_PATH := fs

LOCAL_AUTOTOOLS_VERSION := 3.0.26
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_CFLAGS := -fomit-frame-pointer -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64

LOCAL_AUTOTOOLS_PATCHES := no_salvage_to_files.patch

# No configure step
define LOCAL_AUTOTOOLS_CMD_CONFIGURE
	$(empty)
endef

# With no configure steps, give make build/install full env
LOCAL_AUTOTOOLS_MAKE_BUILD_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_INSTALL_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

# LOCAL_CFLAGS is only given in configure step (that we don't have), add it
# in make build
LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	PREFIX="$(AUTOTOOLS_CONFIGURE_PREFIX)" \
	CFLAGS+="$(LOCAL_CFLAGS)"

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	PREFIX="$(AUTOTOOLS_CONFIGURE_PREFIX)"

include $(BUILD_AUTOTOOLS)
