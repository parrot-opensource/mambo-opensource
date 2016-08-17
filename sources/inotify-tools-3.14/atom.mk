
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := inotify-tools
LOCAL_DESCRIPTION := command line utilities based on inotify: \
	inotifywait simply blocks for inotify events, making it \
	appropriate for use in shell scripts. \
	inotifywatch collects filesystem usage statistics and \
	outputs counts of each inotify event.

LOCAL_CATEGORY_PATH := devel

LOCAL_AUTOTOOLS_VERSION := 3.14
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	inotify-tools-3.14-static-lib.patch

include $(BUILD_AUTOTOOLS)

