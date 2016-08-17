#ifndef P6FB_IOCTL_H
#define P6FB_IOCTL_H 1

#include <linux/fb.h>

/* public interface */
enum {
	NOSTD_RGB = 0,
	NOSTD_YUV420,
	NOSTD_YVU420,
};

enum {
	P6FB_OSDMODE_NONE = 0,
	P6FB_OSDMODE_PLANE,
	P6FB_OSDMODE_PIXEL,
};

struct p6fb_plane_info {
	__u32 pos_x;
	__u32 pos_y;
	__u32 scale; /* 0, 1, 2 */
	__u32 osdmode;
	__u8 osdplane;
};

enum {
	P6FB_RGBMODE_NONE = 0,
	P6FB_RGBMODE_UNIFILL,
	P6FB_RGBMODE_BRIGHTNESS,
};

struct p6fb_rgb_ctl {
	int mode;
	__u8 red;
	__u8 blue;
	__u8 green;
};

struct p6fb_ycc_to_rgb {
	__u32 cr;
	__u32 cb;
};

struct p6fb_fbinfo {
	__u32 map_dma;
};

#define P6FB_SET_RGB_CTL	_IOW('F', 0xFF, struct p6fb_rgb_ctl)
#define P6FB_GET_RGB_CTL	_IOR('F', 0xFE, struct p6fb_rgb_ctl)
#define P6FB_SETUP_PLANE	_IOW('F', 0xFD, struct p6fb_plane_info)
#define P6FB_QUERY_PLANE	_IOR('F', 0xFC, struct p6fb_plane_info)
#define P6FB_SET_YCC_TO_RGB	_IOW('F', 0xFB, struct p6fb_ycc_to_rgb)
#define P6FB_GET_YCC_TO_RGB	_IOR('F', 0xFA, struct p6fb_ycc_to_rgb)
#define P6FB_GET_FBINFO		_IOR('F', 0xF9, struct p6fb_fbinfo)

#endif
