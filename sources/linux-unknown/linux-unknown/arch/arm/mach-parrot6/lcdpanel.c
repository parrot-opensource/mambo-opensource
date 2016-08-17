/* LCD driver info */
#include <mach/fb.h>

struct p6fb_mach_info p6_lcd_devices_info_7p_avea = {
	/* does it make sense to have min & max ? */
	.yres		= {
		.min	= 480,
		.max	= 480,
		.defval	= 480,
	},

	.xres		= {
		.min	= 800,
		.max	= 800,
		.defval = 800,
	},

	.bpp		= {
		.min	= 16,
		.max	= 32,
		.defval = 16,
	},

	.pixclock	= {
		.min	= 40816,
		.max	= 40816,
		.defval = 40816,
	},
	.left_margin = 16-3,
	.right_margin = 8,
	.upper_margin = 8-1  + 1 /* vsync pol bug */,
	.lower_margin = 5,
	.hsync_len = 3,
	.vsync_len = 1,

	.panel = 0x00a00,
};
struct p6fb_mach_info p6_lcd_devices_info_10p_parelia /*__initdata*/ = {
	/* does it make sense to have min & max ? */
	.yres		= {
		.min	= 600,
		.max	= 600,
		.defval	= 600,
	},

	.xres		= {
		.min	= 800,
		.max	= 800,
		.defval = 800,
	},

	.bpp		= {
		.min	= 16,
		.max	= 32,
		.defval = 16,
	},

	.pixclock	= {
		.min	= 25000,
		.max	= 25000,
		.defval = 25000,
	},
	.left_margin = 88,
	.right_margin = 40,
	.upper_margin = 27/*-3*/, /* because we don't inverse vsync, upper_margin
								 is Vsync pulse + front porch. */
	.lower_margin = 1,
	.hsync_len = 128,
	.vsync_len = 3,

	.panel = 0x10a00,
};

struct p6fb_mach_info p6_lcd_devices_info_rnb4 = {
	/* does it make sense to have min & max ? */
	.yres		= {
		.min	= 240,
		.max	= 240,
		.defval	= 240,
	},

	.xres		= {
		.min	= 400,
		.max	= 400,
		.defval = 400,
	},

	.bpp		= {
		.min	= 16,
		.max	= 16,
		.defval = 16,
	},
	.pixclock	= {
		.min	= 130000, // D: 7.800 MHz, H: 19.307 kHz, V: 78.803 Hz
		.max	= 130000,
		.defval = 130000,
	},
/*
	.pixclock	= {
		.min	= 163264, // D: 6.500 MHz, H: 16.089 kHz, V: 65.669 Hz
		.max	= 163264,
		.defval = 163264,
	},
*/
	.left_margin = 1,
	.right_margin = 1,
	.upper_margin = 2 /* vsync pol bug */,
	.lower_margin = 0,
	.hsync_len = 2,
	.vsync_len = 2,

	.panel = 0x01060B,
};

//LCD OSD040JB1CW (see www.osddisplays.com) for more informations
struct p6fb_mach_info p6_lcd_devices_info_fc6xxx = {
	/* does it make sense to have min & max ? */
	.yres		= {
		.min	= 272,
		.max	= 272,
		.defval	= 272,
	},

	.xres		= {
		.min	= 480,
		.max	= 480,
		.defval	= 480,
	},

	.bpp		= {
		.min	= 16,
		.max	= 32,
		.defval = 16,
	},

	.pixclock	= {
		.min	= 111111,
		.max	= 111111,
		.defval = 111111, //# D: 9.750 MHz, H: 18.571 kHz, V: 64.708 Hz
	},

	.left_margin = 2,                              //Horizontal front porch
	.right_margin = 2,                             //Horizontal back porch
	.upper_margin = 2  + 1 /* vsync pol bug */,    //Vertical   front porch
	.lower_margin = 2,                             //Vertical   back porch
	.hsync_len = 41,
	.vsync_len = 10,

	.panel = 0x00a00,
};
