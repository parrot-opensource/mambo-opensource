#ifndef _AAI_PARROT6
#define _AAI_PARROT6

struct parrot_aai_platform_data {
	unsigned int i2s_bit_clock_control; /* 1 : enable clk when needed */
	unsigned int i2s_master_clock_control; /* 1 : enable clk when needed */
	unsigned int sync_freq; /* 44100 or 48000 */
};

#endif
