#ifndef _ULTRA_SND_PARROT6
#define _ULTRA_SND_PARROT6

struct parrot_ultra_snd_platform_data {
	int 			gpio_mux_vbat_jump;	/* gpio num to swith vbat/jump on JPSumo */
	int 			gpio_mux_wheels;	/* gpio num to swith wheels on JPSumo */
};

#endif
