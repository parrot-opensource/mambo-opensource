#ifndef _PLAT_PARROT_JPSUMO_ADC_H
#define _PLAC_PARROT_JPSUMO_ADC_H

struct parrot_jsadc_platform_data {
	int gpio_mux_vbat_jump;	/* gpio num to swith vbat/jump on JPSumo */
	int gpio_mux_wheels;	/* gpio num to swith wheels on JPSumo */
};

#endif
