#ifndef _MMC_PARROT6
#define _MMC_PARROT6

struct parrot_mmc_platform_data {
	unsigned int ocr_mask;
	unsigned int wp_pin;
	unsigned int cd_pin;
};

#endif
