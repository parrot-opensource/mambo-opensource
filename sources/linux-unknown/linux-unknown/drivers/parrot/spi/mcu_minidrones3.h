
#ifndef __MCU_MINIDRONES3
#define __MCU_MINIDRONES3
#include <linux/input.h>

struct mcu_minidrones3_key {
	unsigned int bmask;
	int input_key;
};

struct mcu_minidrones3_platform_data {
	int gpio_rst;
	unsigned long spi_read_delay_ms;
	char *firmware_name;
	int keys_nb;
	struct mcu_minidrones3_key *keys;
};

static inline int mcu_minidrones3_driver_enabled(void)
{
#ifdef CONFIG_SPI_MCU_MINIDRONES3
	return true;
#else
	return false;
#endif
}

#endif /* __MCU_MINIDRONES3 */
