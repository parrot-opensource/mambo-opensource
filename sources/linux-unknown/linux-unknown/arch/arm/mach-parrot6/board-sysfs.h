#ifndef _BOARD_SYSFS_H_
#define _BOARD_SYSFS_H_

#include <linux/init.h>

#ifdef CONFIG_SYSFS

extern void __init p6i_export_i2c_hw_infos(int i2c_bus,
					     unsigned short addr,
					     char *version,
					     const char *extra);

extern void __init p6i_export_uart_hw_infos(int uart,
					      int rts_cts,
					      char *version);


#else
/* No sysfs : info on dmesg only */
static inline void p6i_export_i2c_hw_infos(int i2c_bus,
					     unsigned short addr,
					     char *version,
					     const char *extra)
{
	printk(KERN_INFO "hw_info : i2c-%d device 0x%02X (%s) : %s\n",
	       i2c_bus, addr, version, extra);
}

static inline void p6i_export_uart_hw_infos(int uart,
					      int rts_cts,
					      char *version)
{
	printk(KERN_INFO "hw_info : uart %d (%s) rts/cts %d\n",
	       uart, version, rts_cts);
}

#endif

#endif /* _BOARD_SYSFS_H_ */
