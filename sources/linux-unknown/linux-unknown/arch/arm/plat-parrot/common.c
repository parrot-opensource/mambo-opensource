#include <linux/module.h>
#include <linux/types.h>
#include <mach/parrot.h>

#ifndef parrot_chipid
u32 parrot_chipid;
EXPORT_SYMBOL(parrot_chipid);
#endif
