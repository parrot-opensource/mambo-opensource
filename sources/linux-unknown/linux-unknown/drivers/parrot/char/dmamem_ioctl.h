/**
********************************************************************************
* @file dmamem_ioctl.h
* @brief dmamem ioctl
*
* Copyright (C) 2009 Parrot S.A.
*
********************************************************************************
*/

#ifndef _DMAMEM_IOCTL_H
#define _DMAMEM_IOCTL_H 1

#include <asm/ioctl.h>


struct dmamem_alloc {
	int size; /* size of the block to allocate */
	void *cpu_addr; /* unused */
	void *phy_addr; /* physical address of memory */
};

struct dmamem_flush_inv {
	unsigned long start;
	unsigned long len;
};

#define DMAMEM_MAGIC 'p'
/** DMAMEM_ALLOC
 * allocate dma memory and return physical adress
 *
 * @see dmamem_alloc
 */
#define DMAMEM_ALLOC _IOWR(DMAMEM_MAGIC, 0, struct dmamem_alloc)
/** DMAMEM_ARM_FLUSH_INV
 * flush and invalidate memory allocated by DMAMEM_ALLOC
 *
 * @see dmamem_alloc
 */
#define DMAMEM_ARM_FLUSH_INV _IOWR(DMAMEM_MAGIC, 1, struct dmamem_flush_inv)

#endif
