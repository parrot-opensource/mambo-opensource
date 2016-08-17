/**
 *
 *       @file  onfi.h
 *
 *      @brief  
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  30-Apr-2009
 *
 *        $Id: onfi.h,v 1.4 2009-05-29 09:21:55 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#ifndef _ONFI_H
#define _ONFI_H

#include <linux/list.h>

struct onfi_timings {
	signed char    tALH;
	signed char    tALS;
    signed char    tCEA;
	signed char    tCEH;
	signed char    tCH;
	signed char    tCLH;
	signed char    tCLR;
	signed char    tCLS;
	signed char    tCS;
	signed char    tDH;
	signed char    tDS;
	signed char    tRC;
	signed char    tREA;
	signed char    tREH;
	signed short   tRHZ;
	signed char    tRP;
	signed char    tRR;
	signed char    tWC;
	signed char    tWH;
	signed char    tWHR;
	signed char    tWP;
    signed short   toRST;
    signed short   twRST;
    signed short   teRST;
};

struct onfi_setting {
    struct  list_head       node;
    char const*             name;
    int mode;
    unsigned                read_cache:1;
    unsigned                cache_prg:1;
    unsigned                edo:1;
    unsigned feature:1;
    struct onfi_timings     timings;
};

#define ONFI_READID_CMD         0x90
#define ONFI_READID_ADDR        0x20
#define ONFI_READPARM_CMD       0xEC
#define ONFI_SETFEAT_CMD       0xEF
#define ONFI_READPARM_ADDR      0x00
#define ONFI_NOOP_CMD           -1
#define ONFI_NOOP_ADDR          -1
#define ONFI_READCACHE_CMD      0x31
#define ONFI_READCACHE_END_CMD  0x3f
#define ONFI_CACHEPRG_CMD       0x15

struct mtd_info;

struct onfi_operations {
    int     (*read_mtd)(char*, struct mtd_info*, int, int, size_t);
    void    (*init_mtd)(struct mtd_info*, struct onfi_setting*);
};

extern int onfi_enable(struct mtd_info*,
                       struct onfi_operations const*);

#endif

/* vim:ts=4:sw=4:et:syn=c */
