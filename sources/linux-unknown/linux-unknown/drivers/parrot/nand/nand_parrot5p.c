/**
*******************************************************************************
* @file  nand_parrot5p.c
* @brief NAND flash interface driver for Parrot5+-based platforms.
*
* This file provides a low level interface implementation for the generic
* NAND flash driver.
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     ivan.djelic@parrot.fr
* @date       2006-03-21
*
* Porting to mtd linux
* @author     matthieu.castet@parrot.com
* @date       2007-07-15
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*******************************************************************************
*/

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/ioport.h>

#include <asm/io.h>
/* register def */
#include <mach/platform.h>
/* static mapped device */
#include <mach/map.h>

#include "nand_parrot_ecc.h"

/*
 * Parrot5+ NAND signals are set/cleared with special memory accesses to
 * address 0x500000XX, with the following bit correspondences:
 *
 * XX = A4.A3.A2.A1.A0
 *
 * A0 = nCS
 * A1 = AL
 * A2 = CL
 * A3 = 0 => set or clear nR, nW signals according to access type (read/write)
 * A3 = 1 => do not modify nR, nW signals (both set high)
 * A4 must be set to 1 to access NAND
 *
 * D0-D7/D15 are connected to NAND data bus, and MPMC should be configured in
 * 8-bit or 16-bit mode with proper wait-states.
 */

#define NAND_nCS                         (1 << 0)
#define NAND_AL                          (1 << 1)
#define NAND_CL                          (1 << 2)
#define NAND_NORW                        (1 << 3)
#define NAND_SELECT                      (1 << 4)

//! Enable chip select and clear CL,AL; do not activate nR,nW signals
#define NAND_SELECT_CLEAR(_dev_)         NAND_IF_WRITE(_dev_, NAND_NORW, 0)

//! Write a word to NAND interface
#define NAND_IF_WRITE( _dev_, _sig_, _word_ )                               \
    do {                                                                    \
        if ( (_dev_)->chip.options & NAND_BUSWIDTH_16) {                    \
            writew(_word_, (_dev_)->chip.IO_ADDR_W + ((_sig_) << 1));       \
        }                                                                   \
        else {                                                              \
            writeb(_word_, (_dev_)->chip.IO_ADDR_W + (_sig_))  ;            \
        }                                                                   \
    } while (0)


#define P5P_NAME "nand_p5p"

struct nand_p5p_device {
    struct mtd_info mtd;
    struct nand_chip chip;
    /* device info */
    void __iomem            *regs;
    struct resource         *area;
};

#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/**
 * Issue command and address cycles to the chip
 */
static void nand_p5p_hwcontrol(struct mtd_info *mtd, int cmd,
        unsigned int ctrl)
{
    struct nand_chip *this = mtd->priv;
    struct nand_p5p_device *dev = this->priv;

    if (cmd == NAND_CMD_NONE)
        return;

    if (ctrl & NAND_CLE) {
        NAND_IF_WRITE(dev, NAND_CL, cmd);
        NAND_SELECT_CLEAR(dev);
    }
    else {
        NAND_IF_WRITE(dev, NAND_AL, cmd);
        NAND_SELECT_CLEAR(dev);
    }
}

/**
 * returns 0 if the nand is busy, 1 if it is ready
 */

static int nand_p5p_dev_ready(struct mtd_info *mtd)
{
    return readl(PARROT5_VA_SYS+_P5P_SYS_NAND) & 0x1;
}

/**
 * in chip = -1 close all chips, else enable chip number chip
 */
static void nand_p5p_select_chip(struct mtd_info *mtd, int chip)
{
    struct nand_chip *this = mtd->priv;
    struct nand_p5p_device *dev = this->priv;

    /* we support only one chip */
    if (chip == 0) {
        /* enable chip select and clear other signals */
        NAND_SELECT_CLEAR(dev);
    }
    else if (chip == -1) {
        /* Deselect flash device */
        NAND_IF_WRITE(dev, NAND_nCS|NAND_NORW, 0);
    }
}

/* use arm assembly string function for a bit more speed */
static void nand_p5p_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
    struct nand_chip *this = mtd->priv;
    writesb(this->IO_ADDR_W, buf, len);
}

static void nand_p5p_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
    struct nand_chip *this = mtd->priv;
    readsb(this->IO_ADDR_R, (u8 *)buf, len);
}

static void nand_p5p_write_buf16(struct mtd_info *mtd, const u_char *buf, int len)
{
    struct nand_chip *this = mtd->priv;

    len >>= 1;
    writesw(this->IO_ADDR_W, (u16 *) buf, len);
}

static void nand_p5p_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
    struct nand_chip *this = mtd->priv;

    len >>= 1;
    readsw(this->IO_ADDR_R, (u16 *) buf, len);
}

static void nand_p5p_hw_init(struct nand_p5p_device *info)
{
    u32 reg;

    // configure MPMC : default to 8 bits, no page mode
    // enable byte lanes, otherwise nMPMCWEOUT/nMPMCOEOUT won't move ???!
    reg = _StaticConfig_8bit|_StaticConfig_ByteLane;
    writel(reg, PARROT5_VA_MPMC+_MPMC_StaticConfig0);

    // remove mirror for CS0, CS1 & CS4
    writel(1, PARROT5_VA_MPMC+_MPMC_Control);

    /*
     * Note on NAND timings settings w.r.t MPMC delay settings
     *
     * Write operation
     *            __                                    __
     * nE (MPMC)    \__________________________________/
     *            __:____                        _________
     * nW (MPMC)    :    \______________________/
     *            __:____:              :_______:_________
     * nW (NAND)    :    \_____________/:       :
     *              :    :  ____________:_______:
     * D0-D7      --:----:-<____________:_______>----------
     *              :    :              :       :
     *              <----><------------->       :
     *            WaitWen+1   WRDELAY           :
     *              <--------------------------->
     *                         WaitWr+2
     *
     * Read operation
     *            __                                    __
     * nE (MPMC)    \__________________________________/
     *            __:____               _____
     * nR (MPMC)    :    \_____________/     \____________
     *            __:____:             :___________
     * nR (NAND)    :    \_____________/           \______
     *              :    :             :           :
     *              :    :             :           :
     *              <---->             <----------->
     *            WaitOen:             :  RDDELAY
     *              <------------------>
     *                    WaitRd+1
     */

    // program minimal delay between nE and nR or nW
    // WaitWen+1 = 1 cycle = 9.6 ns
    writel(0, PARROT5_VA_MPMC+_MPMC_StaticWaitWen0);
    // WaitOen = 0 cycle = 0 ns
    writel(0, PARROT5_VA_MPMC+_MPMC_StaticWaitOen0);
    // WaitWr+2 = 7 cycles = 67.3 ns
    writel(5, PARROT5_VA_MPMC+_MPMC_StaticWaitWr0);
    // WaitRd+1 = 4 cycles = 38.46 ns
    writel(3, PARROT5_VA_MPMC+_MPMC_StaticWaitRd0);
    // First access delay (not relevant here)
    writel(8, PARROT5_VA_MPMC+_MPMC_StaticWaitPage0);
    // WaitTurn = 0 ns
    writel(0, PARROT5_VA_MPMC+_MPMC_StaticWaitTurn0);
    // NAND WRDELAY = 4 cycles = 38.46 ns, RDDELAY = 2 cycles = 19.23 ns
    writel(0x2|(0x0 << 2), PARROT5_VA_MPMC+_MPMC_StaticWaitNand0);

    // mux NAND signals on A15-A19
    reg = readl(PARROT5_VA_SYS+_P5_SYS_IOCR4);
    writel(reg|P5P_SYS_IOCR4_NAND, PARROT5_VA_SYS+_P5_SYS_IOCR4);
}

//! Update hardware configuration after device geometry has been queried
static void nand_p5p_hw_init_final(struct nand_p5p_device *dev) 
{
    struct nand_chip *chip = &dev->chip;

    if (chip->options & NAND_BUSWIDTH_16) {
        u32 reg;

        // switch MPMC static configuration to 16-bit
        reg = readl(PARROT5_VA_MPMC+_MPMC_StaticConfig0);
        reg = (reg & ~0x3)|_StaticConfig_16bit;
        writel(reg, PARROT5_VA_MPMC+_MPMC_StaticConfig0);

        chip->write_buf = nand_p5p_write_buf16;
        chip->read_buf = nand_p5p_read_buf16;

        chip->IO_ADDR_R = chip->IO_ADDR_W =
            dev->regs + (NAND_SELECT << 1);

    }
    else {
        chip->write_buf = nand_p5p_write_buf;
        chip->read_buf = nand_p5p_read_buf;
    }

    /* large page */
    if (dev->mtd.writesize > 512) {
        // use more aggressive timings for large page devices
        // WaitWr+2 = 5 cycles = 48.08 ns
        writel(3, PARROT5_VA_MPMC+_MPMC_StaticWaitWr0);
        // WaitRd+1 = 3 cycles = 28.9 ns
        writel(2, PARROT5_VA_MPMC+_MPMC_StaticWaitRd0);
        // NAND WRDELAY = 3 cycles = 28.9 ns, RDDELAY = 2 cycles = 19.23 ns
        writel(0x3|(0x0 << 2), PARROT5_VA_MPMC+_MPMC_StaticWaitNand0);
    }
}

static int nand_p5p_probe(struct platform_device *pdev)
{
    struct nand_p5p_device *nand_p5p_mtd;
    struct nand_chip *chip;
    struct resource *res;
    int err = 0;
#ifdef CONFIG_MTD_CMDLINE_PARTS
    struct mtd_partition *mtd_parts;
    int mtd_parts_nb = 0;
#endif

    nand_p5p_mtd = kzalloc(sizeof(struct nand_p5p_device), GFP_KERNEL);
    if (!nand_p5p_mtd) {
        err = -ENOMEM;
        goto no_mem;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL) {
        dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
        err = -ENOENT;
        goto err_nores;
    }

    nand_p5p_mtd->area = request_mem_region(res->start, (res->end - res->start)+1, P5P_NAME);
    if (nand_p5p_mtd->area == NULL) {
        err = -ENOENT;
        goto no_mem_region;
    }

    nand_p5p_mtd->regs = ioremap(res->start, (res->end - res->start)+1);

    if (nand_p5p_mtd->regs == NULL) {
        err = -EIO;
        goto no_ioremap;
    }

    chip = &nand_p5p_mtd->chip;

    nand_p5p_mtd->mtd.owner = THIS_MODULE;
    nand_p5p_mtd->mtd.name = P5P_NAME;
    nand_p5p_mtd->mtd.priv = &nand_p5p_mtd->chip;
    chip->priv = nand_p5p_mtd;

    /* default to 8 bits mode first
     * 16 bit mode will be set in nand_p5p_hw_init_final */
    chip->IO_ADDR_R = chip->IO_ADDR_W =
        nand_p5p_mtd->regs + NAND_SELECT;
    chip->cmd_ctrl = nand_p5p_hwcontrol;
    chip->dev_ready = nand_p5p_dev_ready;
    chip->select_chip = nand_p5p_select_chip;
    /*
    nand_p5p_mtd->chip->options = 0;
    nand_p5p_mtd->chip->controller =;
    chip->chip_delay = 50;
    */

    nand_p5p_hw_init(nand_p5p_mtd);
    /* first part of the scan to get chip information */
    if (nand_scan_ident(&nand_p5p_mtd->mtd, 1)) {
        err = -ENXIO;
        goto no_flash;
    }
    nand_p5p_hw_init_final(nand_p5p_mtd);
#ifdef CONFIG_MTD_NAND_PARROT_ECC
    nand_parrot_ecc_init_2nd_stage(&nand_p5p_mtd->mtd, chip);
#else
    chip->ecc.mode = NAND_ECC_SOFT;
#endif

    err = nand_scan_tail(&nand_p5p_mtd->mtd);
    if (err)
        goto no_flash;

#ifdef CONFIG_MTD_NAND_PARROT_ECC
    nand_parrot_ecc_init_3nd_stage(&nand_p5p_mtd->mtd, chip);
#endif
#ifdef CONFIG_MTD_CMDLINE_PARTS
    mtd_parts_nb = parse_mtd_partitions(&nand_p5p_mtd->mtd, part_probes,
            &mtd_parts, 0);
    /* TODO
    * add static partition */
    if (mtd_parts_nb > 0) {
        err = add_mtd_partitions(&nand_p5p_mtd->mtd, mtd_parts, mtd_parts_nb);
        if (err)
            goto no_flash;
    }
    else
#endif
    {
        printk(KERN_INFO "%s no partition\n", P5P_NAME);
        add_mtd_device(&nand_p5p_mtd->mtd);
    }
    platform_set_drvdata(pdev, nand_p5p_mtd);

    goto exit;

no_flash:
    iounmap(nand_p5p_mtd->regs);
no_ioremap:
    release_resource(nand_p5p_mtd->area);
err_nores:
no_mem_region:
    kfree(nand_p5p_mtd);
no_mem:
exit:
    return err;
}

static int nand_p5p_remove(struct platform_device *dev)
{
    struct nand_p5p_device *nand_p5p_mtd = platform_get_drvdata(dev);
    platform_set_drvdata(dev, NULL);
    /* Release resources, unregister device */
    nand_release(&nand_p5p_mtd->mtd);
    iounmap(nand_p5p_mtd->regs);
    release_resource(nand_p5p_mtd->area);
    kfree(nand_p5p_mtd);
    return 0;
}

static struct platform_driver p5p_nand = {
    .probe = nand_p5p_probe,
    .remove = nand_p5p_remove,
    .driver     = {
        .name   = "p5p-nand",
        .owner  = THIS_MODULE,
    },
};

static int __init nand_p5p_init(void)
{
    return platform_driver_register(&p5p_nand);
}

module_init(nand_p5p_init);

static void __exit nand_p5p_exit(void)
{
    platform_driver_unregister(&p5p_nand);
}

module_exit(nand_p5p_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Parrot SA by Matthieu CASTET");
MODULE_DESCRIPTION("P5P MTD NAND driver");
