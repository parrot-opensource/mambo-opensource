/**
*******************************************************************************
* @file
* @brief NAND flash interface driver for Parrot5+-based platforms.
*
* This file provides a low level interface implementation for the generic
* NAND flash driver.
*
* Copyright (C) 2010 Parrot S.A.
*
* @author     matthieu.castet@parrot.com
* @date       2010-07-15
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/completion.h>

#include <asm/io.h>
/* register def */
#include <mach/platform.h>

/* XXX */
#include <asm/mach-types.h>
#include <mach/parrot.h>

#define BA315_CHECK 0
#define BA315_DBG 0

#define BA315_ALLOW_FF_BITFLIP 1

/* BA315 registers */

#define BA315_STATUS                         0x00
#define BA315_START                          0x04
#define BA315_TIM0                           0x08
#define BA315_TIM1                           0x0C
#define BA315_CFG                            0x10
#define BA315_CTRL0                          0x14
#define BA315_CTRL1                          0x18
#define BA315_ADDR_LO                        0x1C
#define BA315_TIMEOUT                        0x20
#define BA315_ECC_CFG                        0x24
#define BA315_IRQ_ENABLE                     0x28
#define BA315_IRQ_DISABLE                    0x2C
#define BA315_IRQ_STATUS                     0x30
#define BA315_ADDR_HI                        0x34

/* BA315_STATUS */

#define BA315_STATUS_RnB                     (1 << 0)
#define BA315_STATUS_READY_CMD               (1 << 1)
#define BA315_STATUS_DEC_ERR                 (1 << 2)
#define BA315_STATUS_DEC_FAIL                (1 << 9)
#define BA315_STATUS_WRITE_FAULT             (1 << 10)

#define BA315_STATUS_NB_ERRS(x)              (((x)>>3) & 0x3F)

/* BA315_START */

#define BA315_START_START_PULSE              (1 << 0)
#define BA315_START_RESET_PULSE              (1 << 1)

/* BA315_TIM0  */

#define BA315_TIM0_EDO                       (1 << 31)

/* BA315_TIM1 */

/* BA315_CFG */

#define BA315_CFG_MODE_8_16                  (1 << 0)
#define BA315_CFG_COL_ADDR_NUM               (1 << 1)
#define BA315_CFG_WP_ENABLE                  (1 << 2)

#define BA315_CFG_BOOT_ENABLE                (1 << 29)
#define BA315_CFG_CE_INTERCEPT               (1 << 30)

/* BA315_CTRL0 */
#define BA315_CTRL0_CMD1(cmd) ((cmd) << 0)
#define BA315_CTRL0_CMD2(cmd) (((cmd) & 0xff) << 8)
#define BA315_CTRL0_CMD3(cmd) ((cmd) << 16)

#define BA315_CTRL0_CMD1_EN                  (1 << 24)
#define BA315_CTRL0_CMD2_EN                  (1 << 25)
#define BA315_CTRL0_CMD3_EN                  (1 << 26)
#define BA315_CTRL0_RW                       (1 << 27)
#define BA315_CTRL0_DATA_EN                  (1 << 28)

#define BA315_CTRL0_ADDR_EN(s) ((s) << 29)

/* BA315_CTRL1 */

#define BA315_CTRL1_DATA_SIZE(size) (((size)&0x1fff) << 8)

#define BA315_CTRL1_ECC_ENABLE               (1 << 21)
#define BA315_CTRL1_WAIT_READY               (1 << 22)
#define BA315_CTRL1_CE_BUSY                  (1 << 23)
#define BA315_CTRL1_CE_SELECT                (3 << 24)
#define BA315_CTRL1_DMA_ENABLE               (1 << 26)

/* BA315_ADDR_LO */

#define BA315_ADDR_LO_A0(addr) (((addr)&0xff) << 0)
#define BA315_ADDR_LO_A1(addr) (((addr)&0xff) << 8)
#define BA315_ADDR_LO_A2(addr) (((addr)&0xff) << 16)
#define BA315_ADDR_LO_A3(addr) (((addr)&0xff) << 24)

/* BA315_ADDR_HI */


/* BA315_TIMEOUT */

#define fsBA315_TIMEOUT_DELAY                (0)
#define fwBA315_TIMEOUT_DELAY                (20)

/* BA315_ECC_CFG */

#define BA315_ECC_CFG_ECC_OFFSET(x)          ((x) & 0x1FFF)

#define BA315_ECC_CFG_ECC_INTER(x)           (((x) & 0xf) << 13)

#define BA315_ECC_CFG_PACKET_SIZE(x)         (((x) & 0x3ff) << 17)

#define BA315_ECC_CFG_NB_PACKETS(x)          (((x) & 0xf) << 27)

#define BA315_ECC_CFG_ECC_TYPE               (1 << 31)

/* BA315_IRQ_ENABLE */

#define BA315_IRQ_ENABLE_READY               (1 << 0)
#define BA315_IRQ_ENABLE_OP_COMPLETE         (1 << 1)
#define BA315_IRQ_ENABLE_TIMEOUT             (1 << 2)

/* BA315_IRQ_DISABLE */

#define BA315_IRQ_DISABLE_READY              (1 << 0)
#define BA315_IRQ_DISABLE_OP_COMPLETE        (1 << 1)
#define BA315_IRQ_DISABLE_TIMEOUT            (1 << 2)

/* BA315_IRQ_STATUS */

#define BA315_IRQ_STATUS_READY               (1 << 0)
#define BA315_IRQ_STATUS_OP_COMPLETE         (1 << 1)
#define BA315_IRQ_STATUS_TIMEOUT             (1 << 2)

/* priv dev */

/* make ce high during busy phase */
#define BA315_CTRL1_WAIT_FLAGS (BA315_CTRL1_WAIT_READY|BA315_CTRL1_CE_BUSY)

#define BA315_RAM_SIZE (128*1024)

#define P5P_NAME "ba315"

/* XXX */
static void ba315_onfi_set(struct mtd_info* mtd, int mode);
#ifndef NAND_CMD_PARAM
#define NAND_CMD_PARAM      0xec
#endif
#ifndef NAND_CMD_SET_FEATURE
#define NAND_CMD_SET_FEATURE 0xef
#endif

struct ba315_device {
	struct mtd_info mtd;
	struct nand_chip chip;
	/* device info */
	struct clk* clk;
	void __iomem            *regs;
	struct resource         *area;

	void __iomem            *bc_mem;
	int data_idx;
	int data_size;

	struct completion complete;
	int read_page_hack;
};

typedef struct ba315_timing
{
    u8 twp;      //Timing 0 register
    u8 twh;      //Timing 0 register
    u8 trp;      //Timing 0 register
    u8 treh;     //Timing 0 register
    u8 twsetup;  //Timing 0 register
    u8 trsetup;  //Timing 0 register
    u8 twhr;     //Timing 0 register
    u8 tceh;     //Timing 0 register
    u8 edo;

    u8 bta;      //Timing 1 register
    u8 tbusy;    //Timing 1 register
} ba315_timing;


#define ba315_readl(dev, reg) __raw_readl(dev->regs + reg)
#define ba315_writel(val, dev, reg) __raw_writel(val, dev->regs + reg)

static struct nand_ecclayout ba315_hamming_oob16 = {
    .eccbytes = 3,
    .eccpos = { 0, 1, 2 },
	.oobfree = {
		{.offset = 3, .length = 2},
		/* 5 is for bad block marker */
		{.offset = 6, .length = 10}
	}
};

static struct nand_ecclayout ba315_hamming_oob64 = {
	.eccbytes = 12,
	/* 0,1 is for bad block marker */
	.eccpos = {
		2,  3,  4, 5,  6,  7, 8,  9,  10, 11, 12, 13
	},
	.oobfree = {
		{
			.offset = 14,
			.length = 50
		}
	}
};

static struct nand_ecclayout ba315_rs_oob16 = {
    .eccbytes = 10,
    .eccpos = { 6, 7, 8, 9, 10 ,11, 12, 13, 14, 15 },
	.oobfree = {
#if BA315_ALLOW_FF_BITFLIP
		/* 4 is used for erase page detection, see ba315_rs_oob64 comment */
		{.offset = 0, .length = 4},
#else
		{.offset = 0, .length = 5},
#endif
		/* 5 is for bad block marker */
	}
};

static struct nand_ecclayout ba315_rs_oob64 = {
	.eccbytes = 40,
	.eccpos = {
		6,  7, 8,  9,  10, 11, 12, 13, 14, 15,
        22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
	},
	.oobfree = {
		/* 0,1 is for bad block marker */
#if BA315_ALLOW_FF_BITFLIP
		{ .offset = 2, .length = 3},
			/* 5 is used for erase page detection, it should be in eccpos,
			 but do weird stuff : not contigous, 4*10 != 41, ...
			 */
#else
		{ .offset = 2, .length = 4},
#endif
		{ .offset = 16, .length = 6},
		{ .offset = 32, .length = 6},
		{ .offset = 48, .length = 6},
	}
};

/* the bbt pattern should be outside ecc. ATM only support LP
   use oob 16-19; 32-35
 */
static uint8_t bbt_pattern[] = {'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	16,
	.len = 4,
	.veroffs = 32,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	16,
	.len = 4,
	.veroffs = 32,
	.maxblocks = 4,
	.pattern = mirror_pattern
};

/* timming in ns */
static const ba315_timing ba315_onfi_timing[6] =
{
    {//Mode 0
        .twp = 50,      .twh = 50,  .trp = 50,  .treh = 50, .twsetup = 100,
        .trsetup = 60,  .twhr = 60, .tceh = 0,  .bta = 200, .tbusy = 0,
        .edo = 0,
    },
    {//Mode 1
        .twp = 25,      .twh = 20,  .trp = 30,  .treh = 20, .twsetup = 55,
        .trsetup = 15,  .twhr = 65, .tceh = 0,  .bta = 100, .tbusy = 5,
        .edo = 0,
    },
    {//Mode 2
        .twp = 17,      .twh = 18,  .trp = 25,  .treh = 15, .twsetup = 65,
        .trsetup = 10,  .twhr = 70, .tceh = 0,  .bta = 100, .tbusy = 10,
        .edo = 0,
    },
    {//Mode 3
        .twp = 15,      .twh = 15,  .trp = 20,  .treh = 10, .twsetup = 70,
        .trsetup = 10,  .twhr = 50, .tceh = 0,  .bta = 100, .tbusy = 10,
        .edo = 0,
    },
    {//Mode 4
        .twp = 12,      .twh = 13,  .trp = 12,  .treh = 13, .twsetup = 45,
        .trsetup = 10,  .twhr = 50, .tceh = 0,  .bta = 100, .tbusy = 10,
        .edo = 1,
    },
    {//Mode 5
        .twp = 10,      .twh = 10,  .trp = 10,  .treh = 10,  .twsetup = 50,
        .trsetup = 10,  .twhr = 50, .tceh = 0,  .bta = 100, .tbusy = 10,
        .edo = 1,
    },
};

static int ba315_start_read_page(struct mtd_info *mtd, int column, int page, int raw, int sndcmd);

static void ba315_init_hw(struct ba315_device *controller)
{
	/*
	 * Review all BA315 registers and set them up with regard to parameters
	 * configured at nand probing time
	 */
	ba315_writel(0x7, controller, BA315_IRQ_STATUS);

	ba315_writel(0x7fffffff, controller, BA315_TIM0);
	ba315_writel(0xfff, controller, BA315_TIM1);

	ba315_writel(0, controller, BA315_CTRL0);
	ba315_writel(0, controller, BA315_CTRL1);
	ba315_writel(0, controller, BA315_ADDR_LO);
	ba315_writel(0, controller, BA315_TIMEOUT);
	ba315_writel(0 /*BA315_CFG_MODE_8_16*/ |
			(0x0 << 27 /*BA315_CFG_DEC_CLK_DIV*/) /*|
						BA315_CFG_CE_INTERCEPT*/,
			controller, BA315_CFG);

	/*
	 * ECC configuration handling
	 */
	/* no support for ECC intervals */
	ba315_writel(0, controller, BA315_ECC_CFG);
	ba315_writel(0x7, controller, BA315_IRQ_DISABLE);
	ba315_writel(0, controller, BA315_ADDR_HI);

	return;
}

static int ba315_init_clock(struct ba315_device *dev)
{
	int ret;
	struct clk* ba315_clk;

	ba315_clk = clk_get(NULL, "nand");
	if (IS_ERR(ba315_clk)) {
		ret = PTR_ERR(ba315_clk);
		goto out;
	}

	ret = clk_enable(ba315_clk);
	if (unlikely(ret))
		goto put;

	dev->clk = ba315_clk;
	return 0;

put:
	clk_put(ba315_clk);
out:
	return ret;
}

static void dump_hex(const char *buf, int len)
{
	if (1) {
		int i;
		for (i=0; i < len; i++)
			printk("%02x ", buf[i]);
		printk("\n");
	}
}

static int diff_hex(const char *msg, const char *buf1, const char *buf2, int len)
{
	int i;
	int ret = 0;
	for (i = 0; i < len; i++) {
		if (buf1[i] != buf2[i]) {
			ret = 1;
			printk("[%d] %x!=%x ", i, buf1[i], buf2[i]);
		}
	}
	if (ret)
		printk("%s\n", msg);
	return ret;
}

static int diff_hex_c(const char *msg, const char *buf1, char c, int len)
{
	int i;
	int ret = 0;
	for (i = 0; i < len; i++) {
		if (buf1[i] != c) {
			ret = 1;
			printk("[%d] %x!=%x ", i, buf1[i], c);
		}
	}
	if (ret)
		printk("%s\n", msg);
	return ret;
}

static int
count_bits(const u8 byte)
{
    static const u8 cbits[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
    return cbits[byte & 0xf] + cbits[byte >> 4];
}

static void ba315_dump_regs(struct ba315_device *dev, int all)
{
	printk("status : %08x\n", ba315_readl(dev, BA315_STATUS));
	printk("start : %08x\n", ba315_readl(dev, BA315_START));
	if (all) {
		printk("tim0 : %08x\n", ba315_readl(dev, BA315_TIM0));
		printk("tim1 : %08x\n", ba315_readl(dev, BA315_TIM1));
		printk("cfg : %08x\n", ba315_readl(dev, BA315_CFG));
		printk("ecc : %08x\n", ba315_readl(dev, BA315_ECC_CFG));
	}
	printk("ctrl0 : %08x\n", ba315_readl(dev, BA315_CTRL0));
	printk("ctrl1 : %08x\n", ba315_readl(dev, BA315_CTRL1));
	printk("addrlo : %08x\n", ba315_readl(dev, BA315_ADDR_LO));
	printk("addrhi : %08x\n", ba315_readl(dev, BA315_ADDR_HI));
	printk("irq status : %08x\n", ba315_readl(dev, BA315_IRQ_STATUS));
}

static int ba315_wait_ready(struct mtd_info *mtd, int irq)
{
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	unsigned long timeo;
	int state = chip->state;
	//u64 time = sched_clock();

	if (state == FL_ERASING)
		timeo = (HZ * 400) / 1000;
	else
		timeo = (HZ * 20) / 1000;

	BUG_ON((ba315_readl(dev, BA315_STATUS) & BA315_STATUS_READY_CMD) == 0);

	/* clear irq */
	ba315_writel(0x7, dev, BA315_IRQ_STATUS);
	if (BA315_DBG)
		ba315_dump_regs(dev, 0);
	ba315_writel(BA315_START_START_PULSE, dev, BA315_START);
	if (irq) {
		int ret;
		ba315_writel(BA315_IRQ_STATUS_OP_COMPLETE|BA315_IRQ_STATUS_TIMEOUT, dev, BA315_IRQ_ENABLE);
		ret = wait_for_completion_timeout(&dev->complete, timeo);
		ba315_writel(0, dev, BA315_IRQ_ENABLE);
	}
	else {
		timeo += jiffies;
		/* wait end of command */
		while ((ba315_readl(dev, BA315_IRQ_STATUS) & (BA315_IRQ_STATUS_OP_COMPLETE|BA315_IRQ_STATUS_TIMEOUT)) == 0) {
			if (time_is_before_jiffies(timeo)) {
				break;
			}
			cpu_relax();
		}
	}
	//printk("take %lld\n", sched_clock() - time);
	if ((ba315_readl(dev, BA315_STATUS) & BA315_STATUS_READY_CMD) == 0) {
		printk("ba315 wait timeout\n");
		ba315_writel(BA315_START_RESET_PULSE, dev, BA315_START);
	}
	return 0;
}

static void ba315_select_chip(struct mtd_info *mtd, int chip)
{
	/* TODO : should select ce ...*/
}

static void ba315_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *dev = this->priv;

	if (dev->data_idx < dev->data_size) {
		int len_cached = min(len, dev->data_size-dev->data_idx);
		memcpy(buf, dev->bc_mem + dev->data_idx, len_cached);
		dev->data_idx += len_cached;
		len -= len_cached;
		if (len == 0)
			return;
		buf += len_cached;
	}
	if (BA315_CHECK)
		memset(dev->bc_mem, 0xde, len);

	ba315_writel(BA315_CTRL0_DATA_EN,
			dev, BA315_CTRL0);
	ba315_writel(BA315_CTRL1_DATA_SIZE(len),
			dev, BA315_CTRL1);
	ba315_wait_ready(mtd, len > 10);
	memcpy(buf, dev->bc_mem, len);
	//dump_hex(buf, len);

	dev->data_idx = dev->data_size = 0;
}

static uint8_t ba315_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *dev = this->priv;
	uint8_t ret;

	if (dev->data_idx >= dev->data_size) {
		ba315_read_buf(mtd, &ret, 1);
		return ret;
	}
	//printk("read %02x\n", readb(dev->bc_mem + dev->data_idx));
	ret = readb(dev->bc_mem + dev->data_idx++);
	return ret;
}

#if 0
static u16 ba315_read_word(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *dev = this->priv;

	if (dev->data_idx >= dev->data_size) {
		printk("reading out of buffer\n");
		//XXX issue a read
		return 0xff;
	}
	//printk("read %02x\n", readb(dev->bc_mem + dev->data_idx));
	return readb(dev->bc_mem + dev->data_idx++);
}
#endif

/* we could have implemented nand_wait, but we need dev_ready anyway ... */
static int ba315_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *dev = this->priv;
	int stat = ba315_readl(dev, BA315_STATUS) & BA315_STATUS_RnB;
	if (BA315_CHECK && stat == 0) {
		dump_stack();
	}

	return stat;
}

static int ba315_setup_sp_cycle(struct mtd_info *mtd, int col, int page)
{
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	int cycle = 0;

	if (col != -1) {
		cycle += 1;
		if (chip->options & NAND_BUSWIDTH_16)
			col >>= 1;
	}

	if (page != -1) {
		cycle += (chip->chipsize > (32 << 20)) ? 3:2;
	}

	if (col != -1) {
		ba315_writel(BA315_ADDR_LO_A0(col) |
				BA315_ADDR_LO_A1(page) |
				BA315_ADDR_LO_A2(page>>8) |
				BA315_ADDR_LO_A3(page>>16),
				dev, BA315_ADDR_LO);
	}
	else {
		ba315_writel(BA315_ADDR_LO_A0(page) |
				BA315_ADDR_LO_A1(page>>8) |
				BA315_ADDR_LO_A2(page>>16),
				dev, BA315_ADDR_LO);
	}

	return cycle;
}

static int ba315_setup_lp_cycle(struct mtd_info *mtd, unsigned int cmd, int col, int page)
{
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	int cycle = 0;

	if (col != -1) {
		if (cmd != NAND_CMD_READID && cmd != NAND_CMD_PARAM)
			cycle += 2;
		else
			cycle += 1;

		if (chip->options & NAND_BUSWIDTH_16)
			col >>= 1;
	}
	if (page != -1) {
		cycle += (chip->chipsize > (128 << 20)) ? 3:2;
	}

	if (col != -1) {
		ba315_writel(BA315_ADDR_LO_A0(col) |
				BA315_ADDR_LO_A1(col>>8) |
				BA315_ADDR_LO_A2(page) |
				BA315_ADDR_LO_A3(page>>8),
				dev, BA315_ADDR_LO);
		ba315_writel(BA315_ADDR_LO_A0(page>>16),
				dev, BA315_ADDR_HI);
	}
	else {
		ba315_writel(BA315_ADDR_LO_A0(page) |
				BA315_ADDR_LO_A1(page>>8) |
				BA315_ADDR_LO_A2(page>>16),
				dev, BA315_ADDR_LO);
	}

	return cycle;
}

static void ba315_erase_cmd(struct mtd_info *mtd, int page)
{
	/* no AND erase supported ... */
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	int cycle;
	int col = -1;
	if (mtd->writesize > 512) {
		cycle = ba315_setup_lp_cycle(mtd, NAND_CMD_ERASE1, col, page);
	}
	else {
		cycle = ba315_setup_sp_cycle(mtd, col, page);
	}
	ba315_writel(BA315_CTRL0_CMD1_EN |
			BA315_CTRL0_CMD1(NAND_CMD_ERASE1) |
			BA315_CTRL0_RW |
			BA315_CTRL0_ADDR_EN(cycle) |
			BA315_CTRL0_CMD3_EN |
			BA315_CTRL0_CMD3(NAND_CMD_ERASE2),
			dev, BA315_CTRL0);

	ba315_writel(BA315_CTRL1_WAIT_FLAGS,
			dev, BA315_CTRL1);

	ba315_wait_ready(mtd, 1);

	if (BA315_CHECK) {
		int pagen;
		for (pagen = page; pagen < page+mtd->erasesize/mtd->writesize; pagen++) {
			int ret;
			ba315_start_read_page(mtd, 0, pagen, 1, 1);
			ret = diff_hex_c("erased", dev->bc_mem, 0xff, mtd->writesize + mtd->oobsize);
			if (ret) {
				printk("not erased page %d\n", pagen);
				dump_hex(dev->bc_mem, mtd->writesize + mtd->oobsize);
			}
		}
	}
}

static int ba315_start_write_page(struct mtd_info *mtd, int column, int page, int cached, int raw)
{
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	int cycle;

	if (mtd->writesize > 512) {
		cycle = ba315_setup_lp_cycle(mtd, NAND_CMD_SEQIN, column, page);
		ba315_writel(BA315_CTRL0_CMD1_EN |
				BA315_CTRL0_CMD1(NAND_CMD_SEQIN) |
				BA315_CTRL0_RW |
				BA315_CTRL0_DATA_EN |
				BA315_CTRL0_ADDR_EN(cycle) |
				BA315_CTRL0_CMD3_EN |
				BA315_CTRL0_CMD3(NAND_CMD_PAGEPROG),
				dev, BA315_CTRL0);
	}
	else {
		int readcmd;
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}

		cycle = ba315_setup_sp_cycle(mtd, column, page);
		ba315_writel(BA315_CTRL0_CMD1_EN |
				BA315_CTRL0_CMD1(readcmd) |
				BA315_CTRL0_CMD2_EN |
				BA315_CTRL0_CMD2(NAND_CMD_SEQIN) |
				BA315_CTRL0_RW |
				BA315_CTRL0_DATA_EN |
				BA315_CTRL0_ADDR_EN(cycle) |
				BA315_CTRL0_CMD3_EN |
				BA315_CTRL0_CMD3(NAND_CMD_PAGEPROG),
				dev, BA315_CTRL0);

	}
	ba315_writel(BA315_CTRL1_WAIT_FLAGS |
			(raw?0:BA315_CTRL1_ECC_ENABLE) |
			BA315_CTRL1_DATA_SIZE(mtd->writesize + mtd->oobsize - column),
			dev, BA315_CTRL1);

	return ba315_wait_ready(mtd, 1);
}

static int nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, int cached, int raw)
{
	int status;
	struct ba315_device *dev = chip->priv;

	/* allow to check 0xff page */
	if (BA315_ALLOW_FF_BITFLIP && !raw && chip->ecc.bytes == 10) {
		if (mtd->oobsize == 64)
			chip->oob_poi[5] = 0;
		else
			chip->oob_poi[4] = 0;
	}
	memcpy(dev->bc_mem, buf, mtd->writesize);
	memcpy(dev->bc_mem + mtd->writesize, chip->oob_poi, mtd->oobsize);
	ba315_start_write_page(mtd, 0, page, cached, raw);

	if (BA315_CHECK && memcmp(dev->bc_mem, buf, mtd->writesize) != 0) {
		printk("write : corrupted data ???\n");
	}
	if (BA315_CHECK && !raw) {
		int i;
		/* we don't check for bad block marker */
		for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES; i++) {
			int start = chip->ecc.layout->oobfree[i].offset;
			int len = chip->ecc.layout->oobfree[i].length;
			if (len == 0)
				break;
			if (memcmp(dev->bc_mem + mtd->writesize + start,
						chip->oob_poi + start, len) != 0) {
				printk("write : strange ecc (user @%u:%u)\n", start, len);
				dump_hex(chip->oob_poi, mtd->oobsize);
				dump_hex(dev->bc_mem + mtd->writesize, mtd->oobsize);
			}
		}
#if 0
		/* page full of 0 will generate 0xff ecc ... */
		int i;
		for (i = 2; i < 14; i++)
			if (readb(dev->bc_mem + mtd->writesize + i) != 0xff)
				break;
		if (i == 14) {
			printk("write : void ecc\n");
			dump_hex(dev->bc_mem, mtd->writesize);
			dump_hex(dev->bc_mem + mtd->writesize, mtd->oobsize);
		}
#endif
	}

	status = chip->waitfunc(mtd, chip);
	/*
	 * See if operation failed and additional status checks are
	 * available
	 */
	if ((status & NAND_STATUS_FAIL) && (chip->errstat))
		status = chip->errstat(mtd, chip, FL_WRITING, status,
				page);

	if (status & NAND_STATUS_FAIL)
		return -EIO;

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	ba315_start_read_page(mtd, 0, page, 1, 1);
	if (memcmp(dev->bc_mem, buf, mtd->writesize) != 0) {
		printk("CONFIG_MTD_NAND_VERIFY_WRITE failed on %d\n", page);
		diff_hex("verify write", dev->bc_mem, buf, mtd->writesize);
		return -EIO;
	}
#endif
	return 0;
}

/* write only oob data */
static int nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
				  int page)
{
	int status = 0;
	const uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;
	struct ba315_device *dev = chip->priv;

	memcpy(dev->bc_mem, buf, length);
	ba315_start_write_page(mtd, mtd->writesize, page, 0, 1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static int ba315_start_read_page(struct mtd_info *mtd, int column, int page, int raw, int sndcmd)
{
	struct nand_chip *chip = mtd->priv;
	struct ba315_device *dev = chip->priv;
	int cycle;

	if (mtd->writesize > 512) {
		cycle = ba315_setup_lp_cycle(mtd, NAND_CMD_READ0, column, page);
		ba315_writel(BA315_CTRL0_CMD1_EN |
				BA315_CTRL0_CMD1(NAND_CMD_READ0) |
				BA315_CTRL0_DATA_EN |
				BA315_CTRL0_ADDR_EN(cycle) |
				BA315_CTRL0_CMD3_EN |
				BA315_CTRL0_CMD3(NAND_CMD_READSTART),
				dev, BA315_CTRL0);
	}
	else {
		int readcmd;
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}

		cycle = ba315_setup_sp_cycle(mtd, column, page);
		ba315_writel(BA315_CTRL0_CMD1_EN |
				BA315_CTRL0_CMD1(readcmd) |
				BA315_CTRL0_DATA_EN |
				BA315_CTRL0_ADDR_EN(cycle),
				dev, BA315_CTRL0);

	}
	ba315_writel(BA315_CTRL1_WAIT_FLAGS |
			(raw?0:BA315_CTRL1_ECC_ENABLE) |
			BA315_CTRL1_DATA_SIZE(mtd->writesize + mtd->oobsize - column),
			dev, BA315_CTRL1);

	if (BA315_CHECK)
		memset(dev->bc_mem, 0xde, mtd->writesize + mtd->oobsize - column);
	return ba315_wait_ready(mtd, 1);
}

static int ba315_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	int status;
	struct ba315_device *dev = chip->priv;

	ba315_start_read_page(mtd, 0, page, 0, 1);
	memcpy(buf, dev->bc_mem, mtd->writesize);
	memcpy(chip->oob_poi, dev->bc_mem + mtd->writesize, mtd->oobsize);
	status = ba315_readl(dev, BA315_STATUS);
	/* BA315_STATUS_DEC_ERR indicate correction where done. It can be set
	   or not when BA315_STATUS_DEC_FAIL
	 */
	if (status & BA315_STATUS_DEC_ERR || status & BA315_STATUS_DEC_FAIL) {
		/* XXX reading erased page will produce ecc error with RS... */
		if (chip->ecc.bytes == 10 && (status & BA315_STATUS_DEC_FAIL)) {
			/* first check FF page without bit flip */
			if (status & BA315_STATUS_DEC_ERR && BA315_STATUS_NB_ERRS(status) == 4) {
				int i;
				for (i = 243; i < mtd->writesize; i+=512) {
					/* controller correct blank page ... */
					if (buf[i] != 0x76)
						break;
				}
				if (i >= mtd->writesize) {
					/* check if OOB is all 0xff */
					for (i = 0; i < mtd->oobsize; i++)
						if (chip->oob_poi[i] != 0xff)
							break;
				}
				if (i == mtd->oobsize) {
#if 0
					/* faster version 11.650 MB/s vs 11.110 MB/s but dangerous */
					for (i = 243; i < mtd->writesize; i+=chip->ecc.size) {
						buf[i] = 0xff;
					}
#else
					memset(buf, 0xff, mtd->writesize);
#endif
					goto blank;
				}
			}
#if 0
			if (status & BA315_STATUS_DEC_ERR && BA315_STATUS_NB_ERRS(status) == 4 && buf[243] == 0x76) {
				int i;
				/* check if OOB is all 0xff */
				for (i = 0; i < mtd->oobsize; i++)
					if (chip->oob_poi[i] != 0xff)
						break;

				/* check data is all 0xff */
				if (i == mtd->oobsize) {
					for (i = 0; i < mtd->writesize; i++) {
						/* controller correct blank page ... */
						if ((i & (512-1)) == 243) {
							if (buf[i] != 0x76)
								break;
						}
						else if (buf[i] != 0xff)
							break;
					}
					if (i == mtd->writesize) {
						/* fix corrected page */
						for (i = 243; i < mtd->writesize; i+=chip->ecc.size) {
							buf[i] = 0xff;
						}
						goto blank;
					}
					else {
						printk("BA315 : not 0xff at %d : %x\n", i, buf[i]);
					}
				}
			}
#endif
			/* detect bit flip */
			if (BA315_ALLOW_FF_BITFLIP) {
				int i;
				if (mtd->oobsize == 64)
					i = chip->oob_poi[5];
				else
					i = chip->oob_poi[4];

				/* set data is all 0xff */
				if (count_bits(i) >= 4) {
					/* fix corrected page */
					memset(buf, 0xff, mtd->writesize);
					memset(chip->oob_poi, 0xff, mtd->oobsize);
					mtd->ecc_stats.corrected += 1;
					printk("BA315 : blank page with bit flip on %d\n", page);
					goto blank;
				}
			}
		}

		printk("BA315_STATUS_DEC_ERR : %d %x on %d\n", status & BA315_STATUS_DEC_FAIL, BA315_STATUS_NB_ERRS(status), page);
		dump_hex(chip->oob_poi, mtd->oobsize);
		if (BA315_CHECK) {
			/* read the page with no ecc */
			ba315_start_read_page(mtd, 0, page, 1, 1);
			diff_hex("data", buf, dev->bc_mem, mtd->writesize);
			diff_hex("oob", chip->oob_poi, dev->bc_mem + mtd->writesize, mtd->oobsize);
			/* dump the whole page in case of uncorrectable error */
			if (status & BA315_STATUS_DEC_FAIL) {
				dump_hex(buf, mtd->writesize);
				//dump_hex(dev->bc_mem, mtd->writesize);
			}
		}

		if (status & BA315_STATUS_DEC_FAIL)
			mtd->ecc_stats.failed++;
		else if ((chip->ecc.bytes == 10 && BA315_STATUS_NB_ERRS(status) > 3)
				|| chip->ecc.bytes != 10) {
			/*
			 * FIXME: in order to prevent upper layers (such as UBI) from
			 * torturing and marking a block as bad as soon as 1 bitflip
			 * is persistent, we implement a threshold below which errors
			 * are corrected but not reported. Instead, mtd should provide
			 * a generic way to handle this situation.
			 */
			mtd->ecc_stats.corrected += BA315_STATUS_NB_ERRS(status);
		}
	}
blank:

	return 0;
}

static int ba315_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				  uint8_t *buf, int page)
{
	struct ba315_device *dev = chip->priv;

	ba315_start_read_page(mtd, 0, page, 1, 1);
	memcpy(buf, dev->bc_mem, mtd->writesize);
	memcpy(chip->oob_poi, dev->bc_mem + mtd->writesize, mtd->oobsize);
	return 0;
}

static int ba315_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				 int page, int sndcmd)
{
	struct ba315_device *dev = chip->priv;
	ba315_start_read_page(mtd, mtd->writesize, page, 1, sndcmd);
	memcpy(chip->oob_poi, dev->bc_mem, mtd->oobsize);
	/* force next sndcmd */
	return 1;
}

static void ba315_cmdfunc(struct mtd_info *mtd, unsigned int cmd, int col,
		int page)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *dev = this->priv;
	int cycle = 0;

	dev->data_idx = 0;
	if (cmd != NAND_CMD_READ0 && cmd != NAND_CMD_STATUS && cmd != NAND_CMD_READOOB)
		printk("using cmd : %x\n", cmd);
	switch (cmd) {
		case NAND_CMD_RESET:
			BUG_ON(col != -1 || page != -1);
			dev->data_size = 0;
			ba315_writel(BA315_CTRL0_CMD1_EN |
					BA315_CTRL0_CMD1(NAND_CMD_RESET),
					dev, BA315_CTRL0);
			ba315_writel(BA315_CTRL1_WAIT_FLAGS,
					dev, BA315_CTRL1);
			ba315_wait_ready(mtd, 0);
			break;

		case NAND_CMD_READID:
			BUG_ON(col < 0 || col > 0xff || page != -1);
			dev->data_size = 2;
			cycle = ba315_setup_sp_cycle(mtd, col, page);
			ba315_writel(BA315_CTRL0_CMD1_EN |
					BA315_CTRL0_CMD1(NAND_CMD_READID) |
					BA315_CTRL0_ADDR_EN(cycle) |
				 	BA315_CTRL0_DATA_EN,
					dev, BA315_CTRL0);
			ba315_writel(
					BA315_CTRL1_DATA_SIZE(dev->data_size),
					dev, BA315_CTRL1);
			ba315_wait_ready(mtd, 0);
			break;

		case NAND_CMD_PARAM:
			dev->data_size = 0;
			cycle = ba315_setup_sp_cycle(mtd, col, page);
			ba315_writel(BA315_CTRL0_CMD1_EN |
					BA315_CTRL0_CMD1(NAND_CMD_PARAM) |
					BA315_CTRL0_ADDR_EN(cycle),
					dev, BA315_CTRL0);
			ba315_writel(BA315_CTRL1_WAIT_FLAGS |
					BA315_CTRL1_DATA_SIZE(dev->data_size),
					dev, BA315_CTRL1);
			ba315_wait_ready(mtd, 1); /* take 23 us */
			break;

		case NAND_CMD_STATUS:
			BUG_ON(col != -1 || page != -1);
			/* status could read lot's of time while busy checking
			   but that shouldn't happen : we provide dev_ready */
			dev->data_size = 1;
			ba315_writel(BA315_CTRL0_CMD1_EN |
					BA315_CTRL0_CMD1(NAND_CMD_STATUS) |
				 	BA315_CTRL0_DATA_EN,
					dev, BA315_CTRL0);
			ba315_writel(
					BA315_CTRL1_DATA_SIZE(dev->data_size),
					dev, BA315_CTRL1);
			ba315_wait_ready(mtd, 0);
			break;
			/* can be removed when block_bad is implemented */
		case NAND_CMD_READOOB:
			/* not supported on lp, emulate it */
			col += mtd->writesize;
			cmd = NAND_CMD_READ0;
			ba315_start_read_page(mtd, col, page, 1, 1);
			dev->data_size = mtd->writesize + mtd->oobsize - col;
			break;
		case NAND_CMD_READ0:
			BUG_ON(col != 0);
			/* can be removed on new mtd version */
			dev->read_page_hack = page;
			break;
		default:
			printk("unsupported cmd %d\n", cmd);
			dump_stack();
			break;
	}
}

/* stubs for ECC functions not used by the NAND core */
static int ba315_ecc_calculate(struct mtd_info *mtd, const uint8_t *data,
				uint8_t *ecc_code)
{
	printk(KERN_ERR "ba315_ecc_calculate called unexpectedly\n");
	BUG();
	return -EIO;
}

static int ba315_ecc_correct(struct mtd_info *mtd, uint8_t *data,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	printk(KERN_ERR "ba315_ecc_correct called unexpectedly\n");
	BUG();
	return -EIO;
}

static void ba315_ecc_hwctl(struct mtd_info *mtd, int mode)
{
	printk(KERN_ERR "ba315_ecc_hwctl called unexpectedly\n");
	BUG();
}

static irqreturn_t ba315_irq(int irq, void *dev_id)
{
	struct ba315_device *dev = dev_id;
	u32 status = ba315_readl(dev, BA315_IRQ_STATUS);
	if (status == 0)
		return IRQ_NONE;
	ba315_writel(status, dev, BA315_IRQ_DISABLE);

	complete(&dev->complete);

	return IRQ_HANDLED;
}

static int ba315_probe(struct platform_device *pdev)
{
	struct ba315_device *ba315_mtd;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct resource *res;
	int err = 0;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *mtd_parts;
	int mtd_parts_nb = 0;
	const char* part_probes[] =  {
		"parrotpart",
		"cmdlinepart",
		NULL
	};
#else
#warning no partition
#endif

	ba315_mtd = kzalloc(sizeof(struct ba315_device), GFP_KERNEL);
	if (!ba315_mtd) {
		err = -ENOMEM;
		goto no_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_nores;
	}

	ba315_mtd->area = request_mem_region(res->start, (res->end - res->start)+1, P5P_NAME);
	if (ba315_mtd->area == NULL) {
		err = -ENOENT;
		goto no_mem_region;
	}

	ba315_mtd->regs = ioremap(res->start, (res->end - res->start)+1);

	if (ba315_mtd->regs == NULL) {
		err = -EIO;
		goto no_ioremap;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_noram;
	}

	ba315_mtd->bc_mem = ioremap(res->start, (res->end - res->start)+1);

	if (ba315_mtd->bc_mem == NULL) {
		err = -EIO;
		goto no_ioremap_ram;
	}

	mtd = &ba315_mtd->mtd;
	chip = &ba315_mtd->chip;

	mtd->owner = THIS_MODULE;
	mtd->name = "nand0";//P5P_NAME;
	mtd->priv = &ba315_mtd->chip;
	chip->priv = ba315_mtd;

	/* default to 8 bits mode first
	 * 16 bit mode will be set in ba315_hw_init_final */
	chip->IO_ADDR_R = chip->IO_ADDR_W = 0;
	chip->cmdfunc = ba315_cmdfunc;
	//chip->cmd_ctrl = ba315_hwcontrol;
	chip->dev_ready = ba315_dev_ready;
	chip->select_chip = ba315_select_chip;
	chip->read_byte = ba315_read_byte;
	chip->read_buf = ba315_read_buf;
	/*
	ba315_mtd->chip->options = 0;
	ba315_mtd->chip->controller =;
	chip->chip_delay = 50;
	*/

	if (ba315_init_clock(ba315_mtd)) {
		err = -EIO;
		goto no_ioremap_ram;
	}
	ba315_init_hw(ba315_mtd);

	init_completion(&ba315_mtd->complete);
	err = request_irq(platform_get_irq(pdev, 0), &ba315_irq, 0,
					  "BA315 NAND controller", ba315_mtd);
	if (err) {
		err = -ENOENT;
		goto no_irq;
	}

	/* first part of the scan to get chip information */
	if (nand_scan_ident(mtd, 1, NULL)) {
		err = -ENXIO;
		goto no_flash;
	}
	chip->options |= NAND_NO_SUBPAGE_WRITE;

	if (NAND_CANAUTOINCR(chip)) {
		printk("autoincr page not supported ATM\n");
		chip->options &= ~NAND_NO_AUTOINCR;
	}
	if (chip->options & NAND_BUSWIDTH_16) {
#if 0
		ba315_writel(BA315_CFG_MODE_8_16 | ba315_readl(controller, BA315_CFG),
				controller, BA315_CFG);
		chip->read_word = ba315_read_word;
#endif
		goto no_flash;
	}

	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.size = 512;
#ifndef REED_SOLOMON
	chip->ecc.bytes = 3;
#else
	chip->ecc.bytes = 10;
#endif
	if (chip->onfi_version) {
		int eccb = chip->onfi_params.ecc_bits;
		printk("onfi ecc bits : %d\n", eccb);
		if (eccb > 4) {
			printk("ecc bits not supported\n");
			goto no_flash;
		}
		if (eccb > 1)
			chip->ecc.bytes = 10;
	}
	/* force RS for p6i (OEM) */
	if (parrot_chip_is_p6i())
		chip->ecc.bytes = 10;

    switch (mtd->oobsize) {
		case 16:
			if (chip->ecc.bytes == 3)
				chip->ecc.layout = &ba315_hamming_oob16;
			else
				chip->ecc.layout = &ba315_rs_oob16;
			break;
		case 64:
			if (chip->ecc.bytes == 3)
				chip->ecc.layout = &ba315_hamming_oob64;
			else
				chip->ecc.layout = &ba315_rs_oob64;
			break;
		default:
			printk(KERN_WARNING "No oob scheme defined for "
			       "oobsize %d\n", mtd->oobsize);
			BUG();
	}
	ba315_writel(
			BA315_ECC_CFG_ECC_OFFSET(mtd->writesize + chip->ecc.layout->eccpos[0]) |
			0 |
			BA315_ECC_CFG_ECC_INTER(mtd->oobsize == 64 && chip->ecc.bytes != 3 ? 6:0) |
			BA315_ECC_CFG_PACKET_SIZE(chip->ecc.size) |
			BA315_ECC_CFG_NB_PACKETS(mtd->writesize/chip->ecc.size) |
			((chip->ecc.bytes == 3) ? BA315_ECC_CFG_ECC_TYPE : 0),
			ba315_mtd, BA315_ECC_CFG);

	/* if NAND_USE_FLASH_BBT, bad block marker are not set on bad page
	   (this mean parrotboot need to parse it)
	   also page reserved for bad block marker are reported as bad by mtd

	   so we cannot switch easily to/from BBT.
	   Keep it now for compatibilily

	   NOTE : on 1Go flash bbt (mode 1) save only 100ms...
	 */
#if 1
	/* XXX not supported for rs 16 bytes ecc */
	if (!(mtd->oobsize == 16 && chip->ecc.bytes == 10) && !parrot_chip_is_p6i()) {
		chip->options |= NAND_USE_FLASH_BBT;
		/* force bb marker write in bad page */
		chip->options |= NAND_FLASH_BB;
		if (mtd->oobsize == 64) {
			/* default descr use oob 8 from 13 */
			chip->bbt_td = &bbt_main_descr;
			chip->bbt_md = &bbt_mirror_descr;
		}
	}
#endif


	/* fake function */
	chip->ecc.calculate = ba315_ecc_calculate;
	chip->ecc.correct = ba315_ecc_correct;
	chip->ecc.hwctl = ba315_ecc_hwctl;

	/* override the default read operations */
	chip->ecc.read_page = ba315_read_page;
	chip->ecc.read_page_raw = ba315_read_page_raw;
	chip->ecc.read_oob = ba315_read_oob;
	chip->write_page = nand_write_page;
	chip->ecc.write_oob = nand_write_oob_std;
	chip->erase_cmd = ba315_erase_cmd;

	if (chip->onfi_speed >= 0) {
		ba315_onfi_set(mtd, chip->onfi_speed);
	}
	else
		ba315_onfi_set(mtd, 0);

	ba315_dump_regs(ba315_mtd, 1);

	err = nand_scan_tail(mtd);
	if (err)
		goto no_flash;

#ifdef CONFIG_MTD_PARTITIONS
	mtd_parts_nb = parse_mtd_partitions(mtd, part_probes,
			&mtd_parts, 0);
	/* TODO
	* add static partition */
	if (mtd_parts_nb > 0) {
		err = add_mtd_partitions(mtd, mtd_parts, mtd_parts_nb);
		if (err)
			goto no_part;
	}
	else
#endif
	{
		printk(KERN_INFO "%s no partition\n", P5P_NAME);
		add_mtd_device(mtd);
	}
	platform_set_drvdata(pdev, ba315_mtd);

	goto exit;

no_part:
	nand_release(mtd);
no_flash:
	free_irq(platform_get_irq(pdev, 0), ba315_mtd);
no_irq:
	clk_disable(ba315_mtd->clk);
	clk_put(ba315_mtd->clk);
	iounmap(ba315_mtd->bc_mem);
no_ioremap_ram:
err_noram:
	iounmap(ba315_mtd->regs);
no_ioremap:
	release_resource(ba315_mtd->area);
err_nores:
no_mem_region:
	kfree(ba315_mtd);
no_mem:
exit:
	return err;
}

static int ba315_remove(struct platform_device *pdev)
{
	struct ba315_device *ba315_mtd = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	/* Release resources, unregister device */
	nand_release(&ba315_mtd->mtd);
	free_irq(platform_get_irq(pdev, 0), ba315_mtd);
	iounmap(ba315_mtd->bc_mem);
	iounmap(ba315_mtd->regs);
	clk_disable(ba315_mtd->clk);
	clk_put(ba315_mtd->clk);
	release_resource(ba315_mtd->area);
	kfree(ba315_mtd);
	return 0;
}

static struct platform_driver ba315_nand = {
	.probe = ba315_probe,
	.remove = ba315_remove,
	.driver     = {
		.name   = "p6-nand",
		.owner  = THIS_MODULE,
	},
};

static int __init ba315_init(void)
{
	return platform_driver_register(&ba315_nand);
}

module_init(ba315_init);

static void __exit ba315_exit(void)
{
	platform_driver_unregister(&ba315_nand);
}

module_exit(ba315_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Parrot SA by Matthieu CASTET");
MODULE_DESCRIPTION("P5P MTD NAND driver");


/* onfi support ... */

static unsigned int round_clock(unsigned int tim, unsigned int hclk, unsigned int max)
{
	unsigned int ret;
	if (tim == 0)
		return 0;
	/* round up - 1
	 * (tim * 1000 + hclk - 1)/hclk - 1 = (tim * 1000 - 1)/hclk
	 */
	ret = (tim * 1000 - 1)/hclk;

	if (ret > max)
		ret = max;
	return ret;
}

static void ba315_onfi_set(struct mtd_info* mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct ba315_device *ctrl = this->priv;
	unsigned int hclk = NSEC_PER_SEC / (clk_get_rate(ctrl->clk) / 1000);
	const ba315_timing *timing = &(ba315_onfi_timing[mode]);
	u32 tim;

	printk("setting onfi mode %d\n", mode);

	tim = round_clock(timing->tceh, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->twhr, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->trsetup, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->twsetup, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->treh, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->trp, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->twh, hclk, 0xf);
	tim <<= 4;
	tim |= round_clock(timing->twp, hclk, 0x7);

	if (timing->edo)
		tim |= BA315_TIM0_EDO;

	ba315_writel(tim, ctrl, BA315_TIM0);

	tim = round_clock(timing->tbusy, hclk, 0x1f);
	tim <<= 5;
	tim |= round_clock(timing->bta, hclk, 0x3f);

	ba315_writel(tim, ctrl, BA315_TIM1);
}
