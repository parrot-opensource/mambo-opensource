/**
*******************************************************************************
* @file  nand_ecc.c
* @brief NAND flash driver software ECC support.
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     ivan.djelic@parrot.fr
* @date       2007-03-21
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>


#define NAND_ERR_OK 0
#define NAND_ERR_ECC_OK 1
#define NAND_ERR_ECC_BAD -1

#define INVERT_ECC 1
#define DBG1(...) 
u32 ecc_512_compute_armv4t_step1(const u8 *data, const u8 *table);

/** ECC byte parity table.
 *
 * This table provides the following parity bits:
 *
 * bit0  bit1  bit2  bit3  bit4  bit5  bit6  bit7
 *  P1    P1'   P2    P2'   P4    P4'   P8    0
 *
 */
static const u8 ecc_byte_codes[256] = {
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a,
    0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f,
    0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c,
    0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59,
    0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33,
    0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56,
    0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55,
    0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30,
    0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30,
    0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55,
    0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56,
    0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33,
    0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59,
    0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c,
    0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f,
    0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a,
    0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
};

/** Count bits set to 1 in byte.
 *
 * @param byte Byte
 * @return Number of '1's in byte
 */
static int
count_bits( const u8 byte ) {
    static const u8 cbits[16] = { 0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4 };
    return cbits[byte & 0xf] + cbits[byte >> 4];
}

/** Compute ECC on a 512 bytes buffer.
 *
 * @param data Buffer address
 * @return A 24-bit ECC code
 */
static u32
ecc_512_compute(const u8 *data) __attribute__((unused));
static u32
ecc_512_compute(const u8 *data) {
    int i, parity, b1;
    u32 bcode, code = 0, ecc1 = 0, ecc2 = 0;

    for ( i = 0; i < 512; i++ ) {
        bcode = ecc_byte_codes[data[i]];
        ecc1 ^= bcode;
        code ^= ( bcode & 0x40 )? i : 0;
    }
    parity = ( ecc1 & 0x40 ) >> 6;

    // expand Px' bits to Px'Px
    for ( i = 0; i < 9; i++ ) {
        b1 = ( code >> i ) & 1;
        ecc2 |= ((b1 << 1)|(b1^parity)) << (i << 1);
    }

    return (ecc2 << 6)|(ecc1 & 0x3f);
}

static u32
ecc_512_compute_armv4t(const u8 *data) {
    int i, parity, b1;
    u32 code, ecc1, ecc2 = 0;

    ecc1 = ecc_512_compute_armv4t_step1(data, ecc_byte_codes);
    code = ecc1 >> 8;
    parity = ( ecc1 & 0x40 ) >> 6;

    // expand Px' bits to Px'Px
    for ( i = 0; i < 9; i++ ) {
        b1 = ( code >> i ) & 1;
        ecc2 |= ((b1 << 1)|(b1^parity)) << (i << 1);
    }

    return (ecc2 << 6)|(ecc1 & 0x3f);
}


/** Perform ECC correction on a 512 bytes buffer.
 *
 * @param data      Buffer address
 * @param ecc_flash Original ECC code
 * @param ecc_new   Newly computed code
 * @return NAND_ERR_OK if no error was detected,
 * NAND_ERR_ECC_BAD if an uncorrectable error was detected,
 * NAND_ERR_ECC_OK if a bit-error was detected and corrected
 */
static int
ecc_512_correct( u8 *data, u32 ecc_flash, u32 ecc_new ) {

    int i, nbits, parity, index = 0;
    u32 d;
    int ret = NAND_ERR_OK;

    //CYG_ASSERT(data, "Bad data pointer !");

    d = ecc_flash^ecc_new;
    nbits =
        count_bits((d) & 0xff) +
        count_bits((d >> 8) & 0xff) +
        count_bits((d >> 16) & 0xff);

    switch ( nbits ) {

    case 0:
        // no error detected
        break;

    case 1:
        // 1-bit error in ECC code, no need to correct data
        ret = NAND_ERR_ECC_OK;
        break;

    case 12:
        // maybe a correctable 1-bit error in data
        for ( i = 0; i < 12; i++, d >>= 2 ) {
            parity = d & 3;
            if (( parity == 0 )||( parity == 3 )) {
                // incorrectable error
                ret = NAND_ERR_ECC_BAD;
                break;
            }
            index |= ( parity == 1 )? 0 : (1 << i);
        }
        if ( ret == NAND_ERR_OK ) {
            // correct invalid bit
            data[index >> 3] ^= (1 << (index & 0x7));
            ret = NAND_ERR_ECC_OK;
        }
        break;

    default:
        // all other values are wrong
        ret = NAND_ERR_ECC_BAD;
        break;
    }

    if ( nbits ) {
        DBG1( "ecc %s: nbits=%d, flash=0x%06x, ram=0x%06x\n",
              ( ret == NAND_ERR_ECC_OK )? "fix" : "fatal error",
              nbits, ecc_flash, ecc_new );
    }
    return ret;
}

/**
 * nand_calculate_ecc - [NAND Interface] Calculate 3-byte ECC for 256-byte block
 * @mtd:    MTD block structure
 * @dat:    raw data
 * @ecc_code:   buffer for ECC
 */
static int nand_parrot_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
        u_char *ecc_code)
{
    u32 ecc_new;

#ifdef __arm__
    if ((((long)dat) & 0x3) == 0)
        // assembly version on 4 byte aligned buffer
        ecc_new = ecc_512_compute_armv4t(dat);
    else
#else
#warning no ecc assembly version
#endif
        // safe unaligned version
        ecc_new = ecc_512_compute(dat);

#ifdef INVERT_ECC
    /* we need to inverse ecc here to prevent's that reading from an erased 
     * FLASH results in ECC errors.
     */
    ecc_new = ~ecc_new;
#endif

    ecc_code[0] = (ecc_new >> 0) & 0xff;
    ecc_code[1] = (ecc_new >> 8) & 0xff;
    ecc_code[2] = (ecc_new >> 16) & 0xff;
    return 0;
}

/**
 * nand_correct_data - [NAND Interface] Detect and correct bit error(s)
 * @mtd:    MTD block structure
 * @dat:    raw data read from the chip
 * @read_ecc:   ECC from the chip
 * @calc_ecc:   the ECC calculated from raw data
 *
 * Detect and correct a 1 bit error for 256 byte block
 */
static int nand_parrot_correct_data(struct mtd_info *mtd, u_char *dat,
              u_char *read_ecc, u_char *calc_ecc)
{
    u32 ecc_flash, ecc_new;
    ecc_flash = read_ecc[0] | (read_ecc[1] << 8) | (read_ecc[2] << 16);
    ecc_new = calc_ecc[0] | (calc_ecc[1] << 8) | (calc_ecc[2] << 16);

    /* no need to inverse ecc here because 
     * (~ecc_flash) ^ (~ecc_new) == ecc_flash ^ ecc_new
     */
    return ecc_512_correct(dat, ecc_flash, ecc_new);
}

static struct nand_ecclayout nand_parrot_oob_16 = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 3, .length = 2},
		/* 5 is for bad block marker */
		{.offset = 6, .length = 10}
	}
};

static struct nand_ecclayout nand_parrot_oob_64 = {
	.eccbytes = 24/2,
	.eccpos = {52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {
		/* 0,1 is for bad block marker */
		{.offset = 2, .length = 38 + 12}
	}
};

/* ugly hack to use software mode read/write page ecc routine
 * doesn't seem to improve things, so not used for the moment
 */
//#define ECC_SOFT_HACK

#ifndef ECC_SOFT_HACK
static void nand_p5p_enable_hwecc(struct mtd_info *mtd, int mode)
{
}
#endif

void nand_parrot_ecc_init_2nd_stage(struct mtd_info *mtd,
        struct nand_chip *chip)
{
    switch (mtd->oobsize) {
		case 16:
			chip->ecc.layout = &nand_parrot_oob_16;
			break;
		case 64:
			chip->ecc.layout = &nand_parrot_oob_64;
			break;
		default:
			printk(KERN_WARNING "No oob scheme defined for "
			       "oobsize %d\n", mtd->oobsize);
			BUG();
	}
#ifndef ECC_SOFT_HACK
    chip->ecc.mode = NAND_ECC_HW;

    chip->ecc.size = 512;
    chip->ecc.bytes = 3;

    chip->ecc.hwctl = nand_p5p_enable_hwecc;
    chip->ecc.calculate = nand_parrot_calculate_ecc;
    chip->ecc.correct = nand_parrot_correct_data;
#else 
    chip->ecc.mode = NAND_ECC_SOFT;
#endif
}
EXPORT_SYMBOL(nand_parrot_ecc_init_2nd_stage);

void nand_parrot_ecc_init_3nd_stage(struct mtd_info *mtd,
        struct nand_chip *chip)
{
#ifdef ECC_SOFT_HACK

    chip->ecc.calculate = nand_parrot_calculate_ecc;
    chip->ecc.correct = nand_parrot_correct_data;
    chip->ecc.size = 512;
    chip->ecc.bytes = 3;

    /* XXX very ugly hack because of lack of configuration in software mode */
	/*
	 * Set the number of read / write steps for one page depending on ECC
	 * mode
	 */
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	if(chip->ecc.steps * chip->ecc.size != mtd->writesize) {
		printk(KERN_WARNING "Invalid ecc parameters\n");
		BUG();
	}
	chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;

	/*
	 * Allow subpage writes up to ecc.steps. Not possible for MLC
	 * FLASH.
	 */
	if (!(chip->options & NAND_NO_SUBPAGE_WRITE) &&
	    !(chip->cellinfo & NAND_CI_CELLTYPE_MSK)) {
		switch(chip->ecc.steps) {
		case 2:
			mtd->subpage_sft = 1;
			break;
		case 4:
		case 8:
			mtd->subpage_sft = 2;
			break;
		}
	}
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;
#endif
}

EXPORT_SYMBOL_GPL(nand_parrot_ecc_init_3nd_stage);
