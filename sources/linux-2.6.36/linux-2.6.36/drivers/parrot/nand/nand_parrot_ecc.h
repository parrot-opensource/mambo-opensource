/**
*******************************************************************************
* @file  nand_parrot_ecc.h
* @brief NAND ecc private header
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     ivan.djelic@parrot.fr
* @date       2007-03-21
*******************************************************************************
*/

#ifndef NAND_PARROT_ECC_H
#define NAND_PARROT_ECC_H 1
extern void nand_parrot_ecc_init_2nd_stage(struct mtd_info *mtd,
        struct nand_chip *chip);
extern void nand_parrot_ecc_init_3nd_stage(struct mtd_info *mtd,
        struct nand_chip *chip);
#endif
