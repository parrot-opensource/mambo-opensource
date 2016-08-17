/**
 * @file linux/include/asm-arm/arch-parrot/i2c.h
 *
 * @brief Parrot I2C registers
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @date       2008-08-28
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
 */

#ifndef __ASM_ARCH_PARROT_REGS_I2C_P5_H
#define __ASM_ARCH_PARROT_REGS_I2C_P5_H

/* Parrot I2C registers */

#define I2CM_TRANSMIT       0x00      /* I2C Master Transmit register */
#define I2CM_RECEIVE        0x00      /* I2C Master Receive register */
#define I2CM_ITEN           0x04      /* I2C Master Interrupt Enable register */
#define I2CM_COMMAND        0x08      /* I2C Master Command register */
#define I2CM_STATUS         0x08      /* I2C Master Status register */
#define I2CM_PRESCALE       0x0C      /* I2C Master Prescale register */

/* Registers bitwise definitions */

/* I2CM Status Register */
#define I2CM_STATUS_IF      (1 << 0)  /* Interrupt Flag bit */
#define I2CM_STATUS_TIP     (1 << 1)  /* Transfer In Progress bit */
#define I2CM_STATUS_AL      (1 << 2)  /* Arbitration Lost bit */
#define I2CM_STATUS_BUSY    (1 << 3)  /* I2C Busy bit */
#define I2CM_STATUS_RXACK   (1 << 4)  /* Receive Acknowledge from slave */

/* I2CM Command Register */
#define I2CM_COMMAND_ITACK  (1 << 0)  /* Interrupt Acknowledge bit */
#define I2CM_COMMAND_ACK    (0 << 1)  /* ACK bit command */
#define I2CM_COMMAND_NACK   (1 << 1)  /* NACK bit command */
#define I2CM_COMMAND_WR     (1 << 2)  /* Write bit command */
#define I2CM_COMMAND_RD     (1 << 3)  /* Read bit command */
#define I2CM_COMMAND_STO    (1 << 4)  /* Stop bit command */
#define I2CM_COMMAND_STA    (1 << 5)  /* Start bit command */

/* I2CM Interrupt Enable register */
#define I2CM_ITEN_ITEN      (1 << 0)  /* Interrupt enabled */
#define I2CM_ITEN_ITDIS     (0 << 0)  /* Interrupt disabled */

/* I2CM Transmit register */
#define I2CM_TRANSMIT_WRITE (0 << 0)  /* RW bit : writing to slave */
#define I2CM_TRANSMIT_READ  (1 << 0)  /* RW bit : reading from slave */

#endif /* __ASM_ARCH_PARROT_I2C_H */
