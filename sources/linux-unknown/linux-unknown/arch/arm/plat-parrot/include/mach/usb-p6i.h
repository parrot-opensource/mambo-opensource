/**
 * @file linux/include/asm-arm/arch-parrot/regs-gpio-p5p.h
 * @brief Parrot5+ GPIO registers
 *
 * Copyright (C) 2009 Parrot S.A.
 *
 * @author     olivier.bienvenu@parrot.com
 * @date       2009-04-17
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
 *
 */
#ifndef __ARCH_ARM_PARROT_USB_P6P_H
#define __ARCH_ARM_PARROT_USB_P6P_H

enum p6i_usb2_operating_modes {
	P6P_USB2_MPH_HOST,
	P6P_USB2_DR_HOST,
	P6P_USB2_DR_DEVICE,
	P6P_USB2_DR_OTG,
};

enum p6i_usb2_phy_modes {
	P6P_USB2_PHY_NONE,
	P6P_USB2_PHY_ULPI,
	P6P_USB2_PHY_UTMI,
	P6P_USB2_PHY_UTMI_WIDE,
	P6P_USB2_PHY_SERIAL,
};

struct p6i_usb2_platform_data_s {
	/* board specific information */
	enum p6i_usb2_operating_modes	operating_mode;
	enum p6i_usb2_phy_modes		phy_mode;
	unsigned int			port_enables;
	unsigned int			force_full_speed;
};

#endif /* __ARCH_ARM_PARROT_REGS_P6P_H */
