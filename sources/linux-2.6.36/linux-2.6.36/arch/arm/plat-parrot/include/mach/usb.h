/**
 *  linux/arch/arm/plat-parrot/include/mach/usb.h
 *
 * Copyright (C) 2009 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @date       2009-06-09
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

#ifndef __USB_PARROT6_H
#define __USB_PARROT6_H

typedef struct dwc_otg_info {
	int32_t ctrl_mode;
	int32_t speed;
	int32_t sof_filter;
	int32_t reset_pin;
	uint32_t vbus_detection;
	uint32_t fiq_enable;
	uint32_t overcurrent_disable;
} dwc_otg_info_t;

#endif
