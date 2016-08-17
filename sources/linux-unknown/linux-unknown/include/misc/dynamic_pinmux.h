/*
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author     aurelien.lefebvre@parrot.com
 * @date       2015-04-20
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

#ifndef _DYNAMIC_PINMUX_H
#define _DYNAMIC_PINMUX_H 1

struct dynamic_pinmux_platform_data {
	const char *name;
	int *mux_mode_0;
	int *mux_mode_1;
	int current_mode;
};

#endif /* _DYNAMIC_PINMUX_H */
