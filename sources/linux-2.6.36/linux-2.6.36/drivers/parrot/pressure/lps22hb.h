/*
* include/linux/input/lps22hb.h
*
* STMicroelectronics LPS25 Pressure / Temperature Sensor module driver
*
* Copyright (C) 2015 STMicroelectronics- HESA BU - Environmental Sensor Application
*
* Authors: Adalberto Muhuho (adalberto.muhuho@st.com)
* The structure of this driver is based on reference code previously delivered
* by Lorenzo Sarchi
*
* Version: 0.0.2
* Date   : 2016/Apr/19
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*/
/******************************************************************************
 Revision history:

 Revision 0.0.1 2015/Dec/11:
	first (BETA) release

 Revision 0.0.2 2016/Apr/19:
	Revision 0.0.2 downgrades previous License to GPLv2
******************************************************************************/
#ifndef	__LPS22_H__
#define	__LPS22_H__

#define LPS22_PRS_MIN_POLL_PERIOD_MS	13
#define	LPS22_PRS_DEV_NAME		"lps22hb"

/* input define mappings */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_GAS

/*	Output conversion factors		*/
#define	LPS22HB_SENSITIVITY_T		100	/* =	LSB/degrC	*/
#define	LPS22HB_SENSITIVITY_P		4096	/* =	LSB/mbar	*/

#define	LPS22HB_TEMPERATURE_OFFSET	0 

#ifdef __KERNEL__
struct lps22_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

#endif /* __KERNEL__ */

#endif  /* __LPS22_H__ */
