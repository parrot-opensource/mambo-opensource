#ifndef DELOS_HWREV_H
#define DELOS_HWREV_H

typedef enum _delos_hw_version_t
{
	DELOS_CLASSIC_HW_00 = 0,
	DELOS_CLASSIC_HW_01,
	DELOS_CLASSIC_HW_02,
	DELOS_CLASSIC_HW_03,
	DELOS_CLASSIC_HW_04,
	DELOS_CLASSIC_HW_05,
	DELOS_CLASSIC_HW_06,
	DELOS_CLASSIC_HW_07,

	DELOS_EVO_HW_00,
	DELOS_EVO_HW_01,

	DELOS_V3_HW_00,		/* Delos3 HW00 and WingX HW00 */
	DELOS_V3_HW_01,		/* Delos3 HW01 and WingX HW01 */
	DELOS_V3_HW_02,		/* Delos3 HW02 */

	DELOS_WINGX_HW_00,	/* This HW does not exists */
	DELOS_WINGX_HW_01,	/* This HW does not exists */
	DELOS_WINGX_HW_02,	/* First wingx HW with HW id WingX! */

	DELOS_HW_MAX	/* XXX MUST NOT BE >32,
			   XXX This enum is used to make a mask! */
} delos_hw_version_t;

#endif
