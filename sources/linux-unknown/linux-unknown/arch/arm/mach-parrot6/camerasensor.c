#include <linux/device.h>
#include <linux/platform_device.h>


#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

unsigned long ov772x_get_param(struct soc_camera_platform_info *info)
{
	struct soc_camera_device *icd = info->icd;
	unsigned long bus_params = SOCAM_PCLK_SAMPLE_FALLING | SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_MASTER | SOCAM_DATAWIDTH_8;
	if (icd->width == 640 && icd->height == 480)
		bus_params |= SOCAM_SYNC_ORDER;
	return bus_params;
}

unsigned long cresyn_get_param(struct soc_camera_platform_info *info)
{
	struct soc_camera_device *icd = info->icd;
	unsigned long bus_params = SOCAM_PCLK_SAMPLE_FALLING | SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW | SOCAM_MASTER | SOCAM_DATAWIDTH_8;
	if (icd->width == 640 && icd->height == 480)
		bus_params |= SOCAM_SYNC_ORDER;
	return bus_params;
}
