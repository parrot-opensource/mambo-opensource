/**
********************************************************************************
* @file pwm_ioctl.h
* @brief pwm ioctl
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2007-06-15
********************************************************************************
*/

/**
 * \mainpage
 *
 * This is a framwork for pwm driver.
 * ATM there is only userspace API
 *
 *
 * With this API userspace application can configure, and start/stop pwm.
 *
 * The configuration and control is done via ioctl on a file descriptor.
 *
 * See pwm_ioctl.h for the userspace API documentation.
 *
 * See pwm_ops.h for pwm sub driver API (only usefull for people
 * wanting to add unsuported controller).
 *
 * There is a example valid_pwm.c that can be found in the
 * example section.
 * \example valid_pwm.c
 */

#ifndef _PWM_IOCTL_H
#define _PWM_IOCTL_H 1

#include <asm/ioctl.h>

#define PWM_MAGIC 'p'
/**
 * get the number of pwm
 * @param number of pwm
 */
#define PWM_MAX _IOR(PWM_MAGIC, 0, unsigned int)
/**
 * reserve a pwm
 * @param pwm id
 *
 * @return an error if the pwm is already reserved
 *
 * on device close the pwm requested are released.
 * This mean you need to keep open the pwm device while
 * you want to use it.
 * @see ioctl_pwm_request
 */
#define PWM_REQUEST _IOW(PWM_MAGIC, 1, unsigned int)
/**
 * release a pwm
 * @param pwm id
 * @see ioctl_pwm_release
 */
#define PWM_RELEASE _IOW(PWM_MAGIC, 2, unsigned int)
/**
 * start a pwm
 *
 * no param
 * @see ioctl_pwm_start
 */
#define PWM_START _IO(PWM_MAGIC, 3)
/**
 * stop a pwm
 *
 * no param 
 * @see ioctl_pwm_stop
 */
#define PWM_STOP _IO(PWM_MAGIC, 4)

/**
 * configure pwm freq
 *
 * @param freq
 * @see ioctl_pwm_set_freq
 */
#define PWM_SET_FREQ _IOW(PWM_MAGIC, 5, unsigned int)
/**
 * configure pwm freq
 *
 * @param freq
 * @see ioctl_pwm_get_freq
 */
#define PWM_GET_FREQ _IOR(PWM_MAGIC, 6, unsigned int)

#define PWM_WIDTH_MAX 10000
/**
 * configure pwm width (duty cycle)
 *
 * @param width (0-PWM_WIDTH_MAX)
 * @see ioctl_pwm_set_width
 */
#define PWM_SET_WIDTH _IOW(PWM_MAGIC, 7, unsigned int)
/**
 * configure pwm width (duty cycle)
 *
 * @param width (0-PWM_WIDTH_MAX)
 * @see ioctl_pwm_get_width
 */
#define PWM_GET_WIDTH _IOR(PWM_MAGIC, 8, unsigned int)


/* delos private stuff */
typedef struct { unsigned int val[4]; } __attribute__ ((packed)) pwm_delos_quadruplet;
#define PWM_DELOS_SET_RATIOS _IOR(PWM_MAGIC, 9,  pwm_delos_quadruplet*)
#define PWM_DELOS_SET_SPEEDS _IOR(PWM_MAGIC, 10, pwm_delos_quadruplet*)
#define PWM_DELOS_SET_CTRL   _IOR(PWM_MAGIC, 11, unsigned int)
#define PWM_DELOS_REQUEST    _IO(PWM_MAGIC, 12)


// 8BITS_RATIO_MODE :
#define PWM_WIDTH_8BITS_RATIO_MODE_MAX 256
/* configure pwm freq in order to keep 8bits precision on WIDTH */
#define PWM_SET_FREQ_8BITS_RATIO_MODE _IOW(PWM_MAGIC, 13, unsigned int)
#define PWM_SET_WIDTH_8BITS_RATIO_MODE _IOW(PWM_MAGIC, 14, unsigned int)
#define PWM_GET_WIDTH_8BITS_RATIO_MODE _IOR(PWM_MAGIC, 15, unsigned int)


#endif
