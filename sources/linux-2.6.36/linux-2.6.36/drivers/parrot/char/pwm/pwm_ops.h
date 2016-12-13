/**
********************************************************************************
* @file pwm_ops.h
* @brief pwm subdriver operation
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2007-06-15
********************************************************************************
*/

#ifndef _PWM_OPS_H
#define _PWM_OPS_H 1

/**
 * sub driver callback
 */
struct pwm_ops {
    unsigned int pwm_max; /*!< max number of pwm */
    int (*pwm_start) (unsigned int pwm);
    int (*pwm_stop) (unsigned int pwm);
    int (*pwm_request) (unsigned int pwm);
    int (*pwm_release) (unsigned int pwm);
    int (*pwm_set_width) (unsigned int pwm, unsigned int width);
    int (*pwm_set_freq) (unsigned int pwm, unsigned int freq);
    int (*pwm_get_width) (unsigned int pwm, unsigned int *width);
    int (*pwm_get_freq) (unsigned int pwm, unsigned int *freq);
    struct module *owner; /*!< sub module owner (THIS_MODULE) used for 
                            module refcounting */
	int (*pwm_ioctl) (unsigned int pwm, unsigned int cmd, unsigned long arg);
};

int register_pwm(struct pwm_ops* pwm_ops);
int unregister_pwm(struct pwm_ops* pwm_ops);

#endif
