/**
********************************************************************************
* @file p264_p6_ioctl.h
* @brief p264 ioctl
*
* Copyright (C) 2011 Parrot S.A.
*
* @author     Pierre ELINE <pierre.eline@parrot.com>
* @date       2011-03-01
********************************************************************************
*/


#ifndef _P264_IOCTL_H
#define _P264_IOCTL_H 1

#include <asm/ioctl.h>

typedef struct p264_p6_input_buf_t_
{
   uint32_t phys_Y;
   uint32_t phys_Cb;
   uint32_t phys_Cr;
} p264_p6_input_buf_t;

typedef struct p264_p6_raw_reg_results_t_
{
   uint32_t me_result;
   uint32_t intra_pred_4x4_0;
   uint32_t intra_pred_4x4_1;
   uint32_t pred_result[8];
} p264_p6_raw_reg_results_t;

typedef struct p264_p6_ouput_buf_t_
{
   uint32_t phys_output;
   p264_p6_raw_reg_results_t *reg_output;
} p264_p6_output_buf_t;

#define P264_RES(width,height) ((width)|((height)<<16))

#define P264_MAGIC 'p'


// Setup
#define P264_SET_DIM         _IOW(P264_MAGIC, 0, unsigned int)

#define P264_SET_INPUT_BUF   _IOW(P264_MAGIC, 1, unsigned int)

#define P264_SET_OUTPUT_BUF   _IOW(P264_MAGIC, 2, unsigned int)

#define P264_SET_FRAME_TYPE  _IOW(P264_MAGIC, 3, unsigned int)

#define P264_SET_REF_FRAME   _IOW(P264_MAGIC, 4, unsigned int)

#define P264_SET_DEB_FRAME   _IOW(P264_MAGIC, 5, unsigned int)

// Encode
#define P264_ENCODE_NEXT_MB  _IOW(P264_MAGIC, 6, unsigned int)

#define P264_WAIT_ENCODE     _IOWR(P264_MAGIC, 7, unsigned int)

#define P264_SET_QP          _IOW(P264_MAGIC, 8, unsigned int)

#endif
