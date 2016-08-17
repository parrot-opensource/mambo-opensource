/**
 * @file p264_p6_fiq.S
 *
 * @brief FIQ handler for h264
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * @author     pierre.eline@parrot.com
 * @date       2011-03-1
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

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/map.h>
#include "P6_h264_reg.h"

        .text

#define fiq_reg_output   r8
#define pic_encode_ctx   r10
#define h264_base         r11

// some macros to access the following structures
/*
typedef struct p264_p6_raw_reg_results_t_
{
   uint32_t me_result;
   uint32_t intra_pred_4x4_0;
   uint32_t intra_pred_4x4_1;
   uint32_t pred_result[8];
} p264_p6_raw_reg_results_t;
*/

#define P264_P6_RAW_RESULT_me_result           0
#define P264_P6_RAW_RESULT_intra_pred_4x4_0   4
#define P264_P6_RAW_RESULT_intra_pred_4x4_1   8
#define P264_P6_RAW_RESULT_pred_result        12
#define P264_P6_RAW_RESULT_SIZE               44

/* picture encoding context structure
typedef struct picture_encoding_context_t_
{
  uint32_t current_width;
  uint32_t current_height;
  uint32_t current_i_MB;
  uint32_t current_j_MB;
  uint32_t nb_mb_to_encode; // number of MB to be encoded by fiq
  uint32_t nb_mb_encoded;   // number of MB to already encoded by fiq
  uint32_t nb_mb_user;      // number of MB read by user
  bool     is_running;      //
} picture_encoding_context_t;
*/

#define PICTURE_ENCODING_CTX_current_width    0
#define PICTURE_ENCODING_CTX_current_height    4
#define PICTURE_ENCODING_CTX_current_i_MB      8
#define PICTURE_ENCODING_CTX_current_j_MB     12
#define PICTURE_ENCODING_CTX_nb_mb_to_encode  16
#define PICTURE_ENCODING_CTX_nb_mb_encoded    20
#define PICTURE_ENCODING_CTX_nb_mb_user       24
#define PICTURE_ENCODING_CTX_is_running       28

#define MAX_ENCODED_MB_SIZE 864

    .global  h264_p6_fiq_handler_end
ENTRY(h264_p6_fiq_handler_start)
        ldr     h264_base,.h264_base

        /* acknowledge int */
        ldr     r9,.cst
        str      r9,[h264_base,#H264_ITACK] // H264_ITACK

        // jump to current fiq_reg_output
        ldr      r13,[pic_encode_ctx,#PICTURE_ENCODING_CTX_nb_mb_encoded]  // r13 = nb_mb_encoded
        mov     r12,#P264_P6_RAW_RESULT_SIZE
        mul      r12,r13,r12
        add      r9,r12,fiq_reg_output     // r12 = &fiq_reg_output[nb_mb_encoded]

        // save registers to fiq_output_reg
        ldr     r12,[h264_base,#ME_RESULT]                       // ME_RESULT
        str     r12,[r9,#P264_P6_RAW_RESULT_me_result]          // -> me_result
        ldr     r12,[h264_base,#ME_IPRED0]                       // ME_IPRED0
        str     r12,[r9,#P264_P6_RAW_RESULT_intra_pred_4x4_0]   // -> intra_pred_4x4_0
        ldr     r12,[h264_base,#ME_IPRED1]                       // ME_IPRED1
        str     r12,[r9,#P264_P6_RAW_RESULT_intra_pred_4x4_1]   // -> intra_pred_4x4_1
        ldr     r12,[h264_base,#ME_PRED_BASE_REG]               // ME_PRED_BASE_REG[0]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result]         // -> pred_result[0]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+4]             // ME_PRED_BASE_REG[1]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+4]      // -> pred_result[1]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+8]             // ME_PRED_BASE_REG[2]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+8]      // -> pred_result[2]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+12]            // ME_PRED_BASE_REG[3]s
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+12]     // -> pred_result[3]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+16]            // ME_PRED_BASE_REG[4]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+16]     // -> pred_result[4]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+20]            // ME_PRED_BASE_REG[5]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+20]     // -> pred_result[5]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+24]            // ME_PRED_BASE_REG[6]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+24]     // -> pred_result[6]
        ldr     r12,[h264_base,#ME_PRED_BASE_REG+28]            // ME_PRED_BASE_REG[7]
        str     r12,[r9,#P264_P6_RAW_RESULT_pred_result+28]     // -> pred_result[7]

        // increment nb_mb_encoded
        add     r13,#1
        str      r13,[pic_encode_ctx,#PICTURE_ENCODING_CTX_nb_mb_encoded]


       // increment MB index
       ldr      r9,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_i_MB]   // r9  = current_i_MB
       ldr      r12,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_width] // r12 =  current_width
       lsr      r12,#4                  // r12 = number of MB per line
       add      r9,r9,#1
       cmp      r9,r12                  // picture_encoding_context.current_i_MB >= (picture_encoding_context.current_width>>4 ?
       movge    r9,#0
       ldr      r12,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_j_MB]
       addge    r12,r12,#1
       // store new current_i_MB,new current_j_MB
       str      r12,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_j_MB]
       str      r9,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_i_MB]

       // compute (picture_encoding_context.current_j_MB<<24)|(picture_encoding_context.current_i_MB<<16)|(picture_encoding_context.current_j_MB<<8)|picture_encoding_context.current_i_MB
       ldr      r9,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_i_MB]
       ldr      r12,[pic_encode_ctx,#PICTURE_ENCODING_CTX_current_j_MB]
       orr      r9,r9,r9,LSL#16
       lsl      r12,#8
       orr      r9,r9,r12
       orr      r9,r9,r12,LSL#16

       // set MB index
       str      r9,[h264_base,#H264_MB_ADDR]

       // increment output phys addr
       ldr       r9,[h264_base,#DCT_DEST_Y_ADDR]  // load DCT_DEST_Y_ADDR
       add       r9,#MAX_ENCODED_MB_SIZE          // 864 is the maximum size of an encoded MB
       str      r9,[h264_base,#DCT_DEST_Y_ADDR]  // store new DCT_DEST_Y_ADDR

       // launch next MB
       // compare  nb_mb_to_encode and nb_mb_encoded
       ldr       r9,[pic_encode_ctx,#PICTURE_ENCODING_CTX_nb_mb_to_encode]  // nb_mb_to_encode
       ldr       r12,[pic_encode_ctx,#PICTURE_ENCODING_CTX_nb_mb_encoded]   // nb_mb_encoded
       cmp      r12,r9
       strlt    r9,[h264_base,#H264_START]    // if (nb_mb_encoded<nb_mb_to_encode) write something to reg H264_START
       movge    r9,#0
       strge    r9,[pic_encode_ctx,#PICTURE_ENCODING_CTX_is_running]  // fiq end
       /* return */
       subs  pc, lr, #4

/* registers */
.h264_base:
        .word 0xfd060000
.cst:
        .word 0xFFFFFFFF

h264_p6_fiq_handler_end:
    .end

