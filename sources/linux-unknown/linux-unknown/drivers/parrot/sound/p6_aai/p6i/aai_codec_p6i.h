/**
 * @file   aai_codec_p6i.h
 * @brief  P6i internal codec control
 *
 * @author adrien.charruel@parrot.com
 * @date   2010-11-24
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 */

#ifndef INCONCE_AAI_CODEC_P6I_H
#define INCONCE_AAI_CODEC_P6I_H

void aai_codec_latch(int blocking);
int aai_codec_set_recsel(struct card_data_t *aai, int val);
int aai_codec_set_i2sfreq(struct card_data_t *aai, int val);

#endif

