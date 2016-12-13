/**
 * @brief Device I/O - Description of Parrot6 lcd hardware
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     cedric.papon@parrot.com
 * @author     matthieu.castet@parrot.com
 * @date       2008-05-01
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

#ifndef P6_LCDC_H
#define P6_LCDC_H

////////////////////////////////////////////////////
// Parrot proprietary P6_LCDC registers
////////////////////////////////////////////////////

#define P6_LCDC_STAT     0x0000       // STATUS
#define P6_LCDC_ITEN     0x0004       // INTERRUPT ENABLE
#define P6_LCDC_ITACK    0x0008       // INTERRUPT ACKNOWLEDGE
#define P6_LCDC_CTRL     0x000c       // CONTROL

#define P6_LCDC_FIFO     0x0010       // FIFOs TRESHOLDS
#define P6_LCDC_DMAO     0x0014       // DMA OFFSETs
#define P6_LCDC_DMA      0x0018       // DMA

#define P6_LCDC_PCR      0x001C       // PANEL CONTROL
#define P6_LCDC_VTR      0x0020       // VERTICAL TIMING
#define P6_LCDC_HTR      0x0024       // HORIZONTAL TIMING
#define P6_LCDC_PSR      0x0028       // PANEL SIZE

#define P6_LCDC_OSD_CTL  0x002C       // OSD CONTROL  
#define P6_LCDC_RGB_CTL  0x0030       // RGB CONTROL
#define P6_LCDC_YCC1_S   0x0034       // YCC 1 SIZE
#define P6_LCDC_YCC2_S   0x0038       // YCC 2 SIZE
#define P6_LCDC_YCC1_P   0x003C       // YCC 1 POSITION
#define P6_LCDC_YCC2_P   0x0040       // YCC 2 POSITION

#define P6_LCDC_YCC1_BUFFER 0x0044
#define P6_LCDC_YCC2_BUFFER 0x0048

#define P6_LCDC_RGB_A1   0x0050       // RGB ADDRESS 1
#define P6_LCDC_RGB_A2   0x0054       // RGB ADDRESS 2

#define P6_LCDC_Y1_A1    0x0058       // Y1 ADDRESS 1
#define P6_LCDC_Y1_A2    0x005c       // Y1 ADDRESS 2
#define P6_LCDC_Y1_A3    0x0060       // Y1 ADDRESS 3
#define P6_LCDC_CB1_A1   0x0064       // CB1 ADDRESS 1
#define P6_LCDC_CB1_A2   0x0068       // CB1 ADDRESS 2
#define P6_LCDC_CB1_A3   0x006c       // CB1 ADDRESS 3
#define P6_LCDC_CR1_A1   0x0070       // CR1 ADDRESS 1
#define P6_LCDC_CR1_A2   0x0074       // CR1 ADDRESS 2
#define P6_LCDC_CR1_A3   0x0078       // CR1 ADDRESS 3
#define P6_LCDC_OSD1_A1  0x007c       // OSD1 ADDRESS 1
#define P6_LCDC_OSD1_A2  0x0080       // OSD1 ADDRESS 2
#define P6_LCDC_OSD1_A3  0x0084       // OSD1 ADDRESS 3

#define P6_LCDC_Y2_A1    0x0088       // Y2 ADDRESS 1
#define P6_LCDC_Y2_A2    0x008c       // Y2 ADDRESS 2
#define P6_LCDC_Y2_A3    0x0090       // Y2 ADDRESS 3
#define P6_LCDC_CB2_A1   0x0094       // CB2 ADDRESS 1
#define P6_LCDC_CB2_A2   0x0098       // CB2 ADDRESS 2
#define P6_LCDC_CB2_A3   0x009c       // CB2 ADDRESS 3
#define P6_LCDC_CR2_A1   0x00a0       // CR2 ADDRESS 1
#define P6_LCDC_CR2_A2   0x00a4       // CR2 ADDRESS 2
#define P6_LCDC_CR2_A3   0x00a8       // CR2 ADDRESS 3
#define P6_LCDC_OSD2_A1  0x00AC       // OSD2 ADDRESS 1
#define P6_LCDC_OSD2_A2  0x00B0       // OSD2 ADDRESS 2
#define P6_LCDC_OSD2_A3  0x00B4       // OSD2 ADDRESS 3

#define P6_LCDC_YCC1TORGB_CR 0x00B8
#define P6_LCDC_YCC1TORGB_CB 0x00BC
#define P6_LCDC_YCC2TORGB_CR 0x00c0
#define P6_LCDC_YCC2TORGB_CB 0x00c4

////////////////////////////////////////////////////
// Power-On Reset register contents
////////////////////////////////////////////////////

#define P6_LCDC_RST_STAT     0x00030300
#define P6_LCDC_RST_OTHERS   0x00000000 

////////////////////////////////////////////////////
// Registers bitwise definitions
////////////////////////////////////////////////////

// Status register

#define P6_LCDC_STAT_LCD         (1<<0)  // LCD Event flag
#define P6_LCDC_STAT_UNDF        (1<<1)  // FIFO Underflow flag
#define P6_LCDC_STAT_ERR         (1<<2)  // LCD Error flag

#define P6_LCDC_STAT_HMASK       (3<<8)  // Mask for HStatus
#define P6_LCDC_STAT_HSYNC       (0<<8)  // HSYNC
#define P6_LCDC_STAT_HBACK       (1<<8)  // Back Porch
#define P6_LCDC_STAT_HACTD       (2<<8)  // Active Display
#define P6_LCDC_STAT_HFRNT       (3<<8)  // Front Porch        

#define P6_LCDC_STAT_VMASK       (3<<16) // Mask for VStatus
#define P6_LCDC_STAT_VSYNC       (0<<16) // VSYNC
#define P6_LCDC_STAT_VBACK       (1<<16) // Back Porch
#define P6_LCDC_STAT_VACTD       (2<<16) // Active Display
#define P6_LCDC_STAT_VFRNT       (3<<16) // Front Porch        

// Interrupt Enable register

#define P6_LCDC_ITEN_LCD          (1<<0) // 
#define P6_LCDC_ITEN_UNDF         (1<<1) // 
#define P6_LCDC_ITEN_ERR          (1<<2) //
#define P6_LCDC_ITEN_ALL          (7<<0) //

// Interrupt Acknowledge register

#define P6_LCDC_ITACK_LCD         (1<<0) // 
#define P6_LCDC_ITACK_UND         (1<<1) // 
#define P6_LCDC_ITACK_ERR         (1<<2) //
#define P6_LCDC_ITACK_ALL         (7<<0) //

// Control register 

#define P6_LCDC_CTRL_OUTEN        (1<<0) // Outputs (VCLK, VSYNC, HSYNC, ...) enable

#define P6_LCDC_CTRL_IT_VSYNC     (0<<1) // IT Event = Start of VSync
#define P6_LCDC_CTRL_IT_BACK      (1<<1) // IT Event = Start of Back Porch
#define P6_LCDC_CTRL_IT_ACTD      (2<<1) // IT Event = Start of Active Display
#define P6_LCDC_CTRL_IT_FRONT     (3<<1) // IT Event = Start of Front Porch

#define P6_LCDC_CTRL_RGB_16       (0<<8) // RGB Data in memory is 16-bits wide in 5:5:5:1 format (2 pixels/word)
#define P6_LCDC_CTRL_RGB_24       (1<<8) // RGB Data in memory is 24-bits wide in 8:8:8 format (1 pixel/word)

#define P6_LCDC_CTRL_RGB_EN       (1<<9) // Display RGB Background layer
#define P6_LCDC_CTRL_YCC1_EN     (1<<10) // Display YCC/RGB Plan 1
#define P6_LCDC_CTRL_YCC2_EN     (1<<11) // Display YCC/RGB Plan 2
#define P6_LCDC_CTRL_YCCX_EN(X)  (((X)==1)?P6_LCDC_CTRL_YCC1_EN:P6_LCDC_CTRL_YCC2_EN)

#define P6_LCDC_CTRL_YCC1_RGB    (1<<16) // Plan 1 is fetched with RGB data (YCC 4:2:0 otherwise)
#define P6_LCDC_CTRL_YCC2_RGB    (1<<17) // Plan 2 is fetched with RGB data (YCC 4:2:0 otherwise)
#define P6_LCDC_CTRL_YCCX_RGB(X) (((X)==1)?P6_LCDC_CTRL_YCC1_RGB:P6_LCDC_CTRL_YCC2_RGB)

#define P6_LCDC_CTRL_YCC1_RGB16  (0<<18) // RGB Data in Plan 1 is 16 bits (5:5:5:1) 
#define P6_LCDC_CTRL_YCC1_RGB24  (1<<18) // RGB Data in Plan 1 is 24 bits (8:8:8)

#define P6_LCDC_CTRL_YCC2_RGB16  (0<<19) // RGB Data in Plan 2 is 16 bits (5:5:5:1) 
#define P6_LCDC_CTRL_YCC2_RGB24  (1<<19) // RGB Data in Plan 2 is 24 bits (8:8:8)
#define P6_LCDC_CTRL_YCCX_RGB24(X) (((X)==1)?P6_LCDC_CTRL_YCC1_RGB24:P6_LCDC_CTRL_YCC2_RGB24)

#define P6_LCDC_CTRL_RGB_OUT     (0<<24) // P6_LCDC output format is RGB
#define P6_LCDC_CTRL_ITU_OUT     (1<<24) // P6_LCDC output format is ITU-656

// DMA Register

#define P6_LCDC_DMA_INCR          (0<<0) // Incr Bursts
#define P6_LCDC_DMA_INCR4         (1<<0) // Incr4 Bursts
#define P6_LCDC_DMA_INCR8         (2<<0) // Incr8 Bursts
#define P6_LCDC_DMA_INCR16        (3<<0) // Incr16 Bursts
#define P6_LCDC_DMA_IDLE          (1<<2) // Insert IDLE cycles between each transfers

// Panel Control Register
                                      // Panel VCLK clocking scheme :
#define P6_LCDC_PCR_VCLK_EDGE    (0<<16) // Rising edge
#define P6_LCDC_PCR_VCLK_EDGEN   (1<<16) // Falling edge

                                      // Panel Video Data Interface :
#define P6_LCDC_PCR_RGB_PAR      (0<<17) // Parrallel RGB
#define P6_LCDC_PCR_BGR_PAR      (1<<17) // Parrallel BGR
#define P6_LCDC_PCR_RGB_SER      (2<<17) // Serial R G B
#define P6_LCDC_PCR_BGR_SER      (3<<17) // Serial B G R
//                                                                 _
#define P6_LCDC_PCR_MSK			(0x1f)
#define P6_LCDC_PCR_VSYNC_P       (0<<8) // Vsync normal polarity   _| |_ or _   _
#define P6_LCDC_PCR_VSYNC_N       (1<<8) // Vsync inverted polarity           |_|

#define P6_LCDC_PCR_HSYNC_P       (0<<9)
#define P6_LCDC_PCR_HSYNC_N       (1<<9)

#define P6_LCDC_PCR_VDEN_P       (0<<10)
#define P6_LCDC_PCR_VDEN_N       (1<<10)

#define P6_LCDC_PCR_PANEL_18     (0<<11) // Panel is 18 bits (262 kcolors)
#define P6_LCDC_PCR_PANEL_24     (1<<11) // Panel is 24 bits (16.7 Mcolors)

// OSD Control Register

#define P6_LCDC_OSD_CTL_1PLANE   (0<<16) // Per Plane Blending for Plan 1
#define P6_LCDC_OSD_CTL_1PIXEL   (1<<16) // Per Pixel Blending for Plan 1

#define P6_LCDC_OSD_CTL_2PLANE   (0<<17)
#define P6_LCDC_OSD_CTL_2PIXEL   (1<<17)
#define P6_LCDC_OSD_CTL_XPIXEL(X) (((X)==1)?P6_LCDC_OSD_CTL_1PIXEL:P6_LCDC_OSD_CTL_2PIXEL)

#define P6_LCDC_OSD_CTL_1_EN     (1<<18) // OSD1 enable
#define P6_LCDC_OSD_CTL_2_EN     (1<<19) // OSD2 enable
#define P6_LCDC_OSD_CTL_X_EN(X)  (((X)==1)?P6_LCDC_OSD_CTL_1_EN:P6_LCDC_OSD_CTL_2_EN)

// RGB Control Register

#define P6_LCDC_RGB_CTL_UNIFILL  (1<<24) // RGB Unifill mode
#define P6_LCDC_RGB_CTL_BRIGHT   (1<<25) // RGB Brightness mode

// YCC1 Size Register

#define P6_LCDC_YCC1_S_DS1       (0<<28) // No Downscaling
#define P6_LCDC_YCC1_S_DS4       (1<<28) // Downscaling by 4
#define P6_LCDC_YCC1_S_DS16      (1<<29) // Downscaling by 16

// YCC2 Size Register

#define P6_LCDC_YCC2_S_DS1       (0<<28) // No Downscaling
#define P6_LCDC_YCC2_S_DS4       (1<<28) // Downscaling by 4
#define P6_LCDC_YCC2_S_DS16      (1<<29) // Downscaling by 16

// YCC1 Buffers Register
#define P6_LCDC_YCC1_BUFFER_MASK   (3)    // Next frame buffer
#define P6_LCDC_YCC1_SRC           (1<<4) // Manual address switch
#define P6_LCDC_YCC1_TRIPLE_BUFFER (1<<5) // Triple buffer

// YCC2 Buffers Register
#define P6_LCDC_YCC2_BUFFER_MASK   (3)    // Next frame buffer
#define P6_LCDC_YCC2_SRC           (1<<4) // Manual address switch
#define P6_LCDC_YCC2_TRIPLE_BUFFER (1<<5) // Triple buffer

#endif
