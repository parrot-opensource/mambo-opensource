#ifndef INCONCE_AAIP5_REGS_H
#define INCONCE_AAIP5_REGS_H

//===========================================================================--
// Parrot SA
//
// Project      : Parrot 6
// File name    : aai_regs.h
// Author       : Virginia Martin Heriz
// Change log   : At the end of the file, automatically written by CVS.
// CVS ID       : $Id: regs-aai-p5p.h,v 1.2 2008-11-13 10:37:28 getienne Exp $
//
// ----------------------------------------------------------------------
// Purpose : AAI register offset definition
//           AAI register fields definition
//           AAI register read/write macros  
//
//===========================================================================--


#define P5P_AAI_REGS       0xD0010000
#define P5P_AAI_REGS_SIZE  0x10000
#define P5P_AAI_IRQ        (2+64)



// Register map

#define _AAI_MULT_OUT      0x000   //!< Multimedia Stereo output FIFO (W)
#define _AAI_MULT_IN1      0x040   //!< Multimedia Stereo input1 FIFO (R)
#define _AAI_MULT_IN2      0x080   //!< Multimedia Stereo input2 FIFO (R)
#define _AAI_BT_OUT        0x0C0   //!< Bluetooth PCM Stereo output FIFO (W)
#define _AAI_BT_IN         0x100   //!< Bluetooth PCM Stereo input FIFO (R)
#define _AAI_EXT_OUT       0x140   //!< External PCM Stereo output FIFO (W)
#define _AAI_EXT_IN        0x180   //!< External PCM Stereo input FIFO (R)

#define _AAI_CFG           0x1C0   //!< General Configuration Register (RW)
#define _AAI_FE            0x1C4   //!< Flag Enable Register (RW)
#define _AAI_INT           0x1C8   //!< Interrupt Flag Register (R)

#define _AAI_BT_MULT_SYNC  0x1CC   //!< Bluetooth/Multimedia sync (R)
#define _AAI_EXT_MULT_SYNC 0x1D0   //!< External/Multimedia synch (R)
#define _AAI_MULT_BT_SYNC  0x1D4   //!< Multimedia/Bluetooth sync (R)
#define _AAI_EXT_BT_SYNC   0x1D8   //!< External/Bluetooth sync (R)
#define _AAI_MULT_EXT_SYNC 0x1DC   //!< Multimedia/External sync (R)
#define _AAI_BT_EXT_SYNC   0x1E0   //!< Bluetooth/External sync (R)
#define _AAI_BT_PCM_SI1    0x1E4   //!< Bluetooth PCM 1st slot interval Register (RW) (P5+ only)
#define _AAI_EXT_PCM_SI1   0x1E8   //!< External PCM 1st slot interval Register (RW) (P5+ only)
#define _AAI_BT_PCM_SI2    0x1EC   //!< Bluetooth PCM 2nd slot interval Register (RW) (P5+ only)

#define _AAI_FLATT         0x200   //!< Front Left loudspeaker attenuation (RW)
#define _AAI_FRATT         0x204   //!< Front Right loudspeaker attenuation(RW)
#define _AAI_RLATT         0x208   //!< Rear Left loudspeaker attenuation (RW)
#define _AAI_RRATT         0x20C   //!< Rear Right loudspeaker attenuation (RW)

#define _AAI_BT_OUT2       0x240   //!< Bluetooth 2 PCM output FIFO (W) (P5+ only)
#define _AAI_BT_IN2        0x280   //!< Bluetooth 2 PCM input FIFO (R) (P5+ only)
#define _AAI_EXT_OUT2      0x2C0   //!< External 2 PCM output FIFO (W) (P5+ only)
#define _AAI_EXT_IN2       0x300   //!< External 2 PCM input FIFO (R) (P5+ only)
#define _AAI_BT_OUT3       0x340   //!< Bluetooth 3 PCM output FIFO (W) (P5+ only)
#define _AAI_BT_IN3        0x380   //!< Bluetooth 3 PCM input FIFO (R) (P5+ only)
#define _AAI_MULT_FEEDBACK 0x3C0   //!< Feedback Multimedia input FIFO (R) (P5+ only)

// Registers bitwise definitions

// General Configuration Register
#define AAI_CFG_COMPACT         0x1     //!< Sampling frequency(0=48 kHz,1=44.1 kHz)

#define AAI_CFG_RATE_SLAVE      (1<<1)  //!< Sampling rate master(0)/slave(1) /* unused */
#define AAI_CFG_SYNC_SLAVE      (1<<2)  //!< Sync source master(0)/slave(1) /* unused */
#define AAI_CFG_MSB_PHASE       (1<<3)  //!< MSB=1 clk after frame(0)/in phase(1) /* unused */
#define AAI_CFG_LEFT_FRAME      (1<<4)  //!< External frame for left channel (0/1) /* unused */

#define AAI_CFG_MASTER_CLK      (3<<5)  //!< Master clock output rate
#define AAI_CFG_MASTER_CLK_SHIFT   (5)  //!< Shift for master clock output rate

#define AAI_CFG_SYNC_EDGE       (1<<7)  //!< Falling(0)/rising edge sync/serial bit /* unused */

#define AAI_CFG_EXT_CODEC       (1<<8)  //!< External PCM Mode (0=Texas, 1=OKI)

#define AAI_CFG_BT_TO_EXT       (1<<9)  //!< BT directed to EXT (1=En if EXT_USED) /* unused */


#define AAI_CFG_RUN_MULT        (1<<10) //!< Multimedia state (1=running,0=stopped)
#define AAI_CFG_MIC34_PAD       (1<<11) //!< Ext Mic 3&4 pad use (0=unused,1=used)
#define AAI_CFG_RUN_BT          (1<<12) //!< Bluetooth state (1=running,0=stopped)
#define AAI_CFG_RUN_EXT         (1<<13) //!< External state (1=running,0=stopped)
#define AAI_CFG_RUN_DAC         (1<<14) //!< Analogic internal DAC enable
#define AAI_CFG_RUN_ADC         (1<<15) //!< Analogic internal ADC enable
#define AAI_CFG_FEEDBACK        (1<<16) //!< Feedback Stream Activation(P5+ only)
#define AAI_CFG_MASTER_BLUE     (1<<17) //!< Bluetooth PCM master/slave control mode(P5+ only)
#define AAI_CFG_BLUE_16K        (1<<18) //!< Bluetooth PCM 16kHz Enable(P5+ only)
#define AAI_CFG_MASTER_EXT      (1<<19) //!< External PCM master/slave control mode(P5+ only)
#define AAI_CFG_EXT_16K         (1<<20) //!< External PCM 16kHz Enable(P5+ only)

// Flag Enable Configuration Register
#define AAI_FE_SPK                   0x1 //!< Multimedia Audio Output Enable
#define AAI_FE_MIC12             (1<<1) //!< Multimedia Channel1 Audio Input Enable
#define AAI_FE_MIC34             (1<<2) //!< Multimedia Channel2 Audio Input Enable
#define AAI_FE_MIC16KHZ          (1<<3) //!< Sampling rate for mic(0=8 kHz,1=16 kHz)
#define AAI_FE_BT_TX             (1<<4) //!< Bluetooth Audio Output Enable
#define AAI_FE_BT_RX             (1<<5) //!< Bluetooth Audio Input Enable
#define AAI_FE_EXT_TX            (1<<6) //!< External Audio Output Enable
#define AAI_FE_EXT_RX            (1<<7) //!< External Audio Input Enable
#define AAI_FE_DSP_MODE          (1<<8) //!< 8 kHz/DSP mode enable (0=8/16kHz,1=DSP)
#define AAI_FE_SURROUND          (1<<9) //!< 4 independent SPK lines
#define AAI_FE_ADC_EXT          (1<<10) //!< External ADC enable
#define AAI_FE_FEEDBACK         (1<<11) //!< Feedback Audio Input Enable(P5+ only)
#define AAI_FE_BT_TX2           (1<<12) //!< Bluetooth 2 Audio Output Enable(P5+ only)
#define AAI_FE_BT_RX2           (1<<13) //!< Bluetooth 2 Audio Input Enable(P5+ only)
#define AAI_FE_EXT_TX2          (1<<14) //!< External 2 Audio Output Enable(P5+ only)
#define AAI_FE_EXT_RX2          (1<<15) //!< External 2 Audio Input Enable(P5+ only)
#define AAI_FE_BT_TX3           (1<<16) //!< Bluetooth 3 Audio Output Enable(P5+ only)
#define AAI_FE_BT_RX3           (1<<17) //!< Bluetooth 3 Audio Input Enable(P5+ only)
#define AAI_FE_SPK_MODE          (3<<18) //!< Mask for loud-speaker stream mode(P5+ only)
#define AAI_FE_SPK_MODE_SHIFT       (18) //!< Shift for loud-speaker stream mode(P5+ only)
#define AAI_FE_FEEDBACK_MODE    (3<<20) //!< Mask for feedback stream mode(P5+ only)
#define AAI_FE_FEEDBACK_SRC     (1<<22) //!< Feedback Source selection(P5+ only)
#define AAI_FE_FEEDBACK_MODE_SHIFT (20) //!< Shift for feedback stream mode(P5+ only)
#define AAI_FE_MIC12_MODE       (3<<23) //!< Mask for mic 1&2 stream mode(P5+ only)
#define AAI_FE_MIC12_MODE_SHIFT    (23) //!< Shift for mic 1&2 stream mode(P5+ only)
#define AAI_FE_MIC12_VOLT       (1<<25) //!< mic 1&2 voltage selection(P5+ only)
#define AAI_FE_MIC34_MODE       (3<<26) //!< Mask for mic 3&4 stream mode(P5+ only)
#define AAI_FE_MIC34_MODE_SHIFT    (26) //!< Shift for mic 3&4 stream mode(P5+ only)
#define AAI_FE_MIC34_VOLT       (1<<28) //!< mic 3&4 voltage selection(P5+ only)
#define AAI_FE_MIC34_SRC        (1<<29) //!< mic 3&4 Source selection(P5+ only)

// Interrupt Flag Register
#define AAI_INT_SPK         0x1      //!< 8 samples can be written to SPK fifo
#define AAI_INT_MIC12      (1<<1)   //!< 8 samples can be read from MIC12 fifo
#define AAI_INT_MIC34      (1<<2)   //!< 8 samples can be read from MIC34 fifo
#define AAI_INT_BT_TX      (1<<3)   //!< 8 samples can be written to BT_OUT fifo
#define AAI_INT_BT_RX      (1<<4)   //!< 8 samples can be read from BT_IN fifo
#define AAI_INT_EXT_TX     (1<<5)   //!< 8 samples can be written to EXT_OUT fifo
#define AAI_INT_EXT_RX     (1<<6)   //!< 8 samples can be read from EXT_IN fifo
#define AAI_INT_FEEDBACK   (1<<7)   //!< 8/16 samples can be read from MULT_BACK fifo(P5+ only)
#define AAI_INT_BT_TX2     (1<<8)   //!< 8/16 samples can be written to BT_OUT2 fifo(P5+ only)
#define AAI_INT_BT_RX2     (1<<9)   //!< 8/16 samples can be read from BT_IN2 fifo(P5+ only)
#define AAI_INT_EXT_TX2    (1<<10)  //!< 8/16 samples can be written to EXT_OUT2 fifo(P5+ only)
#define AAI_INT_EXT_RX2    (1<<11)  //!< 8/16 samples can be read from EXT_IN2 fifo(P5+ only)
#define AAI_INT_BT_TX3     (1<<12)  //!< 8/16 samples can be written to BT_OUT3 fifo(P5+ only)
#define AAI_INT_BT_RX3     (1<<13)  //!< 8/16 samples can be read from BT_IN3 fifo(P5+ only)


#define AAI_PCM_SI_MASK    (0xFF) //!< PCM Slots's Mask shift value
#define AAI_PCM_SI_MIN     (0x0F) //!< PCM Slots's Minimum shift value
#define AAI_PCM_SI_MAX     (0xEF) //!< PCM Slots's Maximum shift value 
// Bluetooth PCM 1st slot interval Register 
#define AAI_BT_SI1_OKI    (1<<8) //!< Bluetooth PCM OKI/Texas mode(P5+ only)
// External PCM 1st slot interval Register 
#define AAI_EXT_SI1_OKI   (1<<8) //!< External PCM OKI/Texas mode(P5+ only)
// Bluetooth PCM 2nd slot interval Register
#define AAI_BT_SI2_OKI    (1<<8) //!< Bluetooth PCM OKI/Texas mode(P5+ only) 


#endif /* CYGONCE_AAI_REGS_H */
