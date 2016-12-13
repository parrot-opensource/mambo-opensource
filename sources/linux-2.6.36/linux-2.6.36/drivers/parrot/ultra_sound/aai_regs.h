//===========================================================================--
// Parrot SA Confidential Property
//
// Project      : Parrot 6+
// File name    : aai_regs.h
// Author       : Virginia Martin Heriz
// SVN ID       : $Id: aai_regs.h 1982 2009-11-04 12:54:35Z marc $
//
// ----------------------------------------------------------------------
// Purpose : AAI register offset definition
//           AAI register fields definition
//           AAI register read/write macros
//
//===========================================================================--

//////////////////////////////////////////////////////////////////////////////////////////////
// Register map
//////////////////////////////////////////////////////////////////////////////////////////////

#define   AAI_LEFT_MUSIC_OUT0                  0x000     // Multimedia front left loudspeaker FIFO
#define   AAI_RIGHT_MUSIC_OUT0                 0x020     // Multimedia front right loudspeaker FIFO
#define   AAI_LEFT_MUSIC_OUT1                  0x040     // Multimedia rear left loudspeaker FIFO
#define   AAI_RIGHT_MUSIC_OUT1                 0x060     // Multimedia rear right loudspeaker FIFO

#define   AAI_16KHZ_OUT                        0x080     // 16KHz mono output FIFO
#define   AAI_8KHZ_OUT                         0x0A0     // 8KHz mono output FIFO

//#define   AAI_AUX_OUT                          0x0C0     // 16KHz mono output FIFO
//#define   AAI_MUSIC_ICH1                       0x0E0     // 8KHz mono output FIFO

#define   AAI_16KHZ_ICH0                       0x100     // 16KHz dual input1 FIFO
#define   AAI_8KHZ_ICH0                        0x120     // 8KHz dual input1 FIFO
//#define   AAI_16KHZ_ICH2                       0x140     // 16KHz dual input2 FIFO
//#define   AAI_8KHZ_ICH2                        0x160     // 8KHz dual input2 FIFO
  #define   AAI_MUSIC_ICH2                       0x140     // music input 2 FIFO

#define   AAI_16KHZ_BACK                       0x180     // 16KHz Stereo feedback FIFO
#define   AAI_8KHZ_BACK                        0x1A0     // 8KHz Stereo feedback FIFO

//#define   AAI_MUSIC_ICH3                       0x1C0     // music input 3 FIFO

//#define   AAI_SO1_PCM0                         0x1E0     // PCM1 Audio output1 FIFO
//#define   AAI_SI1_PCM0                         0x200     // PCM1 Audio input1 FIFO
//#define   AAI_SO2_PCM0                         0x220     // PCM1 Audio output2 FIFO
//#define   AAI_SI2_PCM0                         0x240     // PCM1 Audio input2 FIFO
//#define   AAI_SO3_PCM0                         0x260     // PCM1 Audio output3 FIFO
//#define   AAI_SI3_PCM0                         0x280     // PCM1 Audio input3 FIFO
//#define   AAI_SO1_PCM1                         0x2A0     // PCM2 Audio output1 FIFO
//#define   AAI_SI1_PCM1                         0x2C0     // PCM2 Audio input1 FIFO
//#define   AAI_SO2_PCM1                         0x2E0     // PCM2 Audio output2 FIFO
//#define   AAI_SI2_PCM1                         0x300     // PCM2 Audio input2 FIFO
  #define   AAI_SO1_PCM                          0x1E0     // PCM Audio output1 FIFO
  #define   AAI_SI1_PCM                          0x200     // PCM Audio input1 FIFO
  #define   AAI_SO2_PCM                          0x220     // PCM Audio output2 FIFO
  #define   AAI_SI2_PCM                          0x240     // PCM Audio input2 FIFO

  #define   AAI_ADC_DATA                         0x300     // ADC Results

#define   AAI_CFG                              0x320     // AAI General configuration register
#define   AAI_ITEN                             0x324     // AAI interrupt enable regster
#define   AAI_ITS                              0x328     // interrupt status register

#define   AAI_SRC                              0x32C     // AAI multimedia rate control register

#define   AAI_LEFT_MUSIC_VOL_OUT0              0x330     // Front left loudspeaker volume register
#define   AAI_RIGHT_MUSIC_VOL_OUT0             0x334     // Front right loudspeaker volume register
#define   AAI_LEFT_MUSIC_VOL_OUT1              0x338     // Rear left loudspeaker volume register
#define   AAI_RIGHT_MUSIC_VOL_OUT1             0x33C     // Rear right loudspeaker volume register

//#define   AAI_LEFT_AUX_VOL_OUT0                0x340     // Front left loudspeaker volume register
//#define   AAI_RIGHT_AUX_VOL_OUT0               0x344     // Front right loudspeaker volume register
  #define   AAI_CHANNEL1_VOL_OUT0                     0x340     // Auxiliary volume on Output0 register
//#define   AAI_LEFT_AUX_VOL_OUT1                0x348     // Rear left loudspeaker volume register
//#define   AAI_RIGHT_AUX_VOL_OUT1               0x34C     // Rear right loudspeaker volume register
  #define   AAI_CHANNEL1_VOL_OUT1                     0x348     // Auxiliary volume on Output1 register

//#define   AAI_LEFT_16KHZ_VOL_OUT0              0x350     // Front left loudspeaker volume register
//#define   AAI_RIGHT_16KHZ_VOL_OUT0             0x354     // Front right loudspeaker volume register
  #define   AAI_16KHZ_VOL_OUT0                   0x350     // 16 KHz volume on Output0 register
//#define   AAI_LEFT_16KHZ_VOL_OUT1              0x358     // Rear left loudspeaker volume register
//#define   AAI_RIGHT_16KHZ_VOL_OUT1             0x35C     // Rear right loudspeaker volume register
  #define   AAI_16KHZ_VOL_OUT1                   0x358     // 16 KHz volume on Output1 register

//#define   AAI_LEFT_8KHZ_VOL_OUT0               0x360     // Front left loudspeaker volume register
//#define   AAI_RIGHT_8KHZ_VOL_OUT0              0x364     // Front right loudspeaker volume register
  #define   AAI_8KHZ_VOL_OUT0                    0x360     // 8 KHz volume on Output0 register
//#define   AAI_LEFT_8KHZ_VOL_OUT1               0x368     // Rear left loudspeaker volume register
//#define   AAI_RIGHT_8KHZ_VOL_OUT1              0x36C     // Rear right loudspeaker volume register
  #define   AAI_8KHZ_VOL_OUT1                    0x368     // 8 KHz volume on Output1 register

#define   AAI_I2S_FORMAT                       0x370     // Rear right loudspeaker volume register
//#define   AAI_PCM0_CONF                        0x374     // PCM1 configuration register
//#define   AAI_PCM1_CONF                        0x378     // PCM2 configuration register
  #define   AAI_PCM_CONF                         0x374     // PCM configuration register
#define   AAI_DMA_CTRL                         0x37C     // DMA configuration register

//#define   AAI_PCM01_SYNC                       0x380     // PCM1/PCM2 synchronization register
//#define   AAI_PCM10_SYNC                       0x384     // PCM2/PCM1 synchronization register

  #define   AAI_LOUD_REF                         0x388     // Loudness Reference Volume register
  #define   AAI_LOUD_CTRL                        0x390     // Loudness Control register
  #define   AAI_CHANNEL2_CONFIG                  0x394     // Channel2 Input Configuration register
  #define   AAI_USB_SAMPLE                       0x398     // Samples between Start Of Frame register

  #define   AAI_ADC_MODES                        0x39C     // ADC configuration register
  #define   AAI_ADC_ULTRA                        0x3A0     // ADC Ultra sounds capture register
  #define   AAI_ADC_MONITOR                      0x3A4     // ADC DC value monitoring register
  #define   AAI_ADC_LIMITS                       0x3A8     // ADC battery limits register
  #define   AAI_ADC_BATTERY_VALUE                0x3AC     // ADC battery value register

#define   AAI_LEFT_MUSIC_DMASA_OUT0            0x400
#define   AAI_LEFT_MUSIC_DMAFA_OUT0            0x404
#define   AAI_RIGHT_MUSIC_DMASA_OUT0           0x408
#define   AAI_RIGHT_MUSIC_DMAFA_OUT0           0x40C

#define   AAI_LEFT_MUSIC_DMASA_OUT1            0x410
#define   AAI_LEFT_MUSIC_DMAFA_OUT1            0x414
#define   AAI_RIGHT_MUSIC_DMASA_OUT1           0x418
#define   AAI_RIGHT_MUSIC_DMAFA_OUT1           0x41C

#define   AAI_16KHZ_DMASA_OUT                  0x420
#define   AAI_16KHZ_DMAFA_OUT                  0x424
#define   AAI_8KHZ_DMASA_OUT                   0x428
#define   AAI_8KHZ_DMAFA_OUT                   0x42C

//#define   AAI_AUX_DMASA_OUT                    0x430
//#define   AAI_AUX_DMAFA_OUT                    0x434
//#define   AAI_MUSIC_DMASA_ICH1                 0x438
//#define   AAI_MUSIC_DMAFA_ICH1                 0x43C

#define   AAI_MUSIC_DMASA_ICH0                 0x440
#define   AAI_MUSIC_DMAFA_ICH0                 0x444
#define   AAI_16KHZ_DMASA_ICH0                 0x448
#define   AAI_16KHZ_DMAFA_ICH0                 0x44C
#define   AAI_8KHZ_DMASA_ICH0                  0x450
#define   AAI_8KHZ_DMAFA_ICH0                  0x454

#define   AAI_MUSIC_DMASA_ICH2                 0x458
#define   AAI_MUSIC_DMAFA_ICH2                 0x45C
//#define   AAI_16KHZ_DMASA_ICH2                 0x460
//#define   AAI_16KHZ_DMAFA_ICH2                 0x464
//#define   AAI_8KHZ_DMASA_ICH2                  0x468
//#define   AAI_8KHZ_DMAFA_ICH2                  0x46C
//#define   AAI_MUSIC_DMASA_ICH3                 0x470
//#define   AAI_MUSIC_DMAFA_ICH3                 0x474
#define   AAI_MUSIC_DMASA_BACK                 0x478
#define   AAI_MUSIC_DMAFA_BACK                 0x47C

#define   AAI_16KHZ_DMASA_BACK                 0x480
#define   AAI_16KHZ_DMAFA_BACK                 0x484
#define   AAI_8KHZ_DMASA_BACK                  0x488
#define   AAI_8KHZ_DMAFA_BACK                  0x48C

//#define   AAI_SO1_DMASA_PCM0                   0x490
//#define   AAI_SO1_DMAFA_PCM0                   0x494
//#define   AAI_SI1_DMASA_PCM0                   0x498
//#define   AAI_SI1_DMAFA_PCM0                   0x49C
//#define   AAI_SO2_DMASA_PCM0                   0x4A0
//#define   AAI_SO2_DMAFA_PCM0                   0x4A4
//#define   AAI_SI2_DMASA_PCM0                   0x4A8
//#define   AAI_SI2_DMAFA_PCM0                   0x4AC
//#define   AAI_SO3_DMASA_PCM0                   0x4B0
//#define   AAI_SO3_DMAFA_PCM0                   0x4B4
//#define   AAI_SI3_DMASA_PCM0                   0x4B8
//#define   AAI_SI3_DMAFA_PCM0                   0x4BC
//#define   AAI_SO1_DMASA_PCM1                   0x4C0
//#define   AAI_SO1_DMAFA_PCM1                   0x4C4
//#define   AAI_SI1_DMASA_PCM1                   0x4C8
//#define   AAI_SI1_DMAFA_PCM1                   0x4CC
//#define   AAI_SO2_DMASA_PCM1                   0x4D0
//#define   AAI_SO2_DMAFA_PCM1                   0x4D4
//#define   AAI_SI2_DMASA_PCM1                   0x4D8
//#define   AAI_SI2_DMAFA_PCM1                   0x4DC
  #define   AAI_SO1_DMASA_PCM                    0x490
  #define   AAI_SO1_DMAFA_PCM                    0x494
  #define   AAI_SI1_DMASA_PCM                    0x498
  #define   AAI_SI1_DMAFA_PCM                    0x49C
  #define   AAI_SO2_DMASA_PCM                    0x4A0
  #define   AAI_SO2_DMAFA_PCM                    0x4A4
  #define   AAI_SI2_DMASA_PCM                    0x4A8
  #define   AAI_SI2_DMAFA_PCM                    0x4AC

#define   AAI_DMASA_BINARY                     0x4E0
#define   AAI_DMAFA_BINARY                     0x4E4
  #define   ADC_DMASA_SAMP_OUT                   0x4E8
  #define   ADC_DMAFA_SAMP_OUT                   0x4EC
  #define   ADC_DMASA_SAMP_IN                    0x4F0
  #define   ADC_DMAFA_SAMP_IN                    0x4F4

#define   AAI_MUSIC_DMA_COUNT                  0x500
//#define   AAI_AUX_DMA_COUNT                    0x504
#define   AAI_VOICE_DMA_COUNT                  0x508
#define   AAI_BINARY_DMA_COUNT                 0x50C
  #define   AAI_SAMP_DMA_COUNT                   0x510
  #define   AAI_NOISE_DMA_COUNT                  0x514
  #define   AAI_ADC_USB_DMA_COUNT                0x518
  #define   AAI_DAC_USB_DMA_COUNT                0x51C

  #define   AAI_DMASA_ADC_USB                    0x520
  #define   AAI_DMAFA_ADC_USB                    0x524
  #define   AAI_DMASA_DAC_USB                    0x528
  #define   AAI_DMAFA_DAC_USB                    0x52C
  #define   AAI_DMASA_ADC_8KHZ                   0x530
  #define   AAI_DMAFA_ADC_8KHZ                   0x534
  #define   AAI_DMASA_ULTRA                      0x538
  #define   AAI_DMAFA_ULTRA                      0x53C

  #define   AAI_ULTRA_DMA_COUNT                  0x540

#define   AAI_DMA_INT_ACK                      0x600


//////////////////////////////////////////////////////////////////////////////////////////////
// Registers bitwise definitions
//////////////////////////////////////////////////////////////////////////////////////////////

// General Configuration Register (AAI_CFG @0x320)
#define   AAI_CFG_COMPACT                    (1<< 0)
//#define   AAI_CFG_AAI_SLAVE                  (1<< 1)
//#define   AAI_CFG_I2S_SLAVE                  (1<< 2)
#define   AAI_CFG_MSB_PHASE                  (1<< 3)
#define   AAI_CFG_LEFT_FRAME                 (1<< 4)
#define   AAI_CFG_SYNC_EDGE                  (1<< 5)
#define   AAI_CFG_MASTER_CLK_2               (0<< 6)
#define   AAI_CFG_MASTER_CLK_4               (1<< 6)
#define   AAI_CFG_MASTER_CLK_8               (2<< 6)
//#define   AAI_CFG_AUX_FIFO_DIS               (0<< 8)
//#define   AAI_CFG_AUX_FIFO_8                 (1<< 8)
//#define   AAI_CFG_AUX_FIFO_16                (2<< 8)
//#define   AAI_CFG_AUX_FIFO_24                (3<< 8)
//#define   AAI_CFG_AUX_SYNC_NONE              (0<<10)
//#define   AAI_CFG_AUX_SYNC_PARTIAL           (1<<10)
//#define   AAI_CFG_AUX_SYNC_DAC               (2<<10)
  #define   AAI_CFG_GPS_PROMPT                 (1<< 8)
#define   AAI_CFG_RUN_MULTI                  (1<<12)
#define   AAI_CFG_USE_ICH2                   (1<<13)
#define   AAI_CFG_FEEDBACK                   (1<<14)
//#define   AAI_CFG_USE_ICH3                   (1<<15)
#define   AAI_CFG_MUSIC_ICH0                 (1<<16)
//#define   AAI_CFG_MUSIC_ICH2                 (1<<17)
//#define   AAI_CFG_OUT1_BACK                  (1<<18)
  #define   AAI_CFG_IN1_BACK                   (1<<18)
#define   AAI_CFG_MUSIC_BACK                 (1<<19)
//#define   AAI_CFG_VOICE_SYNC_NONE            (0<<20)
//#define   AAI_CFG_VOICE_SYNC_PCM1            (1<<20)
//#define   AAI_CFG_VOICE_SYNC_PCM2            (2<<20)
  #define   AAI_CFG_SYNC_PCM                 (1<<20)
//#define   AAI_CFG_PCM_SYNC_SOFT              (0<<22)
//#define   AAI_CFG_PCM_SYNC_PCM1              (1<<22)
//#define   AAI_CFG_PCM_SYNC_PCM2              (2<<22)
  #define   AAI_CFG_BIN_ACCEPT               (1<<22)
#define   AAI_CFG_BINARY_RUN                 (1<<23)
#define   AAI_CFG_BINARY_REJECT_00           (0<<24)
#define   AAI_CFG_BINARY_REJECT_01           (1<<24)
#define   AAI_CFG_BINARY_REJECT_10           (2<<24)
#define   AAI_CFG_BINARY_REJECT_11           (3<<24)
//#define   AAI_CFG_RUN_PCM0                   (1<<24)
//#define   AAI_CFG_PCM0_16K                   (1<<25)
//#define   AAI_CFG_PCM0_MAST                  (1<<26)
//#define   AAI_CFG_RUN_PCM1                   (1<<27)
//#define   AAI_CFG_PCM1_16K                   (1<<28)
//#define   AAI_CFG_PCM1_MAST                  (1<<29)
  #define   AAI_CFG_EXT_ADC0                   (1<<27)
  #define   AAI_CFG_NOISE_MODE                 (1<<28)
  #define   AAI_CFG_NOISE_SYNC                 (1<<29)
  #define   AAI_CFG_NOISE_SIZE_4               (1<<30)
  #define   AAI_CFG_NOISE_SIZE_8               (2<<30)
  #define   AAI_CFG_NOISE_SIZE_16              (3<<30)

// Interrupt enable register (AAI_ITEN @0x324)
#define   AAI_ITEN_OUT0                      (1<< 0)
#define   AAI_ITEN_OUT1                      (1<< 1)
#define   AAI_ITEN_16KHZ                     (1<< 2)
#define   AAI_ITEN_8KHZ                      (1<< 3)
//#define   AAI_ITEN_AUX                       (1<< 4)
//#define   AAI_ITEN_ICH1                      (1<< 5)
#define   AAI_ITEN_16KHZ_ICH0                (1<< 6)
#define   AAI_ITEN_8KHZ_ICH0                 (1<< 7)
//#define   AAI_ITEN_16KHZ_ICH2                (1<< 8)
//#define   AAI_ITEN_8KHZ_ICH2                 (1<< 9)
  #define   AAI_ITEN_ICH2                      (1<< 8)
//#define   AAI_ITEN_ICH3                      (1<<10)
  #define   AAI_ITEN_ADC_8KHZ                  (1<<10)
#define   AAI_ITEN_16KHZ_BACK                (1<<11)
#define   AAI_ITEN_8KHZ_BACK                 (1<<12)
//#define   AAI_ITEN_SO1_PCM0                  (1<<13)
//#define   AAI_ITEN_SI1_PCM0                  (1<<14)
//#define   AAI_ITEN_SO2_PCM0                  (1<<15)
//#define   AAI_ITEN_SI2_PCM0                  (1<<16)
//#define   AAI_ITEN_SO3_PCM0                  (1<<17)
//#define   AAI_ITEN_SI3_PCM0                  (1<<18)
//#define   AAI_ITEN_SO1_PCM1                  (1<<19)
//#define   AAI_ITEN_SI1_PCM1                  (1<<20)
//#define   AAI_ITEN_SO2_PCM1                  (1<<21)
//#define   AAI_ITEN_SI2_PCM1                  (1<<22)
  #define   AAI_ITEN_SO1_PCM                   (1<<13)
  #define   AAI_ITEN_SI1_PCM                   (1<<14)
  #define   AAI_ITEN_SO2_PCM                   (1<<15)
  #define   AAI_ITEN_SI2_PCM                   (1<<16)
#define   AAI_ITEN_SRC_OUT                   (1<<23)
//#define   AAI_ITEN_AUX_SEL                   (1<<24)
  #define   AAI_ITEN_SRC_RAM                   (1<<24)
#define   AAI_ITEN_BINARY                    (1<<25)
#define   AAI_ITEN_ICH2_SEL_AC_IN2           (0<<26)
#define   AAI_ITEN_ICH2_SEL_AC_IN3           (1<<26)
#define   AAI_ITEN_ICH2_SEL_AC_IN4           (2<<26)
#define   AAI_ITEN_ICH2_SEL_SPDIF            (3<<26)
//#define   AAI_ITEN_IN3_SEL_1                 (0<<28)
//#define   AAI_ITEN_IN3_SEL_2                 (1<<28)
//#define   AAI_ITEN_IN3_SEL_3                 (2<<28)
//#define   AAI_ITEN_IN3_SEL_MOST              (3<<28)
//#define   AAI_ITEN_ASYNC_I2S_IN2_ONLY        (0<<30)
//#define   AAI_ITEN_ASYNC_I2S_IN3_ONLY        (1<<30)
//#define   AAI_ITEN_ASYNC_I2S_IN2_BOTH        (2<<30)
//#define   AAI_ITEN_ASYNC_I2S_IN3_BOTH        (3<<30)
  #define   AAI_ITEN_ADC_TO_CH2                (1<<28)
  #define   AAI_ITEN_ADC_USB                   (1<<30)
  #define   AAI_ITEN_DAC_USB                   (1<<31)

// Interrupt status register (AAI_ITS @0x328)
#define   AAI_ITS_OUT0                       (1<< 0)
#define   AAI_ITS_OUT1                       (1<< 1)
#define   AAI_ITS_16KHZ                      (1<< 2)
#define   AAI_ITS_8KHZ                       (1<< 3)
//#define   AAI_ITS_AUX                        (1<< 4)
//#define   AAI_ITS_ICH1                       (1<< 5)
#define   AAI_ITS_MUSIC_ICH0                 (1<< 6)
#define   AAI_ITS_16KHZ_ICH0                 (1<< 7)
#define   AAI_ITS_8KHZ_ICH0                  (1<< 8)
#define   AAI_ITS_MUSIC_ICH2                 (1<< 9)
//#define   AAI_ITS_16KHZ_ICH2                 (1<<10)
  #define   AAI_ITS_ADC_8KHZ                   (1<<10)
//#define   AAI_ITS_8KHZ_ICH2                  (1<<11)
//#define   AAI_ITS_ICH3                       (1<<12)
#define   AAI_ITS_MUSIC_BACK                 (1<<13)
#define   AAI_ITS_16KHZ_BACK                 (1<<14)
#define   AAI_ITS_8KHZ_BACK                  (1<<15)
#define   AAI_ITS_SO1_PCM                    (1<<16)
#define   AAI_ITS_SI1_PCM                    (1<<17)
#define   AAI_ITS_SO2_PCM                    (1<<18)
#define   AAI_ITS_SI2_PCM                    (1<<19)
//#define   AAI_ITS_SO3_PCM0                   (1<<20)
//#define   AAI_ITS_SI3_PCM0                   (1<<21)
//#define   AAI_ITS_SO1_PCM1                   (1<<22)
//#define   AAI_ITS_SI1_PCM1                   (1<<23)
//#define   AAI_ITS_SO2_PCM1                   (1<<24)
//#define   AAI_ITS_SI2_PCM1                   (1<<25)
  #define   AAI_ITS_MONITOR                    (1<<23)
  #define   AAI_ITS_ALARM                      (1<<24)
  #define   AAI_ITS_ULTRA                      (1<<25)
#define   AAI_ITS_SPDIF_BIN                  (1<<26)
  #define   AAI_ITS_DMA_ERROR                  (1<<27)
  #define   AAI_ITS_SAMP_OUT                   (1<<28)
  #define   AAI_ITS_SAMP_IN                    (1<<29)
  #define   AAI_ITS_ADC_USB                    (1<<30)
  #define   AAI_ITS_DAC_USB                    (1<<31)

// I2S format register (AAI_I2S_FORMAT @0x370)
#define   AAI_I2S_FORMAT_AUX_MATSU           (1<< 0)
#define   AAI_I2S_FORMAT_AUX_RIGHTFRAME      (1<< 1)
#define   AAI_I2S_FORMAT_AUX_RIGHTJUST       (1<< 2)
#define   AAI_I2S_FORMAT_AUX_LSBFIRST        (1<< 3)
#define   AAI_I2S_FORMAT_0_MATSU             (1<< 4)
#define   AAI_I2S_FORMAT_0_RIGHTFRAME        (1<< 5)
#define   AAI_I2S_FORMAT_0_RIGHTJUST         (1<< 6)
#define   AAI_I2S_FORMAT_0_LSBFIRST          (1<< 7)
#define   AAI_I2S_FORMAT_1_MATSU             (1<< 8)
#define   AAI_I2S_FORMAT_1_RIGHTFRAME        (1<< 9)
#define   AAI_I2S_FORMAT_1_RIGHTJUST         (1<<10)
#define   AAI_I2S_FORMAT_1_LSBFIRST          (1<<11)
#define   AAI_I2S_FORMAT_2_MATSU             (1<<12)
#define   AAI_I2S_FORMAT_2_RIGHTFRAME        (1<<13)
#define   AAI_I2S_FORMAT_2_RIGHTJUST         (1<<14)
#define   AAI_I2S_FORMAT_2_LSBFIRST          (1<<15)
#define   AAI_I2S_FORMAT_ASYNC1              (1<<16)
#define   AAI_I2S_FORMAT_ASYNC2              (1<<17)
#define   AAI_I2S_FORMAT_ASYNC3              (1<<18)
//#define   AAI_I2S_FORMAT_SPDIF_PAD           (1<<19)
//#define   AAI_I2S_FORMAT_IN2_11              (0<<20)
//#define   AAI_I2S_FORMAT_IN2_12              (1<<20)
//#define   AAI_I2S_FORMAT_IN2_13              (2<<20)
//#define   AAI_I2S_FORMAT_IN2_1SPDIF          (3<<20)
//#define   AAI_I2S_FORMAT_IN2_21              (0<<22)
//#define   AAI_I2S_FORMAT_IN2_22              (1<<22)
//#define   AAI_I2S_FORMAT_IN2_23              (2<<22)
//#define   AAI_I2S_FORMAT_IN2_2SPDIF          (3<<22)
//#define   AAI_I2S_FORMAT_IN2_31              (0<<24)
//#define   AAI_I2S_FORMAT_IN2_32              (1<<24)
//#define   AAI_I2S_FORMAT_IN2_33              (2<<24)
//#define   AAI_I2S_FORMAT_IN2_3SPDIF          (3<<24)
//#define   AAI_I2S_FORMAT_IN3_11              (0<<26)
//#define   AAI_I2S_FORMAT_IN3_12              (1<<26)
//#define   AAI_I2S_FORMAT_IN3_13              (2<<26)
//#define   AAI_I2S_FORMAT_IN3_1SPDIF          (3<<26)
//#define   AAI_I2S_FORMAT_IN3_21              (0<<28)
//#define   AAI_I2S_FORMAT_IN3_22              (1<<28)
//#define   AAI_I2S_FORMAT_IN3_23              (2<<28)
//#define   AAI_I2S_FORMAT_IN3_2SPDIF          (3<<28)
//#define   AAI_I2S_FORMAT_IN3_31              (0<<30)
//#define   AAI_I2S_FORMAT_IN3_32              (1<<30)
//#define   AAI_I2S_FORMAT_IN3_33              (2<<30)
//#define   AAI_I2S_FORMAT_IN3_3SPDIF          (3<<30)

//// PCM0 config (AAI_PCM0_CONF @0x374)
//#define   AAI_PCM0_CONF_SHIFT2               (1<< 0)
//#define   AAI_PCM0_CONF_SHIFT3               (1<< 8)
//#define   AAI_PCM0_CONF_OKI                  (1<<16)
//#define   AAI_PCM0_CONF_RUN                  (1<<17)
//#define   AAI_PCM0_CONF_MAST                 (1<<18)
//#define   AAI_PCM0_CONF_1MHZ                 (1<<19)
//#define   AAI_PCM0_CONF_16K                  (1<<20)
//#define   AAI_PCM0_CONF_HIGH                 (1<<21)
//#define   AAI_PCM0_CONF_STOP                 (1<<22)
//#define   AAI_PCM0_CONF_PLL_LOCKED           (1<<23)
//// PCM1 config (AAI_PCM1_CONF @0x378)
//#define   AAI_PCM1_CONF_SHIFT2               (1<< 0)
//#define   AAI_PCM1_CONF_OKI                  (1<< 8)
//#define   AAI_PCM1_CONF_RUN                  (1<< 9)
//#define   AAI_PCM1_CONF_MAST                 (1<<10)
//#define   AAI_PCM1_CONF_1MHZ                 (1<<11)
//#define   AAI_PCM1_CONF_16K                  (1<<12)
//#define   AAI_PCM1_CONF_HIGH                 (1<<13)
//#define   AAI_PCM1_CONF_STOP                 (1<<14)
//#define   AAI_PCM1_CONF_PLL_LOCKED           (1<<15)
// PCM config (AAI_PCM_CONF @0x374)
#define   AAI_PCM_CONF_SHIFT2               (255<< 0)
#define   AAI_PCM_CONF_CYCLES_HIGH          (255<< 8)
#define   AAI_PCM_CONF_CYCLES_LOW           (511<<16)
#define   AAI_PCM_CONF_OKI                  (1<<25)
#define   AAI_PCM_CONF_RUN                  (1<<26)
#define   AAI_PCM_CONF_MAST                 (1<<27)
#define   AAI_PCM_CONF_1MHZ                 (1<<28)
#define   AAI_PCM_CONF_16K                  (1<<29)
#define   AAI_PCM_CONF_STOP                 (1<<30)
#define   AAI_PCM_CONF_PLL_LOCKED           (1<<31)

// DMA control (AAI_DMA_CTRL @0x37C)
#define   AAI_DMA_CTRL_OUT0                  (1<< 0)
#define   AAI_DMA_CTRL_OUT1                  (1<< 1)
#define   AAI_DMA_CTRL_16KHZ                 (1<< 2)
#define   AAI_DMA_CTRL_8KHZ                  (1<< 3)
//#define   AAI_DMA_CTRL_AUX                   (1<< 4)
//#define   AAI_DMA_CTRL_ICH1                  (1<< 5)
#define   AAI_DMA_CTRL_ICH0                  (1<< 6)
#define   AAI_DMA_CTRL_ICH2                  (1<< 7)
//#define   AAI_DMA_CTRL_ICH3                  (1<< 8)
#define   AAI_DMA_CTRL_BACK                  (1<< 9)
//#define   AAI_DMA_CTRL_PCM0                  (1<<10)
#define   AAI_DMA_CTRL_ULTRA                  (1<<11)
  #define   AAI_DMA_CTRL_PCM                   (1<<10)
//#define   AAI_DMA_CTRL_NO_INT_AUX            (1<<12)
#define   AAI_DMA_CTRL_OUT0_INTERLEAVE       (1<<13)
#define   AAI_DMA_CTRL_OUT1_INTERLEAVE       (1<<14)
  #define   AAI_DMA_CTRL_RAM_SAMPLE_RATE   (1<<15)
  #define   AAI_DMA_CTRL_ADC_USB           (1<<16)
  #define   AAI_DMA_CTRL_DAC_USB           (1<<17)
  #define   AAI_DMA_CTRL_DAC_24_BITS           (1<<18)
  #define   AAI_DMA_CTRL_ADC_8KHZ          (1<<19)

// Loudness Control register (AAI_LOUD_CTRL @0x390)
  #define   AAI_LOUD_CTRL_OFF                  (1<< 0)

// Channel2 Input Configuration register (AAI_CHANNEL2_CONFIG @0x394)
  #define   AAI_CH2_CFG_AC_IN2_NUM             (1<< 1)
  #define   AAI_CH2_CFG_AC_IN3_NUM             (1<< 9)
  #define   AAI_CH2_CFG_AC_IN4_NUM             (1<<17)
  #define   AAI_CH2_CFG_ASYNC_FRAME2_SD0_DAT3  (0<<24)
  #define   AAI_CH2_CFG_ASYNC_FRAME2_I2C1_SDA  (1<<24)
  #define   AAI_CH2_CFG_ASYNC_FRAME2_SD0_DAT2  (2<<24)
  #define   AAI_CH2_CFG_ASYNC_FRAME2_I2C1_SCL  (3<<24)
  #define   AAI_CH2_CFG_ASYNC_FRAME3_SD0_DAT3  (0<<26)
  #define   AAI_CH2_CFG_ASYNC_FRAME3_I2C1_SDA  (1<<26)
  #define   AAI_CH2_CFG_ASYNC_FRAME3_I2S_IN2   (2<<26)
  #define   AAI_CH2_CFG_ASYNC_FRAME3_I2C1_SCL  (3<<26)
  #define   AAI_CH2_CFG_ASYNC_FRAME4_I2C1_SDA  (0<<28)
  #define   AAI_CH2_CFG_ASYNC_FRAME4_SD0_DAT3  (1<<28)
  #define   AAI_CH2_CFG_ASYNC_FRAME4_I2S_IN2   (2<<28)
  #define   AAI_CH2_CFG_ASYNC_FRAME4_SD0_DAT2  (3<<28)
  #define   AAI_CH2_CFG_SPDIF_SOURCE_I2S_IN1   (0<<30)
  #define   AAI_CH2_CFG_SPDIF_SOURCE_SD0_DAT3  (1<<30)
  #define   AAI_CH2_CFG_SPDIF_SOURCE_I2C1_SCL  (2<<30)
  #define   AAI_CH2_CFG_SPDIF_SOURCE_I2C1_SDA  (3<<30)

// ADC configuration register (AAI_ADC_MODES @0x39C)
  #define   AAI_ADC_MODES_8KHZ_MODE            (1<< 0)
  #define   AAI_ADC_MODES_MONITORING           (1<< 1)
  #define   AAI_ADC_MODES_ULTRA_SOUNDS         (1<< 2)
  #define   AAI_ADC_MODES_BATTERY_ALARM        (1<< 3)
  #define   AAI_ADC_MODES_PERIODIC             (1<< 4)
  #define   AAI_ADC_MODES_8KHZ_IN              (7<< 5)
  #define   AAI_ADC_MODES_ULTRA_IN             (7<< 8)
  #define   AAI_ADC_MODES_BATTERY_IN           (7<<11)

// ADC battery limits (AAI_ADC_LIMITS @0x3A8)
  #define   AAI_ADC_LIMITS_BATTERY_MIN         (1023<< 0)
  #define   AAI_ADC_LIMITS_BATTERY_MAX         (1023<<16)

// DMA IT ack (AAI_DMA_INT_ACK @0x600)
#define   AAI_DMA_INT_ACK_OUT0               (1<< 0)
#define   AAI_DMA_INT_ACK_OUT1               (1<< 1)
#define   AAI_DMA_INT_ACK_16KHZ              (1<< 2)
#define   AAI_DMA_INT_ACK_8KHZ               (1<< 3)
//#define   AAI_DMA_INT_ACK_AUX                (1<< 4)
//#define   AAI_DMA_INT_ACK_ICH1               (1<< 5)
#define   AAI_DMA_INT_ACK_MUSIC_ICH0         (1<< 6)
#define   AAI_DMA_INT_ACK_16KHZ_ICH0         (1<< 7)
#define   AAI_DMA_INT_ACK_8KHZ_ICH0          (1<< 8)
#define   AAI_DMA_INT_ACK_MUSIC_ICH2         (1<< 9)
//#define   AAI_DMA_INT_ACK_16KHZ_ICH2         (1<<10)
//#define   AAI_DMA_INT_ACK_8KHZ_ICH2          (1<<11)
//#define   AAI_DMA_INT_ACK_ICH3               (1<<12)
  #define   AAI_DMA_INT_ACK_ADC_8KHZ           (1<<10)
#define   AAI_DMA_INT_ACK_MUSIC_BACK         (1<<13)
#define   AAI_DMA_INT_ACK_16KHZ_BACK         (1<<14)
#define   AAI_DMA_INT_ACK_8KHZ_BACK          (1<<15)
#define   AAI_DMA_INT_ACK_SO1_PCM            (1<<16)
#define   AAI_DMA_INT_ACK_SI1_PCM            (1<<17)
#define   AAI_DMA_INT_ACK_SO2_PCM            (1<<18)
#define   AAI_DMA_INT_ACK_SI2_PCM            (1<<19)
//#define   AAI_DMA_INT_ACK_SO3_PCM0           (1<<20)
//#define   AAI_DMA_INT_ACK_SI3_PCM0           (1<<21)
//#define   AAI_DMA_INT_ACK_SO1_PCM1           (1<<22)
//#define   AAI_DMA_INT_ACK_SI1_PCM1           (1<<23)
//#define   AAI_DMA_INT_ACK_SO2_PCM1           (1<<24)
//#define   AAI_DMA_INT_ACK_SI2_PCM1           (1<<25)
  #define   AAI_DMA_INT_ACK_ULTRA              (1<<25)
  #define   AAI_DMA_INT_ACK_BIN_SPDIF          (1<<26)
//#define   AAI_DMA_INT_ACK_ERROR              (1<<26)
  #define   AAI_DMA_INT_ACK_ERROR              (1<<27)
  #define   AAI_DMA_INT_ACK_SAMP_OUT           (1<<28)
  #define   AAI_DMA_INT_ACK_SAMP_IN            (1<<29)
  #define   AAI_DMA_INT_ACK_ADC_USB            (1<<30)
  #define   AAI_DMA_INT_ACK_DAC_USB            (1<<31)


//////////////////////////////////////////////////////////////////////////////////////////////
// Registers reset values
//////////////////////////////////////////////////////////////////////////////////////////////

#define   AAI_RST_SRC                   0x01000000
#define   AAI_RST_CH1_VOL               0xffffff00
#define   AAI_RST_PCM_CONF              0x40402000
#define   AAI_RST_CH2_CFG               0x00404040
#define   AAI_RST_ADC_LIMITS            0x03ff0000

#define   AAI_RST_OTHERS                0x00000000


//////////////////////////////////////////////////////////////////////////////////////////////
// Registers masks definitions
//////////////////////////////////////////////////////////////////////////////////////////////

#define   AAI_MSK_SRC                   0x01ffffff
#define   AAI_MSK_LOUD_CTRL             0x00000001
#define   AAI_MSK_I2S_FORMAT            0x0007ffff
#define   AAI_MSK_ADC_MODES             0x00003ff1
#define   AAI_MSK_ADC_LIMITS            0x03ff03ff
#define   AAI_MSK_DMA_CTRL              0x000fe6cf
#define   AAI_MSK_MUSIC_DMA_COUNT       0x00000007
#define   AAI_MSK_VOICE_DMA_COUNT       0x00000003
#define   AAI_MSK_ADD                   0xfffffffc
#define   AAI_MSK_BINARY_DMA_COUNT      0x00000007
#define   AAI_MSK_SAMP_DMA_COUNT        0x00000007
#define   AAI_MSK_NOISE_DMA_COUNT       0x00000007
#define   AAI_MSK_ADC_USB_DMA_COUNT     0x00000007
#define   AAI_MSK_DAC_USB_DMA_COUNT     0x00000007
#define   AAI_MSK_ULTRA_DMA_COUNT       0x00000007
#define   AAI_MSK_DMA_INT_ACK           0xffffffff

#define   AAI_MSK_32                    0xffffffff


//////////////////////////////////////////////////////////////////////////////////////////////
// read/write macros
//////////////////////////////////////////////////////////////////////////////////////////////

//! write to an AAI register
#define aai_write_reg( _reg_, _value_ ) \
 (*((volatile U32 *)(P6_AAI +(_reg_))) = (U32)(_value_))

//! read an AAI register
#define aai_read_reg(_reg_ ) \
 (*((volatile U32 *)(P6_AAI+(_reg_))))


