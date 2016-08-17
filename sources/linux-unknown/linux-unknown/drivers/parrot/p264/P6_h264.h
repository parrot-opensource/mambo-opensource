#include "P6_h264_reg.h"

// registers definitions + useful macro

// Registers bitwise definitions
// Status register
#define DCT_STATUS_END_OK    (1<<0)        // DCT Done
//#define DCT_STATUS_ERROR     (1<<1)        // DCT Error ?

// Interrupt enable register
#define DCT_ITEN_END_OK      (1<<0)        // IT Done enable
//#define DCT_ITEN_ERROR       (1<<1)        // IT Error enable ?

// Interrupt Acknowledge register
#define DCT_ITACK_END_OK     (1<<0)        // IT Done acknowledge
//#define DCT_ITACK_ERROR      (1<<1)        // IT Error acknowledge ?

// DCT control mode (forward or inverse dct)
#define DCT_CTRLMODE_FDCT     0
#define DCT_CTRLMODE_IDCT     1

typedef enum {
  DCT_DMA_INCR    = 0,                    //!<  4 bytes DMA burst
  DCT_DMA_INCR4   = 1,                    //!< 16 bytes DMA burst
  DCT_DMA_INCR8   = 2,                    //!< 32 bytes DMA burst
  DCT_DMA_INCR16  = 3,                    //!< 64 bytes DMA burst
} DCT_DMA_BURST_MODE;

// ME values
#define ME_ALGO_DIS_MVP_INTRA_PRED    (1<<5)  // diasable intra pred and inter MV pred
#define ME_ALGO_I_FRAME               (1<<4)  // configure ME to encode en I frame
#define ME_ALGO_P_FRAME               (0<<4)  // configure ME to encode en I frame

#define ME_ANAL_4x4              (1<<22)
#define ME_ANAL_4x8              (1<<21)
#define ME_ANAL_8x4              (1<<20)
#define ME_ANAL_8x8              (1<<19)
#define ME_ANAL_8x16             (1<<18)
#define ME_ANAL_16x8             (1<<17)
#define ME_ANAL_16x16            (1<<16)


#define ME_PAT_SMALL_DIAMOND    0x08
#define ME_PAT_BIG_DIAMOND      0x09
#define ME_PAT_SMALL_SQUARE     0x0A
#define ME_PAT_BIG_SQUARE       0x0B

#define IS_INTRA_4x4(a)   ((a&0x1F) == 0)
#define IS_INTRA_16x16(a) ((a&0x1F) != 0)
#define CHROMA_MODE(a)    ((a>>16)&0x03)
#define INTRA_16x16_MODE(a) ((a&0x1F)-1 )
#define INTER_16_8_PARTITION(a) (a&0x1F)

#define I_MB_COMPLETION_FLAG 0x0010101
#define I_MB_COMPLETION_MASK 0x00FFFFF
#define P_MB_COMPLETION_FLAG 0x1010101
#define P_MB_COMPLETION_MASK 0xFFFFFFF
