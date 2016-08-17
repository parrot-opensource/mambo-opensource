////////////////////////////////////////////////////
// Parrot proprietary DCT registers
////////////////////////////////////////////////////

// Parrot DCT address: 0xD00B0000
#define P6_BASE_ADDRESS         0xD00B0000

// H264 general registers
#define H264_STATUS             0x000       // Status Register
#define H264_ITEN               0x004       // Interrupt Enable Register
#define H264_ITACK              0x008       // Interrupt Acknowledge Register
#define H264_DMA                0x010       // Dma Register
#define H264_DMAINT		0x02C
#define H264_RESET		0x03C
#define H264_START		0x00C
#define H264_CONFIG		0x028
#define H264_LINESIZE           0x01C
#define H264_FRAMESIZE          0x020
#define H264_MB_ADDR            0x024
#define H264_QP                 0x018       // ME/MC Y/CC quantizer

// DCT registers
#define DCT_CONTROL             0x040       // Control Register
#define DCT_ORIG_Y_ADDR         0x044       // Address Register
#define DCT_ORIG_CU_ADDR        0x048       // Address Register
#define DCT_ORIG_CV_ADDR        0x04C       // Address Register
#define DCT_DEST_Y_ADDR         0x050       // Address Register
#define DCT_DEST_CU_ADDR        0x054       // Address Register
#define DCT_DEST_CV_ADDR        0x058       // Address Register
#define DCT_LINEOFFSET          0x05C       // Line size
#define DCT_Q_ADDR		0x064		// quantization table
//#define DCT_DEBUG               0x030?       // Debug register
//#define DCT_SIGNATURE           0x034?       // Signature Register





// ME registers
#define ME_ALGORITHM             0x100     // I/P frame choice, disable/enable pred_intra
#define ME_RESULT                0x148     // in I-frame mode : intra mode result (intra 16x16 mod0/1/2/3 or intra 4x4)
#define ME_CMB_FRAME_ADDRY       0x11C     // input frame (to be encoded) Y addr
#define ME_CMB_FRAME_ADDRCU      0x120     // input frame (to be encoded) U addr
#define ME_CMB_FRAME_ADDRCV      0x124     // input frame (to be encoded) V addr
#define ME_IPRED0                0x16C     // intra4*4 pred mode results (first 8 4*4 blocks)
#define ME_IPRED1                0x170     // intra4*4 pred mode results (last 8 4*4 blocks)
#define ME_ANALYSIS              0x118     // define MB split pattern (P-frame)
#define ME_PAT_LIST              0x104     // define/enable search patterns (P-frame)
#define ME_PAT_RESIZE            0x108     //
#define ME_SW_FRAME_ADDRY        0x128     // luma reference frame for P-frame
#define ME_SW_FRAME_ADDRCC       0x12C     // chroma reference frame for P-frame
#define ME_SW_CONFIG             0x140     // search window size
#define ME_PRED_BASE_REG         0x14C     // MV result
#define ME_RD                    0x114

//DEB registers
#define DEB_ME_FRAME_ADDRY       0x0A0     // deb reconstructed picture (reference)
#define DEB_ME_FRAME_ADDRCC      0x0A4     // deb reconstructed picture (reference)
#define DEB_ME_TMP_ADDR          0x0A8
#define DEB_MC_TMP_ADDR          0x0AC
#define DEB_CONFIG               0x080
#define DEB_PARAM                0x084
#define DEB_QPS_LEFT             0x08C
#define DEB_QPS_TOP              0x090

//MC registers
#define MC_MB_INFO               0x200


