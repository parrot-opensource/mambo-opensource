/**
********************************************************************************
* @file h264_p6_ioctl.c
* @brief p6 p264 driver
*
* Copyright (C) 2011 Parrot S.A.
*
* @author     Pierre ELINE <pierre.eline@parrot.com>
* @date       2011-02-20
********************************************************************************
*/

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <mach/parrot.h>
#include <asm/fiq.h>
#include <asm/irq.h>

#include "p264_p6_ioctl.h"
#include "P6_h264.h"

// Memory Usage :
// This module makes the assumption that INTRAM is free at 0x80007000 for 8*ceil(MAX_FRAME_WIDTH(in MBs)/8)*16*4*2 = 3072 bytes
// Input and output frame buffers are allocated in user space with dmaalloc, only physical address are transmitted to the module
// Typical dmaalloc ram usage :
//     -  1 input YUV frame of WIDTH*HEIGHT*3/2 bytes
//     -  1 reference frames of WIDTH*HEIGHT*3/2 bytes for P-encoding
//     -  1 deblocking filtered frame of WIDTH*HEIGHT*3/2 bytes
//     -  1 output dct buffer of WIDTH*HEIGHT/16/16*864
// This module also allocates (in ram) a buffer of MAX_P264_MB*sizeof(p264_p6_raw_reg_results_t) bytes to store MB data such as Motion Vector, Intra coding type ...


/* our whitelist h264 include */
#define P264_DRIVER_NAME "p264_p6 driver"
#define MAX_WIDTH   352
#define MAX_HEIGHT  288
#define MAX_P264_MB  MAX_WIDTH*MAX_HEIGHT/16/16

static struct fiq_handler fh =
{
    .name = "p264_p6_fiq"
};
struct pt_regs regs;

static bool opened = false;
static char __iomem *ui_h264_reg;
static char __iomem *ui_sysc_reg;

// h264 picture parameters
static uint8_t* current_Y;
static uint8_t* current_Cb;
static uint8_t* current_Cr;
static uint32_t current_linesize;
static bool I_encoding;

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

static void dump_picture_encoding_context(picture_encoding_context_t *pec)
{
  printk("pec dump\n");
  printk("pec->width %d\n",pec->current_width);
  printk("pec->height %d\n",pec->current_height);
  printk("pec->current_i_MB %d\n",pec->current_i_MB);
  printk("pec->current_j_MB %d\n",pec->current_j_MB);
  printk("pec->nb_mb_to_encode %d\n",pec->nb_mb_to_encode);
  printk("pec->nb_mb_encoded %d\n",pec->nb_mb_encoded);
  printk("pec->nb_mb_user %d\n",pec->nb_mb_user);
  printk("pec->is_running %d\n",pec->is_running);
}

static picture_encoding_context_t picture_encoding_context;

static uint8_t* h264_ref_frame;  // frame decoded by encoder as a reference
static uint8_t* h264_deb_frame;  // frame decoded by encoder as a reference

static p264_p6_raw_reg_results_t* user_reg_output=NULL;
static p264_p6_raw_reg_results_t* fiq_reg_output=NULL;

static int init_h264_ip(void)
{
  // active h264 clock
  uint32_t value;
  value = __raw_readl(ui_sysc_reg + 0x0C);
  __raw_writel(value | (1<<4), ui_sysc_reg + 0x0C);

  // reset IP
  __raw_writel(0x1, ui_h264_reg+H264_RESET);

  // printk used as a delay to wait for IP RESET completion
  printk("p264 driver : reset h264 IP\n");

  // config h264
  //__raw_writel( 0, ui_h264_reg+H264_ITEN ); // no interrupt
  __raw_writel( 0x100, ui_h264_reg+H264_ITEN ); // DEB_INT enabled

  __raw_writel( 0x33, ui_h264_reg+H264_DMA);
  __raw_writel(0x33, ui_h264_reg+H264_DMAINT);

  __raw_writel((1<<18)|(1<<13)|(1<<11)|(1<<9)|(1<<7), ui_h264_reg+H264_CONFIG); // ME_noDEB | MEtoDEB_EN | MEtoDCT_EN | IS_H264 | MEuseMCforCC

  // configure MB split for P frame
  __raw_writel(ME_ANAL_16x16,ui_h264_reg+ME_ANALYSIS); // Authorize only 16x16 block motion

  // configure search patterns for P frames
  __raw_writel((ME_PAT_BIG_SQUARE<<28)|(ME_PAT_BIG_SQUARE<<24)|(ME_PAT_BIG_SQUARE<<20)|(ME_PAT_BIG_SQUARE<<16)|(ME_PAT_BIG_SQUARE<<12)|(ME_PAT_BIG_SQUARE<<8)|(ME_PAT_BIG_SQUARE<<4)|(ME_PAT_BIG_SQUARE<<0),ui_h264_reg+ME_PAT_LIST); // set big square

  __raw_writel(0x00000022,ui_h264_reg+ME_PAT_RESIZE); // no subpixelix mv on luma

  __raw_writel(0x80007000,ui_h264_reg+DEB_ME_TMP_ADDR ); // MUST BE INTRAM


  // init local variable
  // h264 picture parameters
  current_Y  = NULL;
  current_Cb = NULL;
  current_Cr = NULL;
  current_linesize = 0;
  picture_encoding_context.current_width = 0;
  picture_encoding_context.current_height = 0;
  h264_deb_frame = NULL;
  h264_ref_frame = NULL;

  printk("p264 driver : initialized\n");
  return 0;
}


static int p264_open(struct inode *inode, struct file *filp)
{
    if (opened == false)
    {
      filp->private_data = NULL;
      opened = true;
      init_h264_ip();
      return 0;
    }
    else
      return -EBUSY;

}

static int p264_release(struct inode *inode, struct file *filp)
{
    if (opened == true)
    {
      filp->private_data = NULL;
      opened = false;
      return 0;
    }
    else
      return -EBUSY;
}



static int p264_ioctl(struct inode *inode, struct file *filp,
        unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    void __user *arg_struct = (void __user *)arg;

    // test whether fiq is done
    if (picture_encoding_context.nb_mb_to_encode == picture_encoding_context.nb_mb_encoded)
      picture_encoding_context.is_running = false;

    switch (cmd)
    {
        case P264_SET_DIM:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t dim,width,height;
            get_user(dim, (uint32_t __user *) arg);
            width = dim&0x0000FFFF;
            height = dim>>16;
            if (width<=MAX_WIDTH && height<=MAX_HEIGHT)
            {
              picture_encoding_context.current_width =  width;
              picture_encoding_context.current_height = height;
              printk("p264 driver : P264_SET_DIM set image dimensions (%d,%d)\n",picture_encoding_context.current_width,picture_encoding_context.current_height);
              // set ME&MC frame line size
              __raw_writel((picture_encoding_context.current_width<<16)|picture_encoding_context.current_width,ui_h264_reg+H264_LINESIZE);
              // set ME&MC frame dimension
              __raw_writel(((picture_encoding_context.current_height>>4)<<24)|((picture_encoding_context.current_width>>4)<<16)|((picture_encoding_context.current_height>>4)<<8)|((picture_encoding_context.current_width>>4)<<0),ui_h264_reg+H264_FRAMESIZE);
            }
            else
            {
              printk("p264 driver : P264_SET_DIM error, resolution too large (%d,%d)\n",width,height);
            }
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_DIM error, IP still processing...\n");
          }
          break;
        }

        case P264_SET_INPUT_BUF:
        {
          if (picture_encoding_context.is_running == false)
          {
            p264_p6_input_buf_t __user *input_buf = arg_struct;
            if (!access_ok(VERIFY_READ, input_buf, sizeof(p264_p6_input_buf_t)))
            {
              ret = -EFAULT;
              break;
            }

            // retrieve and set input buffer addr
            uint32_t addr;
            __get_user(addr, &input_buf->phys_Y);
            __raw_writel(addr,ui_h264_reg+ME_CMB_FRAME_ADDRY);
            //printk("p264 driver : set input Y 0x%x\n",addr);

            __get_user(addr, &input_buf->phys_Cb);
            __raw_writel(addr,ui_h264_reg+ME_CMB_FRAME_ADDRCU);
            //printk("p264 driver : set input Cb 0x%x\n",addr);

            __get_user(addr, &input_buf->phys_Cr);
            __raw_writel(addr,ui_h264_reg+ME_CMB_FRAME_ADDRCV);
            //printk("p264 driver : set input Cr 0x%x\n",addr);
            break;
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_INPUT_BUF error, IP still processing...\n");
          }
        }

        case P264_SET_OUTPUT_BUF:
        {
          if (picture_encoding_context.is_running == false)
          {
            p264_p6_output_buf_t __user *output = arg_struct;
            if (!access_ok(VERIFY_READ, output, sizeof(p264_p6_output_buf_t)))
            {
              ret = -EFAULT;
              break;
            }

            // retrieve and set ouput buffer addr
            uint32_t addr;
            __get_user(addr, &output->phys_output);
            __raw_writel(addr,ui_h264_reg+DCT_DEST_Y_ADDR);

            __get_user(user_reg_output , &output->reg_output);
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_OUTPUT_BUF error, IP still processing...\n");
          }
          break;
        }

        case P264_SET_FRAME_TYPE:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t I_type;
            get_user(I_type, (uint32_t __user *) arg);
            if (I_type == 0)
            {
              //printk ("p264 driver : NEW I FRAME\n");
              I_encoding = true;
              //uiomap_writel(&ui_h264_reg,ME_ALGO_I_FRAME|ME_ALGO_DIS_MVP_INTRA_PRED,ME_ALGORITHM); // configure ME to I-frame, don't use pred intra prediction
              __raw_writel(ME_ALGO_I_FRAME,ui_h264_reg+ME_ALGORITHM); // configure ME to I-frame, use pred intra prediction
            }
            else
            {
              //printk ("p264 driver : NEW P FRAME\n");
              I_encoding = false;
              //uiomap_writel(&ui_h264_reg,0x70000|ME_ALGO_DIS_MVP_INTRA_PRED|ME_ALGO_P_FRAME|0x07,ME_ALGORITHM); // configure ME to P-frame, don't use MV prediction
              __raw_writel(0x70000|ME_ALGO_P_FRAME|0x07,ui_h264_reg+ME_ALGORITHM); // configure ME to P-frame, use MV prediction
            }

            // reset current macroblock index
            picture_encoding_context.current_i_MB = 0;
            picture_encoding_context.current_j_MB = 0;

            //TODO: stop fiq

            // reset nb_mb_encoded
            picture_encoding_context.nb_mb_encoded = 0;
            picture_encoding_context.nb_mb_to_encode = 0;
            picture_encoding_context.nb_mb_user = 0;
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_FRAME_TYPE error, IP still processing...\n");
          }
          break;
        }

        case P264_SET_REF_FRAME:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t addrY,addrC;
            get_user(addrY, (uint32_t __user *) arg);
            addrC = addrY+picture_encoding_context.current_width*picture_encoding_context.current_height;
            __raw_writel(addrY,ui_h264_reg+ME_SW_FRAME_ADDRY);
            __raw_writel(addrC,ui_h264_reg+ME_SW_FRAME_ADDRCC);
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_REF_FRAME error, IP still processing...\n");
          }
          break;
        }

        case P264_SET_DEB_FRAME:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t addrY,addrC;
            get_user(addrY, (uint32_t __user *) arg);
            addrC = addrY+picture_encoding_context.current_width*picture_encoding_context.current_height;
            __raw_writel(addrY,ui_h264_reg+DEB_ME_FRAME_ADDRY);
            __raw_writel(addrC,ui_h264_reg+DEB_ME_FRAME_ADDRCC);
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_DEB_FRAME error, IP still processing...\n");
          }
          break;
        }

        case P264_ENCODE_NEXT_MB:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t nb_mb;
            get_user(nb_mb, (uint32_t __user *) arg);
            picture_encoding_context.nb_mb_to_encode += nb_mb;

            if (picture_encoding_context.nb_mb_to_encode > (picture_encoding_context.current_width>>4) * (picture_encoding_context.current_height>>4) ||
                picture_encoding_context.nb_mb_to_encode > MAX_P264_MB)
            {
               ret = -EFAULT;
               printk("p264 driver : too much MB to process %d MB\n",picture_encoding_context.nb_mb_to_encode);
            }

            // set MB index
            __raw_writel((picture_encoding_context.current_j_MB<<24)|(picture_encoding_context.current_i_MB<<16)|(picture_encoding_context.current_j_MB<<8)|picture_encoding_context.current_i_MB, ui_h264_reg+H264_MB_ADDR);
            // launch H264 IP
            __raw_writel( 0, ui_h264_reg+H264_START);

            picture_encoding_context.is_running = true;
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_ENCODE_NEXT_MB error, IP still processing...\n");
          }
          break;
        }

        case P264_SET_QP:
        {
          if (picture_encoding_context.is_running == false)
          {
            uint32_t qp;
            get_user(qp, (uint32_t __user *) arg);
            __raw_writel((qp<<24)|(qp<<16)|(qp<<8)|qp,ui_h264_reg+H264_QP);
          }
          else
          {
            ret = -EFAULT;
            printk("p264 driver : P264_SET_QP error, IP still processing...\n");
          }
          break;
        }

        case P264_WAIT_ENCODE:
        {
          uint32_t nb_user_mb=0;
          if (get_user(nb_user_mb, (unsigned int __user *) arg))
          {
             ret = -EFAULT;
          }

          // compute number of new available MB
          int32_t result;
          result = picture_encoding_context.nb_mb_encoded - picture_encoding_context.nb_mb_user;
          if (result>nb_user_mb)
            result = nb_user_mb; // return only the number of mb asked by user

          if (put_user(result, (unsigned int __user *) arg))
          {
             ret = -EFAULT;
          }

          if (!access_ok(VERIFY_READ|VERIFY_WRITE, user_reg_output, result*sizeof(p264_p6_raw_reg_results_t)))
          {
             ret = -EFAULT;
             printk("p264 driver : P264_WAIT_ENCODE wrong ouput reg addr\n");
             user_reg_output = NULL;
             break;
          }
          else if (result > 0 && user_reg_output != NULL)
          {
            // retrieve&store me_result for current MB
            uint32_t reg_copy_size = (sizeof(p264_p6_raw_reg_results_t)>>2) * (result);
            uint32_t* src = &fiq_reg_output[picture_encoding_context.nb_mb_user];
            uint32_t* dst = user_reg_output;
            while(reg_copy_size--)
            {
              put_user(*src, dst);
              src++;
              dst++;
            }
            picture_encoding_context.nb_mb_user+=result;
          }
          //dump_picture_encoding_context(&picture_encoding_context);
          break;
        }

        default:
            ret = -ENOTTY;
    }

    return ret;
}

struct file_operations p264_fops = {
    .ioctl =     p264_ioctl,
    .open =      p264_open,
    .release =   p264_release,
};

static struct miscdevice p264_miscdev = {
        .minor = 64 + 4,
        .name = "p264_p6",
        .fops = &p264_fops,
};

static int init_device(void)
{
        return misc_register(&p264_miscdev);
}

static int p264_probe(struct platform_device *pdev)
{
    extern unsigned char h264_p6_fiq_handler_start, h264_p6_fiq_handler_end;
    struct resource *res;
    printk("p264 driver : probe\n");
    int err = 0;

    err = init_device();
    if (err < 0)
        goto err;
    else
      printk("p264 driver : init device ok\n");

    /*
     * Map registers
     */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL)
    {
        printk("p264 driver : Cannot get IORESOURCE_MEM\n");
        err = -ENOENT;
        goto err;
    }

    /*
     *  alloc ouput buffer for fiq
     */
    fiq_reg_output = kzalloc(MAX_P264_MB*sizeof(p264_p6_raw_reg_results_t), GFP_KERNEL);
    if (!fiq_reg_output)
    {
      printk ("p264 driver : ouput reg alloc failed\n");
      err = -ENOENT;
      goto err;
    }

    /*
     * Install FIQ handler
     */
      if (claim_fiq(&fh) < 0)
      {
          printk("p264 driver : Couldn't claim fiq handler\n");
          err = -EBUSY;
          goto err;
      }

      set_fiq_handler(&h264_p6_fiq_handler_start,
              &h264_p6_fiq_handler_end - &h264_p6_fiq_handler_start);

    /*
     * Set registers useful in FIQ process
     */

    regs.ARM_r8  = (unsigned long)(fiq_reg_output);
    regs.ARM_r10 = (unsigned long)(&picture_encoding_context);
    set_fiq_regs(&regs);

    /*
     * Retrieve interruption number
     */
    int fiq = platform_get_irq(pdev, 0);
    if (fiq < 0)
    {
        printk("p264 driver : No IRQ number \n");
        err = -ENOENT;
        goto no_fiq;
    }
    else
        printk("p264 : fiq number %d\n",fiq);

    /*
     * Set FIQ in VIC register
     */
    writel(1 << fiq, PARROT6_VA_VIC + VIC_INT_SELECT);

    /*
     * Enable FIQ
     */
    enable_fiq(fiq);

    return 0;

no_fiq:
    release_fiq(&fh);
err:
    return err;
}

static int p264_remove(struct platform_device *pdev)
{
    /* we are protected by module refcount.
     * this call is possible only if there is no user
     */
    /*
     * Release FIQ
     */
    disable_fiq(20);
    release_fiq(&fh);

    kfree(fiq_reg_output);
    fiq_reg_output = NULL;

    misc_deregister(&p264_miscdev);
    dev_info(&pdev->dev, "p264 driver removed\n");
    return 0;
}

static struct platform_driver p264_driver = {
    .probe              = p264_probe,
    .remove             = p264_remove,
    .driver             = {
        .name   = "h264_p6",
    },
};

static int __devinit p264_init(void)
{
  printk("p264 driver : p264 init\n");
  /* XXX this should use platform stuff... */
  ui_h264_reg = ioremap(PARROT6_H264, 0x10000);
  if (ui_h264_reg == NULL)
  {
    printk( KERN_ERR "p264 driver : h264 ioremap failed\n");
    return -ENOMEM;
  }
  ui_sysc_reg = ioremap(PARROT6_SYS, 0x10);
  if (ui_sysc_reg == NULL)
  {
    printk( KERN_ERR "p264 driver : sys ioremap failed\n");
    return -ENOMEM;
  }

  return platform_driver_register(&p264_driver);
}

static void __exit p264_exit(void)
{
    printk("p264 driver : p264 exit\n");
    platform_driver_unregister(&p264_driver);
}

module_init(p264_init);
module_exit(p264_exit);


MODULE_AUTHOR("PARROT SA by Pierre ELINE <pierre.eline@parrot.com>");
MODULE_DESCRIPTION("p264 P6 driver");
MODULE_LICENSE("GPL");
