
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/map.h>

    .text

    .global aai_fiq_handler_end

ENTRY(aai_fiq_handler_start)

    /* load stack pointer */
    stmdb   sp!, {r0-r12, lr}

    ldr     lr, [r11, #0x328]
    mov     r0, r9

    #ifdef CONFIG_ARCH_VARIANT_PARROT6I
    ldr     r12, =0xf77fffff
    #elif CONFIG_ARCH_VARIANT_PARROT6
    ldr     r12, =0xf7ffffff
    #else
    # error wrong arch defined
    #endif
    ands    r12, r12, lr
    beq     .launchirq

    mov     r0, #0

.itsloop:
    ldr     r12, [r8, #156]     /* INT flag */
    ands    r12, r12, lr
    beq     .nextit

    /*
     * increment fiq count - debug purpose
     */
    #if CONFIG_AAI_DBG_LEVEL > 0
    ldr     r1, [r8, #236]
    add     r1, r1, #1
    str     r1, [r8, #236]
    #endif

    /*
     * 32 bits channels are processed in irq mode
     */
    ldr     r1, [r8, #24]       /* chan->mode */
    cmp     r1, #32
    beq     .launchirq

    /*
     * increment bytes count
     */
    ldr     r2, [r8, #224]      /* dmaxfersz */
    ldr     r3, [r8, #52]       /* bytes_count */
    add     r3, r3, r2
    str     r3, [r8, #52]

    /*
     * get fifo[0]
     */
    add     r1, r8, #164        /* fifo[0] */

    /*
     * update fifo pointers
     */
    add     r10, r1, #4
    ldmia   r10, {r4-r7}
    str     r7, [r1, #8]        /* store new current */
    add     r7, r7, r2
    subs    r6, r6, r7
    movlt   r7, r4
    str     r7, [r1, #16]       /* store new follow */

    /*
     * period elapsed
     */
    ldr     r2, [r8, #48]       /* period_bytes */
    subs    r12, r2, r3
    ble     .launchirq

    /*
     * check transfer mode
     */
    ldr     r2, [r8, #24]       /* chan->mode */
    cmp     r2, #0xffffffff
    beq     .dmamode

    /*
     * fifo mode
     */
    ldr     r12, [r8, #228]     /* chan->dma_area */
    add     r12, r12, r5        /* cur = dma_area + current - start */
    sub     r12, r12, r4
    ldr     r1, [r1]            /* fifo->hfifo */
    add     r1, r11, r1         /* aai->iobase + hfifo */

    ldr     r3, [r8, #20]
    cmp     r3, #1
    bne     .aai_fifo_tx

    cmp     r2, #2
    beq     .aai_fifo_2w_32b
    cmp     r2, #4
    beq     .aai_fifo_4w_32b
    cmp     r2, #8
    beq     .aai_fifo_8w_32b

    ldmia   r1,   {r4-r7}
    stmia   r12!, {r4-r7}
    ldmia   r1,   {r4-r7}
    stmia   r12!, {r4-r7}
.aai_fifo_8w_32b:
    ldmia   r1,   {r4-r7}
    stmia   r12!, {r4-r7}
.aai_fifo_4w_32b:
    ldmia   r1,   {r4-r5}
    stmia   r12!, {r4-r5}
.aai_fifo_2w_32b:
    ldmia   r1,   {r4-r5}
    stmia   r12!, {r4-r5}

    b       .nextit

.aai_fifo_tx:
    cmp     r2, #2
    beq     .aai_fifo_2w_32b_tx
    cmp     r2, #4
    beq     .aai_fifo_4w_32b_tx
    cmp     r2, #8
    beq     .aai_fifo_8w_32b_tx

    ldmia   r12!, {r4-r7}
    stmia   r1,   {r4-r7}
    ldmia   r12!, {r4-r7}
    stmia   r1,   {r4-r7}
.aai_fifo_8w_32b_tx:
    ldmia   r12!, {r4-r7}
    stmia   r1,   {r4-r7}
.aai_fifo_4w_32b_tx:
    ldmia   r12!, {r4-r5}
    stmia   r1,   {r4-r5}
.aai_fifo_2w_32b_tx:
    ldmia   r12!, {r4-r5}
    stmia   r1,   {r4-r5}
    b       .nextit

.dmamode:
    /*
     * dma mode
     */
    ldr     r4, [r1, #24]
    str     r7, [r11, r4]
    b       .nextit

.launchirq:
    /*
     * call irq
     */
    ldr     r1, .vic_base
    /* mask fiq */
    mov     r12, #(1<<2)
    str     r12, [r1, #0x14]
    /* branch to MOST IRQ */
    mov     r12, #8
    str     r12, [r1, #0x18]

.nextit:
    add     r8, r8, #252    /* next aai_channel */
    add     r0, r0, #1
    cmp     r0, r9          /* AAI_NB_CHANNELS */
    blt     .itsloop

    /* return */
    ldmia   sp!, {r0-r12, lr}
    subs    pc, lr, #4

/* registers */
.vic_base:
    .word   0xfc000000

/* stack */
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee
    .word   0xeeeeeeee

aai_fiq_handler_end:
    .end

