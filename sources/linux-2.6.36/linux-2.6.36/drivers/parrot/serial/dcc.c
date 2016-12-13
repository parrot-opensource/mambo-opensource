/*
 *  linux/drivers/serial/dcc_serial.c
 *
 *  serial port emulation driver for the JTAG DCC Terminal.
 *
 * DCC(JTAG1) protocol version for JTAG ICE/ICD Debuggers:
 * 	Copyright (C) 2003, 2004, 2005 Hyok S. Choi (hyok.choi@samsung.com)
 * 	SAMSUNG ELECTRONICS Co.,Ltd.
 *  Copyright (C) 2008 Parrot SA by matthieu.castet@parrot.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Changelog:
 *   Oct-2003 Hyok S. Choi	Created
 *   Feb-2004 Hyok S. Choi	Updated for serial_core.c and 2.6 kernel
 *   Mar-2005 Hyok S. Choi	renamed from T32 to DCC
 *   Apr-2006 Hyok S. Choi	revised including the MAJOR number
 *
 */

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/major.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <linux/serial_core.h>

/* for PORT_DCC_JTAG1 */
#include <mach/parrot_kernel_ids.h>

//#undef CONFIG_ARCH_PARROT
/* XXX may be use a platform driver for this */
#ifdef CONFIG_ARCH_PARROT
#define DCC_IRQ_USED
#define IRQ_RX 27
#define IRQ_TX 28
#else
/* polling mode */
#define IRQ_RX 0x12345678
#define IRQ_TX 0

#warning polling mode
#endif

#define UART_NR			1	/* we have only one JTAG port */

#ifdef CONFIG_SERIAL_DCC_STDSERIAL
/* use ttyS for emulation of standard serial driver */
#define SERIAL_DCC_NAME		"ttyS"
#define SERIAL_DCC_MAJOR	TTY_MAJOR
#define SERIAL_DCC_MINOR	64
#else
/* use ttyJ0(128) */
#define SERIAL_DCC_NAME		"ttyJ"
#define SERIAL_DCC_MAJOR	204
#define SERIAL_DCC_MINOR	186
#endif

/* DCC Status Bits */
#define DCC_STATUS_RX		(1 << 0)
#define DCC_STATUS_TX		(1 << 1)

/* primitive JTAG1 protocol utilities */
static inline u32 __dcc_getstatus(void)
{
	u32 ret;

	asm __volatile__ ("mrc p14, 0, %0, c0, c0	@ read comms ctrl reg"
		: "=r" (ret));

	return ret;
}

static inline unsigned char __dcc_getchar(void)
{
	unsigned char c;

	asm __volatile__ ("mrc p14, 0, %0, c1, c0	@ read comms data reg"
		: "=r" (c));

	return c;
}

static inline void __dcc_putchar(unsigned char c)
{
	asm __volatile__ ("mcr p14, 0, %0, c1, c0	@ write a char"
		: /* no output register */
		: "r" (c));
}
/* end of JTAG1 dependencies */

static void dcc_serial_tx_chars(struct uart_port *port, int max_count);
static void dcc_serial_rx_chars(struct uart_port *port);
#ifdef DCC_IRQ_USED /* real IRQ used */
/* check if there is a char to read and when don't read too much char */
#define dcc_serial_tx_ready(port) \
	((__dcc_getstatus() & DCC_STATUS_TX) == 0)

enum {
	TX_OFF,
	TX_ON,
};
static int dcc_serial_tx_enable;
enum {
	RX_OFF,
	RX_ON,
	RX_PAUSE,
};
static int dcc_serial_rx_enable;
/* port locked, interrupts locally disabled */
static void dcc_serial_start_tx(struct uart_port *port)
{
	if (dcc_serial_tx_enable == TX_OFF) {
		enable_irq(port->irq);
		dcc_serial_tx_enable = TX_ON;
	}
}

/* port locked, interrupts locally disabled */
static void dcc_serial_stop_tx(struct uart_port *port)
{
	if (dcc_serial_tx_enable == TX_ON) {
		disable_irq(port->irq);
		dcc_serial_tx_enable = TX_OFF;
	}
}

/* port locked, interrupts locally disabled */
static void dcc_serial_stop_rx(struct uart_port *port)
{
	if (dcc_serial_rx_enable == RX_ON) {
		disable_irq((int)port->membase);
	}
	dcc_serial_rx_enable = RX_OFF;
}

/* port locked, interrupts locally disabled */
static void dcc_serial_throttle_rx(struct uart_port *port, int stop)
{
	if (stop && dcc_serial_rx_enable == RX_ON) {
		disable_irq((int)port->membase);
		dcc_serial_rx_enable = RX_PAUSE;
	}
	else if (stop == 0 && dcc_serial_rx_enable == RX_PAUSE) {
		enable_irq((int)port->membase);
		dcc_serial_rx_enable = RX_ON;
	}
}

#if 0
static void dcc_serial_rx_mchars(struct uart_port *port)
{
	int timeout;
	unsigned int read = 0;
	unsigned int it = 0;
	unsigned int it1 = 0;
	do {
		timeout = 1000;
		/* wait for a char */
		//while ((__dcc_getstatus() & DCC_STATUS_RX) == 0 && timeout-- > 0) {
		while ((readl(0xfc000008) & 0x08000000) == 0) {
			cpu_relax();
		}
		/* read it */
		if (timeout >= 0) {
			if (readl(0xfc000008) & 0x08000000)
				it++;
			read++;
			if (read & 0x100) {
				printk("dcc_serial stat %u %u %u\n", read, it, it1);
				read = it = it1 = 0;
			}
			dcc_serial_rx_chars(port);
			if (readl(0xfc000008) & 0x08000000)
				it1++;
		}
	} while (/*timeout >= 0*/1);
}
#endif
#ifdef CONFIG_SERIAL_DCC_IDLE_POLL
int dcc_idle(void)
{
	return 0;
}
#endif

static irqreturn_t dcc_serial_int_rx(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	spin_lock(&port->lock);
	dcc_serial_rx_chars(port);
	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static irqreturn_t dcc_serial_int_tx(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	spin_lock(&port->lock);

	dcc_serial_tx_chars(port, 64);
	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static int dcc_serial_startup(struct uart_port *port)
{
	int retval;

	dcc_serial_tx_enable = TX_ON;
	dcc_serial_rx_enable = RX_ON;
	/* Allocate the IRQ */
	retval = request_irq(port->irq, dcc_serial_int_tx, IRQF_DISABLED,
			     "serial_dcc_serial_tx", port);
	if (retval)
		return retval;

	/* Allocate the IRQ */
	retval = request_irq((int)port->membase, dcc_serial_int_rx, IRQF_DISABLED,
			     "serial_dcc_serial_rx", port);
	if (retval)
		return retval;

	return 0;
}

static void dcc_serial_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
	free_irq((int)port->membase, port);
}
#else /* emulation by scheduled work */
static void dcc_serial_poll(struct work_struct *work);
static DECLARE_DELAYED_WORK(dcc_serial_poll_task, dcc_serial_poll);
static struct uart_port dcc_serial_port;
static int dcc_serial_active;

/* in polling mode, we wait for the next character */
static int dcc_serial_tx_ready(struct uart_port *port) {
	while (__dcc_getstatus() & DCC_STATUS_TX) {
		/* while waiting, we can check rx : it is free */
		dcc_serial_rx_chars(port);
		cpu_relax();
	}
	return 1;
}

static void dcc_serial_start_tx(struct uart_port *port)
{
	dcc_serial_tx_chars(port, 0xFFFF);
}

static void dcc_serial_stop_tx(struct uart_port *port)
{
}

static void dcc_serial_throttle_rx(struct uart_port *port, int stop)
{
}

static void dcc_serial_stop_rx(struct uart_port *port)
{
}

/* you can call this on your idle loop instead of sleeping
 */
#ifdef CONFIG_SERIAL_DCC_IDLE_POLL
int dcc_idle(void)
{
	struct uart_port *port = &dcc_serial_port;
	if (dcc_serial_active == 0)
		return 0;

	spin_lock(&port->lock);

	dcc_serial_rx_chars(port);
	dcc_serial_tx_chars(port, 64);
	dcc_serial_rx_chars(port);

	spin_unlock(&port->lock);
	return 0;
}
#endif

/* we poll dcc every jiffies */
static void dcc_serial_poll(struct work_struct *work)
{
	struct uart_port *port = &dcc_serial_port;

	spin_lock(&port->lock);

	dcc_serial_rx_chars(port);
	dcc_serial_tx_chars(port, 0xFFFF);
	dcc_serial_rx_chars(port);

	schedule_delayed_work(&dcc_serial_poll_task, 1);

	spin_unlock(&port->lock);
}
static int dcc_serial_startup(struct uart_port *port)
{
	/* shcedule the polling work */
	schedule_delayed_work(&dcc_serial_poll_task, 1);
	dcc_serial_active = 1;

	return 0;
}

static void dcc_serial_shutdown(struct uart_port *port)
{
	cancel_rearming_delayed_work(&dcc_serial_poll_task);
	dcc_serial_active = 0;
}
#endif /* end of DCC_IRQ_USED */


static void dcc_serial_putchar(struct uart_port *port, int ch)
{
	while (__dcc_getstatus() & DCC_STATUS_TX)
		cpu_relax();
	__dcc_putchar((char)(ch & 0xFF));
}

static void dcc_serial_rx_chars(struct uart_port *port)
{
	unsigned char ch;
	/* max char to send */
	int count = 64;
	struct tty_struct *tty = port->info->port.tty;

	/*
	 * check input.
	 * checking JTAG flag is better to resolve the status test.
	 * incount is NOT used for JTAG1 protocol.
	 */

	while (__dcc_getstatus() & DCC_STATUS_RX && count-- > 0) {
		/* for JTAG 1 protocol, incount is always 1. */
		ch = __dcc_getchar();

		tty_insert_flip_char(tty, ch, TTY_NORMAL);
		port->icount.rx++;
	}
	tty_flip_buffer_push(tty);
}

static void dcc_serial_tx_chars(struct uart_port *port, int max_count)
{
	struct circ_buf *xmit = &port->info->xmit;

	if (port->x_char) {
		dcc_serial_putchar(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		dcc_serial_stop_tx(port);
		goto out;
	}

	while (!uart_circ_empty(xmit) && dcc_serial_tx_ready(port)
			&& max_count-- > 0) {
		__dcc_putchar(xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}
	if (uart_circ_empty(xmit)) {
		dcc_serial_stop_tx(port);
	}

out:
	return;
}

static unsigned int dcc_serial_tx_empty(struct uart_port *port)
{
	return (__dcc_getstatus() & DCC_STATUS_TX) ? 0:TIOCSER_TEMT;
}

static unsigned int dcc_serial_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CD;
}

/* need for throttle/unthrottle */
static void dcc_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* throttle clear TIOCM_RTS, unthrottle set it */
	dcc_serial_throttle_rx(port, !(mctrl & TIOCM_RTS));
}

static void dcc_serial_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * We don't support parity, stop bits, or anything other
	 * than 8 bits, so clear these termios flags.
	 */
	termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CREAD);
	termios->c_cflag |= CS8;

	/*
	 * We don't appear to support any error conditions either.
	 */
	termios->c_iflag &= ~(INPCK | IGNPAR | IGNBRK | BRKINT);

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16); 
	quot = uart_get_divisor(port, baud);

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char * dcc_serial_type(struct uart_port *port)
{
	return port->type == PORT_DCC_JTAG1 ? "DCC" : NULL;
}

static int dcc_serial_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void dcc_serial_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_DCC_JTAG1;
		dcc_serial_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int dcc_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_DCC_JTAG1)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

/* dummy operation handlers for uart_ops */
static void dcc_serial_dummy_ops(struct uart_port *port)
{
}

static void dcc_serial_break_ctl(struct uart_port *port, int i)
{
}

static struct uart_ops dcc_serial_pops = {
	.tx_empty	= dcc_serial_tx_empty,
	.set_mctrl	= dcc_serial_set_mctrl,
	.get_mctrl	= dcc_serial_get_mctrl,
	.stop_tx	= dcc_serial_stop_tx,
	.start_tx	= dcc_serial_start_tx,
	.stop_rx	= dcc_serial_stop_rx,
	.enable_ms	= dcc_serial_dummy_ops,
	.break_ctl	= dcc_serial_break_ctl,
	.startup	= dcc_serial_startup,
	.shutdown	= dcc_serial_shutdown,
	.set_termios	= dcc_serial_set_termios,
	.type		= dcc_serial_type,
	.release_port	= dcc_serial_dummy_ops,
	.request_port	= dcc_serial_request_port,
	.config_port	= dcc_serial_config_port,
	.verify_port	= dcc_serial_verify_port,
};

static struct uart_port dcc_serial_port = {
	.mapbase	= 0x12345678,		/* for serial_core.c */
	.iotype		= UPIO_MEM,	
	.membase	= (char*)IRQ_RX,	/* we need these garbages */
	.irq		= IRQ_TX,
	.uartclk	= 14745600,
	.fifosize	= 0,
	.ops		= &dcc_serial_pops,
	.flags		= UPF_BOOT_AUTOCONF,
	.line		= 0,
};

#ifdef CONFIG_SERIAL_DCC_CONSOLE
static void dcc_serial_console_write(struct console *co, const char *s, unsigned int count)
{
	uart_console_write(&dcc_serial_port, s, count, dcc_serial_putchar);
}

static int __init dcc_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &dcc_serial_port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver dcc_serial_reg;
static struct console dcc_serial_console = {
	.name		= SERIAL_DCC_NAME,
	.write		= dcc_serial_console_write,
	.device		= uart_console_device,
	.setup		= dcc_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &dcc_serial_reg,
};

static int __init dcc_serial_console_init(void)
{
	register_console(&dcc_serial_console);
	return 0;
}
console_initcall(dcc_serial_console_init);

#define DCC_CONSOLE	&dcc_serial_console
#else
#define DCC_CONSOLE	NULL
#endif

static struct uart_driver dcc_serial_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= SERIAL_DCC_NAME,
	.dev_name	= SERIAL_DCC_NAME,
	.major		= SERIAL_DCC_MAJOR,
	.minor		= SERIAL_DCC_MINOR,
	.nr		= UART_NR,
	.cons		= DCC_CONSOLE,
};

static int __init dcc_serial_init(void)
{
	int ret;

	printk(KERN_INFO "DCC: JTAG1 Serial emulation driver driver $Revision: 1.9 $\n");

	ret = uart_register_driver(&dcc_serial_reg);

	if (ret)
		return ret;

	uart_add_one_port(&dcc_serial_reg, &dcc_serial_port);

	return 0;
}

__initcall(dcc_serial_init);

MODULE_DESCRIPTION("DCC(JTAG1) JTAG debugger console emulation driver");
MODULE_AUTHOR("Hyok S. Choi <hyok.choi@samsung.com>");
MODULE_SUPPORTED_DEVICE(SERIAL_DCC_NAME);
MODULE_LICENSE("GPL");
