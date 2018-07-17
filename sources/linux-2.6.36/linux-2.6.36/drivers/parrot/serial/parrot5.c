/*
 * @file linux/drivers/char/parrot5.c
 * @brief Driver for Parrot5/5+/6 UARTs
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-07-11
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

#if defined(CONFIG_SERIAL_PARROT5_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/kernel.h>

#include <linux/io.h>
#include <asm/sizes.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>

/* for PORT_PARROT5 */
#include <mach/parrot_kernel_ids.h>

#define PARROT5_SERIAL_NAME	       "ttyPA"
#define PARROT5_SERIAL_MAJOR	       204
#define PARROT5_SERIAL_MINOR	       64

#define UART_TRX_DUMMY_RX	       (1 << 16)
#define UART_NR                        4

/* module parameter to enable/disable low latency */
static int low_latency_map[UART_NR] = {[0 ... (UART_NR-1)] = 0};
module_param_array(low_latency_map, int, NULL, S_IRUGO);
MODULE_PARM_DESC(low_latency_map, "UARTs low latency enable/disable");

#define UART_SNOOP_PATTERN_MAX_LEN 32 /* must be a power of 2 */

struct uart_snoop {
	unsigned int            length;
	unsigned int            pos;
	unsigned int            filter_first;
	unsigned int            filter_last;
	int                     pending;
	int                     synchronized;
	u64                     nextstamp;
	u16                     pattern[UART_SNOOP_PATTERN_MAX_LEN];
	u8                      buffer[UART_SNOOP_PATTERN_MAX_LEN];
	u32                     version;
	u8                     *page;
};

static int parrot5_serial_snoop_queue(struct uart_port *port, unsigned char c);

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_parrot5_port {
	struct uart_port	port;
	u32                     spd;
	u32                     lcr;
	u32                     lcr_fifo;
	unsigned int            tx_fifosize;
	unsigned int            tx_threshold;
	struct clk             *uart_clk;
	unsigned int            console_active;
#if defined(CONFIG_SERIAL_PARROT5_SNOOP)
	struct uart_snoop       snoop;
#endif
};

/* port locked, interrupts locally disabled */
static void parrot5_serial_start_tx(struct uart_port *port)
{
	__raw_writel(UART_ITX_TIM, port->membase+_UART_ITX);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_stop_tx(struct uart_port *port)
{
	/* writing 0 to RSTBRK disables TX interrupts */
	__raw_writel(0, port->membase+_UART_RSTBRK);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_stop_rx(struct uart_port *port)
{
	__raw_writel(0, port->membase+_UART_IRX);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_enable_ms(struct uart_port *port)
{
	/* nothing to do */
}

static void parrot5_serial_rx_chars(struct uart_port *port, u32 status)
{
	unsigned int c, flag;

	while (status & UART_STATUS_RXFILLED) {

		c = __raw_readl(port->membase+_UART_TRX) | UART_TRX_DUMMY_RX;
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(c & UART_TRX_ANY_ERROR)) {

			if (c & UART_TRX_RXBREAK) {
				port->icount.brk++;
				if (uart_handle_break(port)) {
					goto ignore_char;
				}
			}
			if (c & UART_TRX_PARITY_ERROR) {
				port->icount.parity++;
			}
			if (c & UART_TRX_FRAME_ERROR) {
				port->icount.frame++;
			}
			if (c & UART_TRX_OVERRUN) {
				port->icount.overrun++;
			}

			c &= port->read_status_mask;

			if (c & UART_TRX_RXBREAK) {
				flag = TTY_BREAK;
			}
			else if (c & UART_TRX_PARITY_ERROR) {
				flag = TTY_PARITY;
			}
			else if (c & UART_TRX_FRAME_ERROR) {
				flag = TTY_FRAME;
			}
		}

		if (uart_handle_sysrq_char(port, c & 0xff)) {
			goto ignore_char;
		}
		if (!parrot5_serial_snoop_queue(port, c & 0xff))
			uart_insert_char(port, c, UART_TRX_OVERRUN, c, flag);

	ignore_char:
		status = __raw_readl(port->membase+_UART_STATUS);
	}
}

static void parrot5_serial_tx_chars(struct uart_port *port, u32 status)
{
	int count;
	struct circ_buf *xmit = &port->state->xmit;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	if (port->x_char) {
		__raw_writel(port->x_char, port->membase+_UART_TRX);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		parrot5_serial_stop_tx(port);
		goto out;
	}

	count = up->tx_fifosize;

	/* on Parrot5+ chips TX fifo has a configurable IRQ threshold */
	if (up->tx_threshold && (status & UART_STATUS_TXFILLED)) {
		count -= up->tx_threshold;
	}

	while (!uart_circ_empty(xmit) && (count-- > 0)) {

		__raw_writel(xmit->buf[xmit->tail], port->membase+_UART_TRX);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}
	if (uart_circ_empty(xmit)) {
		parrot5_serial_stop_tx(port);
	}

out:
	return;
}

static irqreturn_t parrot5_serial_int_pushthread(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct tty_struct *tty = port->state->port.tty;
	tty_flip_buffer_push(tty);
	return IRQ_HANDLED;
}

static irqreturn_t parrot5_serial_int(int irq, void *dev_id)
{
	u32 status;
	struct uart_port *port = dev_id;
	struct tty_struct *tty = port->state->port.tty;
	irqreturn_t ret = IRQ_HANDLED;

	spin_lock(&port->lock);

	status = __raw_readl(port->membase+_UART_STATUS);

	do {
		if (status & UART_STATUS_ITRX) {
			parrot5_serial_rx_chars(port, status);
			/* tty_flip_buffer_push doc say we can't call this from
			 * interrupt context. Moved in threaded irq for low
			 * latency port context, when process is not queued.
			 */
			if (!low_latency_map[port->line]) {
				spin_unlock(&port->lock);
				tty_flip_buffer_push(tty);
				spin_lock(&port->lock);
			}
			else {
				ret = IRQ_WAKE_THREAD;
			}
		}
		if (status & UART_STATUS_ITTX) {
			parrot5_serial_tx_chars(port, status);
		}
		status = __raw_readl(port->membase+_UART_STATUS);

	} while (status & (UART_STATUS_ITRX|UART_STATUS_ITTX));

	spin_unlock(&port->lock);

	return ret;
}

/* port unlocked, interrupts status is called dependant */
static unsigned int parrot5_serial_tx_empty(struct uart_port *port)
{
	u32 status = __raw_readl(port->membase+_UART_STATUS);
	return (status & UART_STATUS_TXFILLED)? 0 : TIOCSER_TEMT;
}

/* port locked, interrupts locally disabled */
static unsigned int parrot5_serial_get_mctrl(struct uart_port *port)
{
	unsigned int result;
	/*
	u32 status;

	status = __raw_readl(port->membase+_UART_STATUS);
	*/
	result = TIOCM_CAR | TIOCM_DSR;

	/* pretend CTS is always on, otherwise we would have to dynamically
	   report CTS changes, which is useless anyway since everything is done
	   in HW */
	if (1 /* !(status & UART_STATUS_CTSN) */) {
		result |= TIOCM_CTS;
	}
	return result;
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 lcr = __raw_readl(port->membase+_UART_LCR);

	/* allow manual control of nRTS pin */
	if (mctrl & TIOCM_RTS) {
		/* restore automatic nRTS control */
		lcr &= ~UART_LCR_RTS_CTRL;
	}
	else {
		/* force nRTS to 1, i.e. throttle */
		lcr |= UART_LCR_RTS_CTRL;
	}
	__raw_writel(lcr, port->membase+_UART_LCR);
}

/* port unlocked, interrupts status is called dependant */
static void parrot5_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (break_state) {
		/* special write: any value will do */
		__raw_writel(0, port->membase+_UART_SETBRK);
	}
	else {
		/* special write: any value will do */
		__raw_writel(0, port->membase+_UART_RSTBRK);
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static struct clk *parrot5_serial_clk_enable(int line)
{
	static const char *clk_name[] = {"uart0", "uart1", "uart2", "uart3"};
	struct clk *uart_clk;

	uart_clk = clk_get(NULL, clk_name[line]);
	if (!IS_ERR(uart_clk)) {
		clk_enable(uart_clk);
	}
	return uart_clk;
}

/* port_sem locked, interrupts globally disabled */
static int parrot5_serial_startup(struct uart_port *port)
{
	int ret;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	/* allocate IRQ */
	if (low_latency_map[port->line])
		ret = request_threaded_irq(port->irq, parrot5_serial_int,
					   parrot5_serial_int_pushthread, 0,
					   "parrot5-uart", port);
	else
		ret = request_irq(port->irq, parrot5_serial_int, 0,
				  "parrot5-uart", port);
	if (ret) {
		goto out;
	}

	up->uart_clk = parrot5_serial_clk_enable(port->line);
	if (IS_ERR(up->uart_clk)) {
		free_irq(port->irq, port);
		goto out;
	}

	/* stop UART */
	__raw_writel(0, port->membase+_UART_SPD);

	/* write configuration */
	__raw_writel(up->lcr, port->membase+_UART_LCR);

	spin_lock_irq(&port->lock);

	/* start UART */
	__raw_writel(up->spd, port->membase+_UART_SPD);

	/* enable interrupts */
	__raw_writel(UART_IRX_RIM, port->membase+_UART_IRX);
        __raw_writel(UART_ITX_TIM, port->membase+_UART_ITX);

	spin_unlock_irq(&port->lock);

 out:
	return ret;
}

/* port_sem locked, interrupts status is caller dependent */
static void parrot5_serial_shutdown(struct uart_port *port)
{
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	/* disable all interrupts and reset break */
	spin_lock_irq(&port->lock);
	__raw_writel(0, port->membase+_UART_IRX);
        __raw_writel(0, port->membase+_UART_RSTBRK);
	spin_unlock_irq(&port->lock);

	/* free the interrupt */
	free_irq(port->irq, port);

	/* disable the port */
	__raw_writel(0, port->membase+_UART_SPD);

	if (up->console_active == 0) {
		clk_disable(up->uart_clk);
		clk_put(up->uart_clk);
	}
}

static void parrot5_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{
	u32 lcr;
	unsigned long flags;
	unsigned int baud, csize;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the baudrate for us.
	 */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk >> 20,
				  port->uartclk >> 4);

	if (baud) {
		up->spd = (port->uartclk+(baud/2))/baud;
	}

	/* we only support CS7 and CS8 */
	csize = termios->c_cflag & CSIZE;

	if ((csize != CS7) && (csize != CS8)) {
		/* default to CS8 */
		csize = (old && ((old->c_cflag & CSIZE) == CS7))? CS7 : CS8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= csize;
	}
	lcr = (csize == CS8)? UART_LCR_EIGHTBITS : 0;

	if (termios->c_cflag & CSTOPB) {
		lcr |= UART_LCR_TWOSTOP;
	}
	if (termios->c_cflag & PARENB) {
		lcr |= UART_LCR_PARITY;
		if (!(termios->c_cflag & PARODD)) {
			lcr |= UART_LCR_EVEN;
		}
	}
	if (termios->c_cflag & CRTSCTS) {
		lcr |= UART_LCR_CTSMODE;
	}
	if (port->fifosize > 1) {
		lcr |= up->lcr_fifo;
	}

	/* change port with interrupts disabled */
	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_TRX_OVERRUN | 0xff;

	if (termios->c_iflag & INPCK) {
		port->read_status_mask |=
			UART_TRX_FRAME_ERROR | UART_TRX_PARITY_ERROR;
	}
	if (termios->c_iflag & (BRKINT | PARMRK)) {
		port->read_status_mask |= UART_TRX_RXBREAK;
	}

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR) {
		port->ignore_status_mask |=
			UART_TRX_FRAME_ERROR | UART_TRX_PARITY_ERROR;
	}
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_TRX_RXBREAK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR) {
			port->ignore_status_mask |= UART_TRX_OVERRUN;
		}
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0) {
		port->ignore_status_mask |= UART_TRX_DUMMY_RX;
	}

	up->lcr = lcr;

	/* set baud rate */
	__raw_writel(up->spd, port->membase+_UART_SPD);

	/* configure port */
	__raw_writel(up->lcr, port->membase+_UART_LCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *parrot5_serial_type(struct uart_port *port)
{
	return (port->type == PORT_PARROT5)? "PARROT5" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void parrot5_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int parrot5_serial_request_port(struct uart_port *port)
{
	struct resource *res;

	res = request_mem_region(port->mapbase, SZ_4K, "parrot5-uart");

	return (res != NULL)? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void parrot5_serial_config_port(struct uart_port *port, int flags)
{
	if ((flags & UART_CONFIG_TYPE) &&
	    (parrot5_serial_request_port(port) == 0)) {
		port->type = PORT_PARROT5;
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int parrot5_serial_verify_port(struct uart_port *port,
				      struct serial_struct *ser)
{
	int ret = 0;

	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_PARROT5)) {
		ret = -EINVAL;
	}

	return ret;
}

static struct uart_ops parrot5_serial_pops = {
	.tx_empty	= parrot5_serial_tx_empty,
	.set_mctrl	= parrot5_serial_set_mctrl,
	.get_mctrl	= parrot5_serial_get_mctrl,
	.stop_tx	= parrot5_serial_stop_tx,
	.start_tx	= parrot5_serial_start_tx,
	.stop_rx	= parrot5_serial_stop_rx,
	.enable_ms	= parrot5_serial_enable_ms,
	.break_ctl	= parrot5_serial_break_ctl,
	.startup	= parrot5_serial_startup,
	.shutdown	= parrot5_serial_shutdown,
	.set_termios	= parrot5_serial_set_termios,
	.type		= parrot5_serial_type,
	.release_port	= parrot5_serial_release_port,
	.request_port	= parrot5_serial_request_port,
	.config_port	= parrot5_serial_config_port,
	.verify_port	= parrot5_serial_verify_port,
};

static struct uart_parrot5_port p5_ports[UART_NR];

/*
 * Initialize a single Parrot5/5+/6 serial port.
 */
static void __init parrot5_serial_init_port(struct uart_parrot5_port *up,
					    int index)
{
	/* configure fifos size and threshold */
	if (parrot_chip_is_p5()) {
		up->tx_fifosize = UART_TX_FIFO_SIZE_P5;
		/* Parrot5 does not have a tx threshold */
		up->tx_threshold = 0;
		/*
		 * set rx fifo threshold to 16 bytes, i.e. an interrupt will
		 * be raised when 16 empty slots are left in rx fifo
		 */
		up->lcr_fifo =
			UART_LCR_RXFIFO|
			UART_LCR_TXFIFO|
			UART_LCR_FIFO_THRESHOLD_16;
	}
	else {
		/* Parrot5+ and Parrot6 */
		up->tx_fifosize = UART_TX_FIFO_SIZE_P5P;
		/*
		 * set default tx threshold to 4 bytes, i.e. a tx
		 * interrupt will be raised when 4 bytes are left in tx fifo
		 */
		up->tx_threshold = 4;
		/*
		 * set rx fifo threshold to 48 bytes, i.e. an rx interrupt
		 * will be raised when 16 empty slots are left in rx fifo
		 */
		up->lcr_fifo =
			UART_LCR_RXFIFO|
			UART_LCR_TXFIFO|
			UART_LCR_FIFO_RX_THRESHOLD_48|
			UART_LCR_FIFO_TX_THRESHOLD_4;
	}

	up->spd = 0;
	up->lcr = 0;
	up->port.iotype = UPIO_MEM;
	up->port.fifosize = up->tx_fifosize-up->tx_threshold;
	up->port.ops = &parrot5_serial_pops;
	/* Force low latency for port 1 */
	if (low_latency_map[index])
		up->port.flags = UPF_BOOT_AUTOCONF|UPF_LOW_LATENCY;
	else
		up->port.flags = UPF_BOOT_AUTOCONF;
	up->port.line = index;
}

#ifdef CONFIG_SERIAL_PARROT5_CONSOLE

static void parrot5_serial_console_putchar(struct uart_port *port, int ch)
{
	while (__raw_readl(port->membase+_UART_STATUS) 
	       & UART_STATUS_TXFILLED) {
		barrier();
	}
	__raw_writel(ch, port->membase+_UART_TRX);
}

static void
parrot5_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	struct uart_port *port = (struct uart_port *)&p5_ports[co->index];
	uart_console_write(port, s, count, parrot5_serial_console_putchar);
}

static void __init
parrot5_serial_console_get_options(struct uart_port *port, int *baud,
				   int *parity, int *bits)
{
	u32 lcr, spd = __raw_readl(port->membase+_UART_SPD);

	if (spd) {
		*baud = clk_get_rate(NULL)/spd;

		lcr = __raw_readl(port->membase+_UART_LCR);

		*parity = 'n';
		if (lcr & UART_LCR_PARITY) {
			*parity = (lcr & UART_LCR_EVEN)? 'e' : 'o';
		}

		*bits = (lcr & UART_LCR_EIGHTBITS)? 8 : 7;
	}
}

static int __init parrot5_serial_console_setup(struct console *co,
					       char *options)
{
	struct uart_parrot5_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if ((co->index < 0) || (co->index >= UART_NR)) {
		co->index = 0;
	}
	up = &p5_ports[co->index];
	if (up->port.membase == NULL)
		return -ENODEV;

	up->console_active = 1;
	parrot5_serial_clk_enable(co->index);

	if (options) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	}
	else {
		parrot5_serial_console_get_options(&up->port, &baud, &parity,
						   &bits);
	}

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

#if defined(CONFIG_SERIAL_PARROT5_SNOOP)
/*
 * When option CONFIG_SERIAL_PARROT5_SNOOP is enabled, a sysfs interface
 * is added for matching a pattern on UART received bytes.
 * Detecting a pattern on rx bytes can be useful to trigger actions
 * with low-latency requirements
 * Two new attributes are created:
 *
 * <uart_sysfs_path>/snoop_pattern
 * <uart_sysfs_path>/snoop_match
 *
 * Writing an hexadecimal string into 'snoop_pattern' activates
 * pattern matching. Writing a null string disables it.
 * The hexadecimal string should be a sequence of 16-bit unsigned
 * numbers, each number representing a matched byte; for instance:
 *
 *  ffa3ffb4ffc60000ffaa
 *  | | | |_ value
 *  | | |_ mask
 *  | |
 *  | |_ value
 *  |_ mask
 *
 * A received byte 'b' matches if (b & mask) == value.
 * The above pattern example matches the following byte sequence:
 * a3 b4 c6 XX aa (XX = don't care)
 *
 * The latest matching sequence can be retrieved by reading (or
 * mmapping) attribute 'snoop_match', which is formatted as follows:
 *
 * struct uart_match {
 *         u32 version;
 *         u8  data[32]; // matched sequence cannot currently exceed 32 bytes
 * };
 *
 * Field 'version' is incremented each time 'data' is updated;
 * it can be used to get consistent values when mmap() is used.
 */
static void parrot5_serial_snoop_flush_queue(struct uart_port *port,
					     struct uart_snoop *sn);

static ssize_t parrot5_serial_snoop_pattern_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	int nibble, pos_first, pos_last;
	unsigned long flags;
	unsigned int i, j, length;
	u16 p, pattern[UART_SNOOP_PATTERN_MAX_LEN];
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(dev);
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;

	pos_first = -1;
	pos_last  = -1;

	length = count/4;
	if (length > UART_SNOOP_PATTERN_MAX_LEN)
		goto bad_pattern;

	for (i = 0; i < length; i++) {
		p = 0;
		for (j = 0; j < 4; j++) {
			nibble = hex_to_bin(buf[4*i+j]);
			if (nibble < 0)
				goto bad_pattern;
			p <<= 4;
			p |= nibble;
		}
		pattern[i] = p;

		if (pattern[i] & 0xff00) {
			/* remember the first non-zero mask in pattern */
			if (pos_first < 0)
				pos_first = i;
			/* remember the last non-zero mask in pattern */
			pos_last = i;
		}
	}

	if ((pos_first < 0) || (pos_last < 0))
		goto bad_pattern;

	spin_lock_irqsave(&port->lock, flags);

	parrot5_serial_snoop_flush_queue(port, sn);
	sn->synchronized = 0;
	sn->version = 0;
	sn->pos = 0;
	sn->filter_first = (unsigned int)pos_first;
	sn->filter_last  = (unsigned int)pos_last;
	sn->length = length;
	if (sn->length > 0)
		memcpy(sn->pattern, pattern, sn->length*sizeof(u16));

	spin_unlock_irqrestore(&port->lock, flags);

	return count;

bad_pattern:
	dev_err(dev, "invalid pattern specification\n");
	return -EINVAL;
}

static ssize_t parrot5_serial_snoop_pattern_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	ssize_t count = 0;
	unsigned long flags;
	unsigned int i, length;
	u16 pattern[UART_SNOOP_PATTERN_MAX_LEN];
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(dev);
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;

	spin_lock_irqsave(&port->lock, flags);
	length = sn->length;
	if (length > 0)
		memcpy(pattern, sn->pattern, length*sizeof(u16));
	spin_unlock_irqrestore(&port->lock, flags);

	for (i = 0; i < length; i++)
		count += scnprintf(&buf[count], 5, "%04x", pattern[i]);

	return count;
}

static DEVICE_ATTR(snoop_pattern,
		   S_IWUSR|S_IRUGO,
		   parrot5_serial_snoop_pattern_show,
		   parrot5_serial_snoop_pattern_store);

static ssize_t parrot5_serial_snoop_match_read(struct file *filp,
					       struct kobject *kobj,
					       struct bin_attribute *attr,
					       char *buf,
					       loff_t off,
					       size_t count)
{
	unsigned long flags;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(dev);
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;

	if (unlikely(!count))
		return count;

	if (off >= PAGE_SIZE)
		return -EINVAL;

	if (off+count > PAGE_SIZE)
		count = PAGE_SIZE-off;

	spin_lock_irqsave(&port->lock, flags);
	memcpy(buf, sn->page + off, count);
	spin_unlock_irqrestore(&port->lock, flags);

	return count;
}

static int parrot5_serial_snoop_match_mmap(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *attr,
					   struct vm_area_struct *vma)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(dev);
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;

	if (vma->vm_flags & VM_WRITE)
		return -EPERM;

	vma->vm_flags |= VM_RESERVED;

	return vm_insert_page(vma, vma->vm_start, virt_to_page(sn->page));
}

static struct bin_attribute snoop_match = {
	.attr = {
		.name = "snoop_match",
		.mode = S_IRUGO,
	},
	.read = parrot5_serial_snoop_match_read,
	.mmap = parrot5_serial_snoop_match_mmap,
	.size = PAGE_SIZE,
};

#define POS(_p)								\
	(((_p) + UART_SNOOP_PATTERN_MAX_LEN) & (UART_SNOOP_PATTERN_MAX_LEN-1))

static inline int match_byte(const struct uart_snoop *sn, unsigned int n)
{
	u16 pat;
	unsigned int pos;

	pat = sn->pattern[n];
	pos = POS(sn->pos + n - sn->length);

	return (sn->buffer[pos] & (pat >> 8)) == (pat & 0xff);
}


/* port locked, interrupts locally disabled */
static int parrot5_serial_snoop_match(struct uart_port *port, unsigned char c)
{
	unsigned int i, pos;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;

	if (!sn->length)
		return 0;

	/* first, store current byte */
	sn->buffer[sn->pos++] = c;
	sn->pos = POS(sn->pos);

	/* fast check: verify first and last discriminant bytes */
	if (!match_byte(sn, sn->filter_first) ||
	    !match_byte(sn, sn->filter_last))
		/* mismatch */
		return 0;

	/* slow check: verify rest of pattern */
	for (i = sn->filter_first+1; i < sn->filter_last; i++)
		if (!match_byte(sn, i))
			/* mismatch */
			return 0;

	/* match! */
	/* if not enough bytes have been seen, we may have a false positive */
	sn->version++;
	memcpy(sn->page, &sn->version, sizeof(sn->version));
	pos = POS(sn->pos - sn->length);

	for (i = 0; i < sn->length; i++)
		sn->page[sizeof(sn->version)+i] = sn->buffer[POS(pos+i)];

	return 1;
}

static void parrot5_serial_snoop_flush_queue(struct uart_port *port,
					     struct uart_snoop *sn)
{
	int i, pos, c;

	if (!sn->pending)
		return;

	pos = POS(sn->pos - sn->pending);

	for (i = 0; i < sn->pending; i++) {
		c = sn->buffer[POS(pos+i)];
		uart_insert_char(port, c, UART_TRX_OVERRUN, c, TTY_NORMAL);
	}

	sn->pending = 0;
}

static int parrot5_serial_snoop_queue(struct uart_port *port, unsigned char c)
{
	u16 pat;
	int match;
	u64 timestamp;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;
	struct uart_snoop *sn = &up->snoop;
	const u64 period = CONFIG_SERIAL_PARROT5_SNOOP_FILTER_PERIOD*1000000ULL;

	match = parrot5_serial_snoop_match(port, c);

	if (!period)
		return 0;

	if (sn->synchronized) {
		/* keep this byte queued in snoop buffer */
		pat = sn->pattern[sn->pending++];
		if ((c & (pat >> 8)) != (pat & 0xff)) {
			/* mismatch in queue */
			parrot5_serial_snoop_flush_queue(port, sn);
			sn->synchronized = 0;
		}
	} else {
		/* we're not synchronized, flush immediately */
		uart_insert_char(port, c, UART_TRX_OVERRUN, c, TTY_NORMAL);
	}

	if (sn->pending == sn->length) {
		/* decide if we should decimate or flush */
		timestamp = sched_clock();
		if (timestamp < sn->nextstamp)
			/* drop frame */
			sn->pending = 0;
		else
			/* keep frame and save next timestamp */
			sn->nextstamp = timestamp + period;
		parrot5_serial_snoop_flush_queue(port, sn);
	}

	if (match)
		sn->synchronized = 1;

	return 1;
}

static int parrot5_serial_snoop_probe(struct platform_device *dev)
{
	int ret;
	struct uart_parrot5_port *up = platform_get_drvdata(dev);
	struct uart_snoop *sn = &up->snoop;

	sn->length = 0;
	memset(sn->buffer, 0, sizeof(sn->buffer));

	sn->page = (u8 *)get_zeroed_page(GFP_KERNEL);
	if (sn->page == NULL) {
		dev_err(&dev->dev, "failed to allocate data page\n");
		ret = -ENOMEM;
		goto nopage;
	}

	ret = device_create_file(&dev->dev, &dev_attr_snoop_pattern);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to create pattern attribute\n");
		goto noattr;
	}

	ret = device_create_bin_file(&dev->dev, &snoop_match);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to create match attribute\n");
		goto nobinattr;
	}

	return 0;

nobinattr:
	device_remove_file(&dev->dev, &dev_attr_snoop_pattern);
noattr:
	free_page((unsigned long)sn->page);
nopage:
	return ret;
}

static void parrot5_serial_snoop_remove(struct platform_device *dev)
{
	struct uart_parrot5_port *up = platform_get_drvdata(dev);
	struct uart_snoop *sn = &up->snoop;

	device_remove_file(&dev->dev, &dev_attr_snoop_pattern);
	device_remove_bin_file(&dev->dev, &snoop_match);
	free_page((unsigned long)sn->page);
	sn->page = NULL;
}

#else /* !CONFIG_SERIAL_PARROT5_SNOOP */

static int parrot5_serial_snoop_queue(struct uart_port *port, unsigned char c)
{
	return 0;
}

static int parrot5_serial_snoop_probe(struct platform_device *dev)
{
	return 0;
}

static void parrot5_serial_snoop_remove(struct platform_device *dev)
{
}

#endif /* CONFIG_SERIAL_PARROT5_SNOOP */

static struct uart_driver parrot5_reg;
static struct console parrot5_serial_console = {
	.name		= PARROT5_SERIAL_NAME,
	.write		= parrot5_serial_console_write,
	.device		= uart_console_device,
	.setup		= parrot5_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &parrot5_reg,
};

/* XXX TODO support earlycon stuff ... */
#define PARROT5_CONSOLE	(&parrot5_serial_console)
#else
#define PARROT5_CONSOLE	NULL
#endif

static struct uart_driver parrot5_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "parrot5_serial",
	.nr			= UART_NR,
	.dev_name		= PARROT5_SERIAL_NAME,
	.major			= PARROT5_SERIAL_MAJOR,
	.minor			= PARROT5_SERIAL_MINOR,
	.cons			= PARROT5_CONSOLE,
};

static int parrot5_serial_suspend(struct platform_device *dev,
				  pm_message_t state)
{
	struct uart_parrot5_port *up = platform_get_drvdata(dev);

	if (up) {
		uart_suspend_port(&parrot5_reg, &up->port);
	}

	return 0;
}

static int parrot5_serial_resume(struct platform_device *dev)
{
	struct uart_parrot5_port *up = platform_get_drvdata(dev);

	if (up) {
		uart_resume_port(&parrot5_reg, &up->port);
	}

	return 0;
}

static int parrot5_serial_probe(struct platform_device *dev)
{
	int ret;
	struct resource *res;
	struct uart_parrot5_port *up = &p5_ports[dev->id];

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noregs;
	}

	up->port.dev = &dev->dev;
	up->port.mapbase = res->start;
	up->port.membase = ioremap(res->start, res->end-res->start+1);

	if (!up->port.membase) {
		ret = -ENOMEM;
		goto nomap;
	}

	up->port.irq = platform_get_irq(dev, 0);
	if (up->port.irq < 0) {
		ret = -ENOENT;
		goto noirq;
	}
	up->port.uartclk = clk_get_rate(NULL);
	parrot5_serial_init_port(up, dev->id);

	ret = uart_add_one_port(&parrot5_reg, &up->port);
	if (ret) {
		goto noadd;
	}

	platform_set_drvdata(dev, up);

	ret = parrot5_serial_snoop_probe(dev);
	if (ret)
		goto nosnoop;

	return 0;

nosnoop:
	platform_set_drvdata(dev, NULL);
	uart_remove_one_port(&parrot5_reg, &up->port);
noadd:
noirq:
	iounmap(up->port.membase);
nomap:
noregs:
	return ret;
}

static int parrot5_serial_remove(struct platform_device *dev)
{
	struct uart_parrot5_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (up) {
		parrot5_serial_snoop_remove(dev);
		uart_remove_one_port(&parrot5_reg, &up->port);
		iounmap(up->port.membase);
	}

	return 0;
}

static struct platform_driver parrot5_serial_driver = {
	.probe		= parrot5_serial_probe,
	.remove		= parrot5_serial_remove,
	.suspend	= parrot5_serial_suspend,
	.resume		= parrot5_serial_resume,
	.driver		= {
		.name	= "parrot5-uart",
	},
};

static int __init parrot5_serial_init(void)
{
	int ret;
	int i;

	printk(KERN_INFO "Serial: Parrot5/5+ UART driver $Revision: 1.19 $\n");
	for (i=0; i<UART_NR; i++)
		if (low_latency_map[i])
			printk(KERN_INFO "Serial: UART %d is low latency\n", i);

	ret = uart_register_driver(&parrot5_reg);
	if (ret == 0) {
		ret = platform_driver_register(&parrot5_serial_driver);
		if (ret) {
			uart_unregister_driver(&parrot5_reg);
		}
	}
	return ret;
}

static void __exit parrot5_serial_exit(void)
{
	platform_driver_unregister(&parrot5_serial_driver);
	uart_unregister_driver(&parrot5_reg);
}

module_init(parrot5_serial_init);
module_exit(parrot5_serial_exit);

MODULE_AUTHOR("Parrot S.A. <ivan.djelic@parrot.com>");
MODULE_DESCRIPTION("Parrot5/5+/6 serial port driver");
MODULE_LICENSE("GPL");
