// SPDX-License-Identifier: GPL-2.0
/*
 * tty0uart - Null-modem emulator connecting virtual tty to virtual UART 
 * Copyright (C) 2019  Hyeonki Hong <hhk7734@gmail.com>
 *
 * Based in tty0tty - linux null modem emulator
 *  - Copyright (c) 2013  Luis Claudio Gambôa Lopes (lcgamboa@yahoo.com)
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#endif
#include <asm/uaccess.h>

#include <linux/platform_device.h>
#include <linux/serial_core.h>

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Hyeonki Hong <hhk7734@gmail.com>"
#define DRIVER_DESC                                                            \
	"tty0uart: Null-modem emulator connecting virtual tty to virtual UART"

short pairs = 1; //Default number of pairs of devices
module_param(pairs, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(pairs,
		 "Number of pairs of devices to be created, maximum of 4");

#if 0
#define TTY0UART_MAJOR 240 /* experimental range */
#define TTY0UART_MINOR 16
#else
#define TTY0UART_MAJOR 0 /* dynamic allocation */
#define TTY0UART_MINOR 0
#endif

#define PORT_TTY0UART 255

#define MAX_PORT_NUM 4

/*
 * fake UART values
 * MCR - Modem Control Register
 * MSR - Modem Status Register
 */

// /* modem lines */
// #define TIOCM_LE 0x001
// #define TIOCM_DTR 0x002
// #define TIOCM_RTS 0x004
// #define TIOCM_ST 0x008
// #define TIOCM_SR 0x010
// #define TIOCM_CTS 0x020
// #define TIOCM_CAR 0x040
// #define TIOCM_RNG 0x080
// #define TIOCM_DSR 0x100
// #define TIOCM_CD TIOCM_CAR
// #define TIOCM_RI TIOCM_RNG
// #define TIOCM_OUT1 0x2000
// #define TIOCM_OUT2 0x4000
// #define TIOCM_LOOP 0x8000

static struct tty_port *tty0uart_tty_port;

struct tty0uart_tty_serial {
	struct tty_struct *tty; /* pointer to the tty for this device */
	bool is_open;
	struct semaphore sem; /* locks this structure */

	/* for tiocmget and tiocmset functions */
	unsigned int msr; /* MSR shadow */
	unsigned int mcr; /* MCR shadow */

	/* for ioctl fun */
	struct serial_struct serial;
	wait_queue_head_t wait;
	struct async_icount icount;
};

static struct tty0uart_tty_serial **tty0uart_tty_serials; /* initially all NULL */

struct tty0uart_uart_serial {
	struct uart_port port;
	bool is_open;

	/* for tiocmget and tiocmset functions */
	unsigned int msr; /* MSR shadow */
	unsigned int mcr; /* MCR shadow */
};

static struct tty0uart_uart_serial tty0uart_uart_serials[MAX_PORT_NUM];

static int tty0uart_tty_open(struct tty_struct *tty, struct file *file)
{
	struct tty0uart_tty_serial *t_serial;
	int index;
	unsigned int tty_msr = 0;
	unsigned int uart_mcr = 0;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	/* get the serial object associated with this tty pointer */
	index = tty->index;
	t_serial = tty0uart_tty_serials[index];

	if (!t_serial) {
		/* first time accessing this device, let's create it */
		t_serial = kmalloc(sizeof(*t_serial), GFP_KERNEL);
		if (!t_serial)
			return -ENOMEM;

		sema_init(&t_serial->sem, 1);
		t_serial->is_open = false;

		tty0uart_tty_serials[index] = t_serial;
	}

	tty0uart_tty_port[index].tty = tty;
	tty->port = &tty0uart_tty_port[index];

	// if (tty0uart_uart_serials[index] != NULL)
	if (tty0uart_uart_serials[index].is_open)
		uart_mcr = tty0uart_uart_serials[index].mcr;

	//null modem connection
	if ((uart_mcr & TIOCM_RTS) == TIOCM_RTS) {
		tty_msr |= TIOCM_CTS;
	}

	if ((uart_mcr & TIOCM_DTR) == TIOCM_DTR) {
		tty_msr |= TIOCM_DSR;
		tty_msr |= TIOCM_CD;
	}

	t_serial->msr = tty_msr;
	t_serial->mcr = 0;

	/* register the tty driver */

	down(&t_serial->sem);

	/* save our structure within the tty structure */
	tty->driver_data = t_serial;
	t_serial->tty = tty;

	t_serial->is_open = true;

	up(&t_serial->sem);

	return 0;
}

static void do_close(struct tty0uart_tty_serial *t_serial)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	// if (tty0uart_uart_serials[t_serial->tty->index] != NULL)
	if (tty0uart_uart_serials[t_serial->tty->index].is_open)
		tty0uart_uart_serials[t_serial->tty->index].msr = 0;

	down(&t_serial->sem);
	if (!t_serial->is_open) {
		/* port was never opened */
		goto exit;
	}

	t_serial->is_open = false;
exit:
	up(&t_serial->sem);
}

static void tty0uart_tty_close(struct tty_struct *tty, struct file *file)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (t_serial)
		do_close(t_serial);
}

static int tty0uart_tty_write(struct tty_struct *tty,
			      const unsigned char *buffer, int count)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;
	struct tty0uart_uart_serial *u_serial;
	struct uart_port *u_port;
	struct tty_port *u_t_port;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if ((!t_serial) || (!t_serial->is_open))
		return -ENODEV;

	u_serial = &tty0uart_uart_serials[tty->index];

	if ((!u_serial) || (!u_serial->is_open)) {
		printk(KERN_ERR "/dev/ttyVS%d does not open.\n", tty->index);
		return -ENODEV;
	}

	u_port = &u_serial->port;
	u_t_port = &u_port->state->port;

	tty_insert_flip_string(u_t_port, buffer, count);

	u_port->icount.rx += count;

	// spin_unlock(&u_port->lock);
	tty_flip_buffer_push(u_t_port);
	// spin_lock(&u_port->lock);

	return count;
}

static int tty0uart_tty_write_room(struct tty_struct *tty)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;
	int room = -EINVAL;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (!t_serial)
		return -ENODEV;

	down(&t_serial->sem);

	if (!t_serial->is_open)
		goto exit;

	/* calculate how much room is left in the device */
	room = 255;

exit:
	up(&t_serial->sem);
	return room;
}

#define RELEVANT_IFLAG(iflag)                                                  \
	((iflag) & (IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK))

static void tty0uart_tty_set_termios(struct tty_struct *tty,
				     struct ktermios *old_termios)
{
	unsigned int cflag;
	unsigned int iflag;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	/* check that they really want us to change something */
	if (old_termios) {
		if ((cflag == old_termios->c_cflag) &&
		    (RELEVANT_IFLAG(iflag) ==
		     RELEVANT_IFLAG(old_termios->c_iflag))) {
#ifdef SCULL_DEBUG
			printk(KERN_DEBUG " - nothing to change...\n");
#endif
			return;
		}
	}
#ifdef SCULL_DEBUG
	/* get the byte size */
	switch (cflag & CSIZE) {
	case CS5:
		printk(KERN_DEBUG " - data bits = 5\n");
		break;
	case CS6:
		printk(KERN_DEBUG " - data bits = 6\n");
		break;
	case CS7:
		printk(KERN_DEBUG " - data bits = 7\n");
		break;
	default:
	case CS8:
		printk(KERN_DEBUG " - data bits = 8\n");
		break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			printk(KERN_DEBUG " - parity = odd\n");
		else
			printk(KERN_DEBUG " - parity = even\n");
	else
		printk(KERN_DEBUG " - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		printk(KERN_DEBUG " - stop bits = 2\n");
	else
		printk(KERN_DEBUG " - stop bits = 1\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		printk(KERN_DEBUG " - RTS/CTS is enabled\n");
	else
		printk(KERN_DEBUG " - RTS/CTS is disabled\n");

	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and 
	 * stop character in the device */
	if (I_IXOFF(tty) || I_IXON(tty)) {
		unsigned char stop_char = STOP_CHAR(tty);
		unsigned char start_char = START_CHAR(tty);

		/* if we are implementing INBOUND XON/XOFF */
		if (I_IXOFF(tty))
			printk(KERN_DEBUG " - INBOUND XON/XOFF is enabled, "
					  "XON = %2x, XOFF = %2x\n",
			       start_char, stop_char);
		else
			printk(KERN_DEBUG " - INBOUND XON/XOFF is disabled\n");

		/* if we are implementing OUTBOUND XON/XOFF */
		if (I_IXON(tty))
			printk(KERN_DEBUG " - OUTBOUND XON/XOFF is enabled, "
					  "XON = %2x, XOFF = %2x\n",
			       start_char, stop_char);
		else
			printk(KERN_DEBUG " - OUTBOUND XON/XOFF is disabled\n");
	}

	/* get the baud rate wanted */
	printk(KERN_DEBUG " - baud rate = %d\n", tty_get_baud_rate(tty));
#endif
}

//static int tty0uart_tty_tiocmget(struct tty_struct *tty, struct file *file)
static int tty0uart_tty_tiocmget(struct tty_struct *tty)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;
	unsigned int result = 0;
	unsigned int msr = t_serial->msr;
	unsigned int mcr = t_serial->mcr;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	result =
		((mcr & TIOCM_DTR) ? TIOCM_DTR : 0) | /* DTR is set */
		((mcr & TIOCM_RTS) ? TIOCM_RTS : 0) | /* RTS is set */
		((mcr & TIOCM_LOOP) ? TIOCM_LOOP : 0) | /* LOOP is set */
		((msr & TIOCM_CTS) ? TIOCM_CTS : 0) | /* CTS is set */
		((msr & TIOCM_CD) ? TIOCM_CAR : 0) | /* Carrier detect is set */
		((msr & TIOCM_RI) ? TIOCM_RI : 0) | /* Ring Indicator is set */
		((msr & TIOCM_DSR) ? TIOCM_DSR : 0); /* DSR is set */

	return result;
}

//static int tty0uart_tty_tiocmset(struct tty_struct *tty, struct file *file,
static int tty0uart_tty_tiocmset(struct tty_struct *tty, unsigned int set,
				 unsigned int clear)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;
	unsigned int tty_mcr = t_serial->mcr;
	unsigned int uart_msr = 0;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	// if (tty0uart_uart_serials[t_serial->tty->index] != NULL)
	if (tty0uart_uart_serials[t_serial->tty->index].is_open)
		uart_msr = tty0uart_uart_serials[t_serial->tty->index].msr;

	//null modem connection
	if (set & TIOCM_RTS) {
		tty_mcr |= TIOCM_RTS;
		uart_msr |= TIOCM_CTS;
	}

	if (set & TIOCM_DTR) {
		tty_mcr |= TIOCM_DTR;
		uart_msr |= TIOCM_DSR;
		uart_msr |= TIOCM_CD;
	}

	if (clear & TIOCM_RTS) {
		tty_mcr &= ~TIOCM_RTS;
		uart_msr &= ~TIOCM_CTS;
	}

	if (clear & TIOCM_DTR) {
		tty_mcr &= ~TIOCM_DTR;
		uart_msr &= ~TIOCM_DSR;
		uart_msr &= ~TIOCM_CD;
	}

	/* set the new MCR value in the device */
	t_serial->mcr = tty_mcr;

	// if (tty0uart_uart_serials[t_serial->tty->index] != NULL)
	if (tty0uart_uart_serials[t_serial->tty->index].is_open)
		tty0uart_uart_serials[t_serial->tty->index].msr = uart_msr;

	return 0;
}

static int tty0uart_tty_ioctl_tiocgserial(struct tty_struct *tty,
					  unsigned int cmd, unsigned long arg)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (cmd == TIOCGSERIAL) {
		struct serial_struct tmp;

		if (!arg)
			return -EFAULT;

		memset(&tmp, 0, sizeof(tmp));

		tmp.type = t_serial->serial.type;
		tmp.line = t_serial->serial.line;
		tmp.port = t_serial->serial.port;
		tmp.irq = t_serial->serial.irq;
		tmp.flags = ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
		tmp.xmit_fifo_size = t_serial->serial.xmit_fifo_size;
		tmp.baud_base = t_serial->serial.baud_base;
		tmp.close_delay = 5 * HZ;
		tmp.closing_wait = 30 * HZ;
		tmp.custom_divisor = t_serial->serial.custom_divisor;
		tmp.hub6 = t_serial->serial.hub6;
		tmp.io_type = t_serial->serial.io_type;

		if (copy_to_user((void __user *)arg, &tmp,
				 sizeof(struct serial_struct)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int tty0uart_tty_ioctl_tiocmiwait(struct tty_struct *tty,
					 unsigned int cmd, unsigned long arg)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (cmd == TIOCMIWAIT) {
		DECLARE_WAITQUEUE(wait, current);
		struct async_icount cnow;
		struct async_icount cprev;

		cprev = t_serial->icount;
		while (1) {
			add_wait_queue(&t_serial->wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&t_serial->wait, &wait);

			/* see if a signal woke us up */
			if (signal_pending(current))
				return -ERESTARTSYS;

			cnow = t_serial->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			    ((arg & TIOCM_CD) && (cnow.dcd != cprev.dcd)) ||
			    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))) {
				return 0;
			}
			cprev = cnow;
		}
	}
	return -ENOIOCTLCMD;
}

static int tty0uart_tty_ioctl_tiocgicount(struct tty_struct *tty,
					  unsigned int cmd, unsigned long arg)
{
	struct tty0uart_tty_serial *t_serial = tty->driver_data;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (cmd == TIOCGICOUNT) {
		struct async_icount cnow = t_serial->icount;
		struct serial_icounter_struct icount;

		icount.cts = cnow.cts;
		icount.dsr = cnow.dsr;
		icount.rng = cnow.rng;
		icount.dcd = cnow.dcd;
		icount.rx = cnow.rx;
		icount.tx = cnow.tx;
		icount.frame = cnow.frame;
		icount.overrun = cnow.overrun;
		icount.parity = cnow.parity;
		icount.brk = cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int tty0uart_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
			      unsigned long arg)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	switch (cmd) {
	case TIOCGSERIAL:
		return tty0uart_tty_ioctl_tiocgserial(tty, cmd, arg);
	case TIOCMIWAIT:
		return tty0uart_tty_ioctl_tiocmiwait(tty, cmd, arg);
	case TIOCGICOUNT:
		return tty0uart_tty_ioctl_tiocgicount(tty, cmd, arg);
	}

	return -ENOIOCTLCMD;
}

static struct tty_operations serial_ops = {
	.open = tty0uart_tty_open,
	.close = tty0uart_tty_close,
	.write = tty0uart_tty_write,
	.write_room = tty0uart_tty_write_room,
	.set_termios = tty0uart_tty_set_termios,
	.tiocmget = tty0uart_tty_tiocmget,
	.tiocmset = tty0uart_tty_tiocmset,
	.ioctl = tty0uart_tty_ioctl,
};

static struct tty_driver *tty0uart_tty_driver;

static int tty0uart_tty_init(void)
{
	int ret;
	int i;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (pairs > MAX_PORT_NUM)
		pairs = MAX_PORT_NUM;
	if (pairs < 1)
		pairs = 1;

	tty0uart_tty_port =
		kmalloc(pairs * sizeof(struct tty_port), GFP_KERNEL);
	tty0uart_tty_serials = kmalloc(
		pairs * sizeof(struct tty0uart_tty_serial *), GFP_KERNEL);

	for (i = 0; i < pairs; i++)
		tty0uart_tty_serials[i] = NULL;

	/* allocate the tty driver */
	tty0uart_tty_driver = alloc_tty_driver(pairs);
	if (!tty0uart_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	tty0uart_tty_driver->owner = THIS_MODULE;
	tty0uart_tty_driver->driver_name = "tty0uart_tty";
	tty0uart_tty_driver->name = "ttyvs";
	/* no more devfs subsystem */
	tty0uart_tty_driver->major = TTY0UART_MAJOR;
	tty0uart_tty_driver->minor_start = TTY0UART_MINOR;
	tty0uart_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty0uart_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty0uart_tty_driver->flags =
		TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW;
	/* no more devfs subsystem */
	tty0uart_tty_driver->init_termios = tty_std_termios;
	tty0uart_tty_driver->init_termios.c_iflag = 0;
	tty0uart_tty_driver->init_termios.c_oflag = 0;
	tty0uart_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	tty0uart_tty_driver->init_termios.c_lflag = 0;
	tty0uart_tty_driver->init_termios.c_ispeed = 38400;
	tty0uart_tty_driver->init_termios.c_ospeed = 38400;

	tty_set_operations(tty0uart_tty_driver, &serial_ops);

	for (i = 0; i < pairs; i++) {
		tty_port_init(&tty0uart_tty_port[i]);
		tty_port_link_device(&tty0uart_tty_port[i], tty0uart_tty_driver,
				     i);
	}

	ret = tty_register_driver(tty0uart_tty_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register tty0uart_tty_driver.\n");
		put_tty_driver(tty0uart_tty_driver);
		return ret;
	}

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");
	return ret;
}

static void tty0uart_tty_exit(void)
{
	struct tty0uart_tty_serial *t_serial;
	int i;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	for (i = 0; i < pairs; ++i) {
		tty_port_destroy(&tty0uart_tty_port[i]);
		tty_unregister_device(tty0uart_tty_driver, i);
	}

	tty_unregister_driver(tty0uart_tty_driver);

	/* shut down all of the timers and free the memory */
	for (i = 0; i < pairs; ++i) {
		t_serial = tty0uart_tty_serials[i];
		if (t_serial) {
			/* close the port */
			if (t_serial->is_open)
				do_close(t_serial);

			/* shut down our timer and free the memory */
			kfree(t_serial);
			tty0uart_tty_serials[i] = NULL;
		}
	}
	kfree(tty0uart_tty_port);
	kfree(tty0uart_tty_serials);
}

static struct uart_driver tty0uart_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "tty0uart_uart",
	.dev_name = "ttyVS",
	.major = TTY0UART_MAJOR,
	.minor = TTY0UART_MINOR,
};

static inline struct tty0uart_uart_serial *
to_tty0uart_uart_serial(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	return container_of(port, struct tty0uart_uart_serial, port);
}

static u_int tty0uart_uart_tx_empty(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	return TIOCSER_TEMT;
}

static void tty0uart_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct tty0uart_uart_serial *u_serial = to_tty0uart_uart_serial(port);
	unsigned int uart_mcr = u_serial->mcr;
	unsigned int tty_msr = 0;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (tty0uart_tty_serials[port->line] != NULL)
		if (tty0uart_tty_serials[port->line]->is_open)
			tty_msr = tty0uart_tty_serials[port->line]->msr;

	//null modem connection
	if (mctrl & TIOCM_RTS) {
		uart_mcr |= TIOCM_RTS;
		tty_msr |= TIOCM_CTS;
	} else {
		uart_mcr &= ~TIOCM_RTS;
		tty_msr &= ~TIOCM_CTS;
	}

	if (mctrl & TIOCM_DTR) {
		uart_mcr |= TIOCM_DTR;
		tty_msr |= TIOCM_DSR;
		tty_msr |= TIOCM_CD;
	} else {
		uart_mcr &= ~TIOCM_DTR;
		tty_msr &= ~TIOCM_DSR;
		tty_msr &= ~TIOCM_CD;
	}

	/* set the new MCR value in the device */
	u_serial->mcr = uart_mcr;

	if (tty0uart_tty_serials[port->line] != NULL)
		if (tty0uart_tty_serials[port->line]->is_open)
			tty0uart_tty_serials[port->line]->msr = tty_msr;
}

static unsigned int tty0uart_uart_get_mctrl(struct uart_port *port)
{
	struct tty0uart_uart_serial *u_serial = to_tty0uart_uart_serial(port);
	unsigned int result = 0;
	unsigned int msr = u_serial->msr;
	unsigned int mcr = u_serial->mcr;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	result =
		((mcr & TIOCM_DTR) ? TIOCM_DTR : 0) | /* DTR is set */
		((mcr & TIOCM_RTS) ? TIOCM_RTS : 0) | /* RTS is set */
		((mcr & TIOCM_LOOP) ? TIOCM_LOOP : 0) | /* LOOP is set */
		((msr & TIOCM_CTS) ? TIOCM_CTS : 0) | /* CTS is set */
		((msr & TIOCM_CD) ? TIOCM_CAR : 0) | /* Carrier detect is set */
		((msr & TIOCM_RI) ? TIOCM_RI : 0) | /* Ring Indicator is set */
		((msr & TIOCM_DSR) ? TIOCM_DSR : 0); /* DSR is set */

	return result;
}

static void tty0uart_uart_stop_tx(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct tty0uart_tty_serial *t_serial = tty0uart_tty_serials[port->line];
	struct tty_port *t_port;
	size_t count;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if ((!t_serial) || (!t_serial->is_open)) {
		printk(KERN_ERR "/dev/ttyvs%d does not open.\n", port->line);
		xmit->tail = xmit->head;
		return;
	}

	if (uart_circ_empty(xmit))
		return;

	down(&t_serial->sem);

	t_port = t_serial->tty->port;

	for (;;) {
		if (xmit->head < xmit->tail) {
			count = UART_XMIT_SIZE - xmit->tail;
			tty_insert_flip_string(t_port, xmit->buf + xmit->tail,
					       count);
			xmit->tail = 0;
			port->icount.tx += count;
		}

		if (xmit->tail < xmit->head) {
			count = xmit->head - xmit->tail;
			tty_insert_flip_string(t_port, xmit->buf + xmit->tail,
					       count);
			xmit->tail =
				(xmit->tail + count) & (UART_XMIT_SIZE - 1);
			port->icount.tx += count;
		}

		if (uart_circ_empty(xmit))
			break;
	}

	tty_flip_buffer_push(t_port);

	up(&t_serial->sem);
}

static void tty0uart_uart_stop_rx(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_enable_ms(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_break_ctl(struct uart_port *port, int ctl)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static int tty0uart_uart_startup(struct uart_port *port)
{
	struct tty0uart_uart_serial *u_serial = to_tty0uart_uart_serial(port);
	unsigned int uart_msr = 0;
	unsigned int tty_mcr = 0;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	u_serial->is_open = true;

	if (tty0uart_tty_serials[port->line] != NULL)
		if (tty0uart_tty_serials[port->line]->is_open)
			tty_mcr = tty0uart_tty_serials[port->line]->mcr;

	//null modem connection
	if ((tty_mcr & TIOCM_RTS) == TIOCM_RTS) {
		uart_msr |= TIOCM_CTS;
	}

	if ((tty_mcr & TIOCM_DTR) == TIOCM_DTR) {
		uart_msr |= TIOCM_DSR;
		uart_msr |= TIOCM_CD;
	}

	u_serial->msr = uart_msr;
	u_serial->mcr = 0;

	return 0;
}

static void tty0uart_uart_shutdown(struct uart_port *port)
{
	struct tty0uart_uart_serial *u_serial = to_tty0uart_uart_serial(port);

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (tty0uart_tty_serials[port->line] != NULL)
		if (tty0uart_tty_serials[port->line]->is_open)
			tty0uart_tty_serials[port->line]->msr = 0;

	u_serial->is_open = false;
}

static void tty0uart_uart_flush_buffer(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_set_termios(struct uart_port *port,
				      struct ktermios *new,
				      struct ktermios *old)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_set_ldisc(struct uart_port *port,
				    struct ktermios *termios)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static void tty0uart_uart_pm(struct uart_port *port, unsigned int state,
			     unsigned int oldstate)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static const char *tty0uart_uart_type(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	return (port->type == PORT_TTY0UART) ? "tty0uart_uart" : NULL;
}

static void tty0uart_uart_release_port(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);
}

static int tty0uart_uart_request_port(struct uart_port *port)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	return 0;
}

static void tty0uart_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_TTY0UART;
		tty0uart_uart_request_port(port);
	}
}

static int tty0uart_uart_verify_port(struct uart_port *port,
				     struct serial_struct *ser)
{
	int ret = 0;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	if (port->type != PORT_TTY0UART)
		ret = -EINVAL;

	return ret;
}

static const struct uart_ops tty0uart_uart_ops = {
	.tx_empty = tty0uart_uart_tx_empty,
	.set_mctrl = tty0uart_uart_set_mctrl,
	.get_mctrl = tty0uart_uart_get_mctrl,
	.stop_tx = tty0uart_uart_stop_tx,
	.start_tx = tty0uart_uart_start_tx,
	.stop_rx = tty0uart_uart_stop_rx,
	.enable_ms = tty0uart_uart_enable_ms,
	.break_ctl = tty0uart_uart_break_ctl,
	.startup = tty0uart_uart_startup,
	.shutdown = tty0uart_uart_shutdown,
	.flush_buffer = tty0uart_uart_flush_buffer,
	.set_termios = tty0uart_uart_set_termios,
	.set_ldisc = tty0uart_uart_set_ldisc,
	.pm = tty0uart_uart_pm,
	.type = tty0uart_uart_type,
	.release_port = tty0uart_uart_release_port,
	.request_port = tty0uart_uart_request_port,
	.config_port = tty0uart_uart_config_port,
	.verify_port = tty0uart_uart_verify_port,
};

static void tty0uart_uart_init_port(struct tty0uart_uart_serial *serial,
				    struct platform_device *pdev)
{
	struct uart_port *port = &serial->port;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &tty0uart_uart_ops;
	port->fifosize = 1;
	port->line = pdev->id;
	port->dev = &pdev->dev;
	port->type = PORT_TTY0UART;
}

static int tty0uart_uart_serial_probe(struct platform_device *pdev)
{
	struct tty0uart_uart_serial *serial = &tty0uart_uart_serials[pdev->id];
	int ret = -ENODEV;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	tty0uart_uart_init_port(serial, pdev);
	ret = uart_add_one_port(&tty0uart_uart_driver, &serial->port);

	platform_set_drvdata(pdev, serial);
	return ret;
}

static int tty0uart_uart_serial_remove(struct platform_device *pdev)
{
	struct tty0uart_uart_serial *serial = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	platform_set_drvdata(pdev, NULL);
	uart_remove_one_port(&tty0uart_uart_driver, &serial->port);
	return 0;
}

#define tty0uart_uart_serial_suspend NULL
#define tty0uart_uart_serial_resume NULL

static struct platform_driver tty0uart_uart_serial_driver = {
	.probe		= tty0uart_uart_serial_probe,
	.remove		= tty0uart_uart_serial_remove,
	.suspend	= tty0uart_uart_serial_suspend,
	.resume		= tty0uart_uart_serial_resume,
	.driver		= {
		.name		= "tty0uart_serial",
		.owner = THIS_MODULE,
	},
};

static struct platform_device **tty0uart_uart_devices;

static int tty0uart_uart_init(void)
{
	int ret;
	int i;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	tty0uart_uart_devices =
		kmalloc(pairs * sizeof(struct platform_device *), GFP_KERNEL);

	for (i = 0; i < pairs; ++i) {
		/* 
		 * Calling platform_device_unregister()
		 * will automatically free it by release()
		 */
		tty0uart_uart_devices[i] =
			platform_device_alloc("tty0uart_serial", i);
		ret = platform_device_add(tty0uart_uart_devices[i]);
		if (ret) {
			while (i--) {
				platform_device_unregister(
					tty0uart_uart_devices[i]);
			}
			printk(KERN_ERR
			       "Failed to register tty0uart_uart_devices\n");
			return ret;
		}
	}

	tty0uart_uart_driver.nr = pairs;

	ret = uart_register_driver(&tty0uart_uart_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register tty0uart_uart_driver\n");
		for (i = 0; i < pairs; ++i) {
			platform_device_unregister(tty0uart_uart_devices[i]);
		}
		return ret;
	}

	ret = platform_driver_register(&tty0uart_uart_serial_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register tty0_uart_serial_driver\n");
		uart_unregister_driver(&tty0uart_uart_driver);
		for (i = 0; i < pairs; ++i) {
			platform_device_unregister(tty0uart_uart_devices[i]);
		}
	}

	return ret;
}

static void tty0uart_uart_exit(void)
{
	int i;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	platform_driver_unregister(&tty0uart_uart_serial_driver);

	uart_unregister_driver(&tty0uart_uart_driver);

	for (i = 0; i < pairs; ++i) {
		platform_device_unregister(tty0uart_uart_devices[i]);
	}

	kfree(tty0uart_uart_devices);
}

static int __init tty0uart_init(void)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	ret = tty0uart_tty_init();
	if (ret) {
		printk(KERN_ERR "Failed to initialize tty0uart_tty\n");
		return ret;
	}

	ret = tty0uart_uart_init();
	if (ret) {
		printk(KERN_ERR "Failed to initialize tty0uart_uart\n");
		return ret;
	}

	return ret;
}

static void __exit tty0uart_exit(void)
{
	printk(KERN_DEBUG "%s\n", __FUNCTION__);

	tty0uart_tty_exit();
	tty0uart_uart_exit();
}

module_init(tty0uart_init);
module_exit(tty0uart_exit);

/* Module information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);