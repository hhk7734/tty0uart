// SPDX-License-Identifier: GPL-2.0
/*
 * tty0uart - Null-modem emulator connecting virtual tty to virtual UART 
 * Copyright (C) 2019  Hyeonki Hong <hhk7734@gmail.com>
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>

static int __init tty0uart_init(void)
{
	int ret;

	return ret;
}

static void __exit tty0uart_exit(void)
{
}

device_initcall(tty0uart_init);
module_exit(tty0uart_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hyeonki Hong <hhk7734@gmail.com>");
MODULE_DESCRIPTION("Null-modem emulator connecting virtual tty to virtual UART");