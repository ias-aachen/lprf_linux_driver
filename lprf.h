/*
 * IAS LPRF driver
 *
 * Copyright (C) 2015 IAS RWTH Aachen
 * Adapted from  AT86RF230 driver, Copyright (C) 2009-2012 Siemens AG
 * Also based on scull driver from "Linux Device Drivers"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details
 *
 * Written by:
 * Tilman Sinning <tilman.sinning@rwth-aachen.de
 * Moritz Schrey <mschrey@ias.rwth-aachen.de>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Alexander Smirnov <alex.bluesman.smirnov@gmail.com>
 * Alexander Aring <aar@pengutronix.de>
 * Alessandro Rubini
 * Jonathan Corbet
 */

#ifndef _LPRF_H_
#define _LPRF_H_

#define LPRF_DEBUG  // Remove comment to show debug outputs
#define LPRF_INFO  // Remove comment to show info outputs
#define LPRF_MAX_BUF 256
#define FRAME_LENGTH 32
#define KBIT_RATE 2000

#define COUNTER_H_BYTE(c) (((c) & 0xFF0000) >> 16)
#define COUNTER_M_BYTE(c) (((c) & 0x00FF00) >> 8)
#define COUNTER_L_BYTE(c) ((c) & 0x0000FF)

#undef PRINT_DEBUG
#ifdef LPRF_DEBUG
	#define PRINT_DEBUG(fmt, args...) printk( KERN_DEBUG "lprf: " fmt "\n", ## args)
#else
	#define PRINT_DEBUG(fmt, args...)
#endif

#undef PRINT_INFO
#ifdef LPRF_INFO
	#define PRINT_INFO(fmt, args...) printk( KERN_INFO "lprf: " fmt "\n", ## args)
#else
	#define PRINT_INFO(fmt, args...)
#endif







struct lprf {
	struct spi_device *spi_device;
	struct regmap *regmap;
	struct mutex mutex;
	struct cdev my_char_dev;
	struct spi_message spi_message;
	DECLARE_KFIFO_PTR(spi_buffer, uint8_t);
	struct ieee802154_hw *ieee802154_hw;
	atomic_t is_reading_from_fifo;
	wait_queue_head_t wait_for_fifo_data;
};


#endif // _LPRF_H_
