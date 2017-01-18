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

// Remove comment to show debug outputs
// #define LPRF_DEBUG

/*
 * Debug macro for timing critical parts like polling. Activating
 * this macro will lead to a lot of debug messages and should only
 * be used with decreased time resolution for polling (RX_POLLING_INTERVAL)
 */
// #define LPRF_DEBUG_KRIT

/* This is the maximum length of a received IEEE frame including
 * synchronization information. 127bytes payload + 5 bytes synchronization
 * header + 1 byte physical header + 2 extra bytes just to be safe and get no
 * problems during post processing like shifting the received bytes. */
#define FRAME_LENGTH 135
#define MAX_SPI_BUFFER_SIZE (FRAME_LENGTH + 2)
#define KBIT_RATE 2000

#define RX_POLLING_INTERVAL ktime_set(0, 5000000)
#define RX_RX_INTERVAL ktime_set(0, 600000)
#define TX_RX_INTERVAL ktime_set(0, 600000)
#define RETRY_INTERVAL ktime_set(0, 100000)

#define REGR 0x80 // Register read access command
#define REGW 0xc0 // Register write access command
#define FRMR 0x20 // Frame read access command
#define FRMW 0x60 // Frame write access command

#define PHY_SM_STATUS(phy_status)   (((phy_status) & 0xe0) >> 5)
#define PHY_SM_ENABLE(phy_status)   (((phy_status) & 0x10) >> 4)
#define PHY_FIFO_EMPTY(phy_status)  (((phy_status) & 0x08) >> 3)
#define PHY_FIFO_FULL(phy_status)   (((phy_status) & 0x04) >> 2)

#define PHY_SM_DEEPSLEEP            0x01
#define PHY_SM_SLEEP                0x02
#define PHY_SM_BUSY                 0x03
#define PHY_SM_TX_RDY               0x04
#define PHY_SM_SENDING              0x05
#define PHY_SM_RX_RDY               0x06
#define PHY_SM_RECEIVING            0x07

#define BIT24_H_BYTE(c) (((c) & 0xFF0000) >> 16)
#define BIT24_M_BYTE(c) (((c) & 0x00FF00) >> 8)
#define BIT24_L_BYTE(c) ((c) & 0x0000FF)

#undef PRINT_DEBUG
#ifdef LPRF_DEBUG
	#define PRINT_DEBUG(fmt, args...) printk( KERN_DEBUG "lprf: " fmt "\n", ## args)
#else
	#define PRINT_DEBUG(fmt, args...)
#endif

#undef PRINT_KRIT
#ifdef LPRF_DEBUG_KRIT
	#define PRINT_KRIT(fmt, args...) printk( KERN_DEBUG "lprf: " fmt "\n", ## args)
#else
	#define PRINT_KRIT(fmt, args...)
#endif


#define RETURN_ON_ERROR(f) ret = (f); \
	if (ret) { \
		return ret; \
	}


#endif // _LPRF_H_
