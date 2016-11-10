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



#undef PRINT_DEBUG
#ifdef LPRF_DEBUG
	#define PRINT_DEBUG(fmt, args...) printk( KERN_DEBUG "lprf: " fmt, ## args)
#else
	#define PRINT_DEBUG(fmt, args...) /* not debugging: nothing */
#endif












#endif // _LPRF_H_
