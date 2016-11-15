/* this is a mixture from at86rf230.c and scull-main
 * all references to at86rf230 are changed to lprf
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
 * Works on kernel version 4.4.15+
 *
 * Written by:
 * Tilman Sinning <tilman.sinning@rwth-aachen.de>
 * Moritz Schrey <mschrey@ias.rwth-aachen.de>
 * Jan Richter-Brockmann <jan.richter-brockmann@rwth-aachen.de>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Alexander Smirnov <alex.bluesman.smirnov@gmail.com>
 * Alexander Aring <aar@pengutronix.de>
 * Alessandro Rubini
 * Jonathan Corbet
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include "lprf.h"
#include "lprf_registers.h"


struct lprf_platform_data {
	int some_custom_value;
};

static const struct regmap_config lprf_regmap_spi_config = {
	.reg_bits = 16,
	.reg_stride = 1,
	.pad_bits = 0,
	.val_bits = 8,
	.fast_io = 0, // use mutex or spinlock for locking
	.read_flag_mask = 0x80,
	.write_flag_mask = 0xc0,
	.use_single_rw = 1, // single read write commands or bulk read write
	.can_multi_write = 0,
	.cache_type = REGCACHE_NONE,
};

static inline int lprf_read_register(struct lprf *lprf, unsigned int address, unsigned int *value)
{
	int ret = 0;
	ret = regmap_read(lprf->regmap, address, value);
	PRINT_DEBUG( "Read value %X from LPRF register %X\n", *value, address);
	return ret;
}

static inline int lprf_write_register(struct lprf *lprf, unsigned int address, unsigned int value)
{
	int ret = 0;
	ret = regmap_write(lprf->regmap, address, value);
	PRINT_DEBUG( "Write value %X to LPRF register %X\n", value, address);
	return ret;
}

static inline int lprf_read_subreg(struct lprf *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int *data)
{
	int ret = 0;
	ret = lprf_read_register(lprf, addr, data);
	if (ret)
		*data = (*data & mask) >> shift;
	return ret;
}

static inline int lprf_write_subreg(struct lprf *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int data)
{
	return regmap_update_bits(lprf->regmap, addr, mask, data << shift);
}


/**
 * Detect LPRF Chip
 *
 * @lprf: lprf struct containing hardware information of lprf chip
 *
 * returns 0 on success, a negative error number on error.
 */
static int lprf_detect_device(struct lprf *lprf)
{
	int rx_buf = 0, ret=0, chip_id = 0;

	ret = lprf_read_register(lprf, RG_CHIP_ID_H, &rx_buf);
	if(ret)
		return ret;
	chip_id |= (rx_buf << 8);

	ret = lprf_read_register(lprf, RG_CHIP_ID_L, &rx_buf);
	if(ret)
		return ret;
	chip_id |= rx_buf;

	if (chip_id != 0x1a51)
	{
		PRINT_DEBUG("Chip with invalid Chip ID %X found\n", chip_id);
		return -ENODEV;
	}
	PRINT_INFO("LPRF Chip found with Chip ID %X\n", chip_id);
	return 0;

}

static int lprf_probe(struct spi_device *spi)
{
	u32 custom_value = 0;
	int ret = 0;
	struct lprf_platform_data *pdata = 0;
	struct lprf *lprf;
	PRINT_DEBUG( "call lprf_probe\n");

	pdata = spi->dev.platform_data;

	// Get platform data
	if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) {
		if (!pdata)
			return -ENOENT;
	}

	PRINT_DEBUG( "successfully parsed platform data\n");


	ret = of_property_read_u32(spi->dev.of_node, "some-custom-value", &custom_value);
	PRINT_DEBUG( "returned value:\t%d, custom value:\t%d\n", ret, custom_value);


	lprf = kmalloc(sizeof(*lprf), GFP_KERNEL);
	lprf->spi_device = spi;

	lprf->regmap = devm_regmap_init_spi(spi, &lprf_regmap_spi_config);
	if (IS_ERR(lprf->regmap)) {
		PRINT_DEBUG( "Failed to allocate register map: %d\n", (int) PTR_ERR(lprf->regmap) );
	}

	PRINT_DEBUG( "successfully initialized Register map\n");

	ret = lprf_detect_device(lprf);
	if(ret)
		goto free_device;

free_device:
	kfree(lprf);

	return ret;
}

static int lprf_remove(struct spi_device *spi)
{
	PRINT_DEBUG( "call lprf_remove\n");
	return 0;
}

static const struct of_device_id lprf_of_match[] = {
	{ .compatible = "ias,lprf", },
	{ },
};
MODULE_DEVICE_TABLE(of, lprf_of_match);

static const struct spi_device_id lprf_device_id[] = {
	{ .name = "lprf", },
	{ },
};
MODULE_DEVICE_TABLE(spi, lprf_device_id);

static struct spi_driver lprf_driver = {
	.id_table = lprf_device_id,
	.driver = {
		.of_match_table = of_match_ptr(lprf_of_match),
		.name	= "lprf",
	},
	.probe      = lprf_probe,
	.remove     = lprf_remove,
};

module_spi_driver(lprf_driver);

MODULE_DESCRIPTION("LPRF RX Driver");
MODULE_LICENSE("GPL v2");



