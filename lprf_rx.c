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
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>
#include <linux/kfifo.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include "lprf.h"
#include "lprf_registers.h"


static int init_lprf_hardware(struct lprf *lprf);
static inline int __lprf_read_frame(struct lprf *lprf);

static int lprf_start_ieee802154(struct ieee802154_hw *hw);
static void lprf_stop_ieee802154(struct ieee802154_hw *hw);
static int lprf_set_ieee802154_channel(struct ieee802154_hw *hw, u8 page, u8 channel);
static int lprf_set_ieee802154_addr_filter(struct ieee802154_hw *hw,
		    struct ieee802154_hw_addr_filt *filt,
		    unsigned long changed);
static int lprf_xmit_ieee802154_async(struct ieee802154_hw *hw, struct sk_buff *skb);
static int lprf_ieee802154_energy_detection(struct ieee802154_hw *hw, u8 *level);


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

static const struct ieee802154_ops  ieee802154_lprf_callbacks = {
	.owner = THIS_MODULE,
	.start = lprf_start_ieee802154,
	.stop = lprf_stop_ieee802154,
	.xmit_sync = 0, // should not be used anymore
	.xmit_async = lprf_xmit_ieee802154_async, // needs to be implemented for Tx
	.ed = lprf_ieee802154_energy_detection, // can we support this? Can not be disabled in hw_flags
	.set_channel = lprf_set_ieee802154_channel,
	.set_hw_addr_filt = lprf_set_ieee802154_addr_filter,
	.set_txpower = 0,	// needs to be implemented for tx (seems to work without)
	.set_lbt = 0,		// Disabled in hw_flags
	.set_cca_mode = 0,	// can we support this? Can not be disabled in hw_flags (seems to work without)
	.set_cca_ed_level = 0,	// can we support this? Can not be disabled in hw_flags (seems to work without)
	.set_csma_params = 0,	// Disabled in hw_flags
	.set_frame_retries = 0,	// Disabled in hw_flags
	.set_promiscuous_mode = 0, // Disabled in hw_flags
};

/**
 * Calculates the RX Length counter based on the datarate and frame_length (assuming 32MHz clock speed on chip)
 *
 * @kbitrate: over the air daterate in kb/s
 * @frame_length: frame length in bytes
 */
static inline int get_rx_length_counter_H(int kbit_rate, int frame_length)
{
	const int chip_speed_kHz = 32000;
	return 8 * frame_length * chip_speed_kHz / kbit_rate + 4 * chip_speed_kHz / kbit_rate;
}

/**
 * Reverses the bit order of byte
 */
static inline void reverse_bit_order(uint8_t *byte)
{
	*byte = ((*byte & 0xaa) >> 1) | ((*byte & 0x55) << 1);
	*byte = ((*byte & 0xcc) >> 2) | ((*byte & 0x33) << 2);
	*byte = (*byte >> 4) | (*byte << 4);
}

static inline int lprf_read_register(struct lprf *lprf, unsigned int address, unsigned int *value)
{
	int ret = 0;
	ret = regmap_read(lprf->regmap, address, value);
	// PRINT_DEBUG( "Read value %X from LPRF register %X", *value, address);
	return ret;
}

static inline int lprf_write_register(struct lprf *lprf, unsigned int address, unsigned int value)
{
	int ret = 0;
	ret = regmap_write(lprf->regmap, address, value);
	// PRINT_DEBUG( "Write value %X to LPRF register %X", value, address);
	return ret;
}

static inline int lprf_read_subreg(struct lprf *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int *data)
{
	int ret = 0;
	ret = lprf_read_register(lprf, addr, data);
	*data = (*data & mask) >> shift;
	return ret;
}

static inline int lprf_write_subreg(struct lprf *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int data)
{
	return regmap_update_bits(lprf->regmap, addr, mask, data << shift);
}

static void preprocess_received_data(uint8_t *data, int length)
{
	int i = 0;
	for (i = 0; i < length; ++i)
	{
		reverse_bit_order(&data[i]);
	}
}

static void __lprf_read_frame_complete(void *context)
{
	struct lprf *lprf = 0;
	struct spi_transfer *transfer = 0;
	uint8_t *rx_buf = 0;
	uint8_t *data_buf = 0;
	uint8_t *tx_buf = 0;
	uint8_t status = 0;
	int length = 0;
	int bytes_copied = 0;

	PRINT_DEBUG("Spi transfer completed");

	lprf = context;
	transfer = list_entry(lprf->spi_message.transfers.next, struct spi_transfer, transfer_list);
	rx_buf = (uint8_t*) transfer->rx_buf;
	tx_buf = (uint8_t*) transfer->tx_buf;
	data_buf = rx_buf + 2; // first two bytes of rx_buf do not contain data

	status = rx_buf[0];
	length = rx_buf[1];

	preprocess_received_data(data_buf, length);
	bytes_copied = kfifo_in(&lprf->spi_buffer, data_buf, length);

	wake_up_interruptible(&lprf->wait_for_fifo_data);
	PRINT_DEBUG("Copied %d bytes of %d received bytes to ring buffer", bytes_copied, length);

	if (length > 0) // LPRF FIFO not empty -> get more data
	{
		// note that buffers get reused without deleting old data
		__lprf_read_frame(lprf);
		return;
	}

	atomic_set(&lprf->is_reading_from_fifo, 0);
	spi_transfer_del(transfer);
	kfree(rx_buf);
	kfree(tx_buf);
	kfree(transfer);

}

/**
 * Execute one Frame Read Access Command on LPRF-Chip
 *
 * @lprf: lprf structure with hardware information
 *
 * @rx_buf: dynamically allocated buffer for the received data. Make sure that rx_buf is at least
 * LPRF_MAX_BUF + 2 (258 bytes) long (maximum number of received bytes per read access)
 *
 * @tx_buf: dynamically allocated buffer for transmitted data. Has to be the same length as rx_buf.
 *
 * returns zero on success or negative error code. Note that the freeing of the rx and tx buffers
 * is taken care of by __lprf_read_frame_complete().
 */
static inline int __lprf_read_frame(struct lprf *lprf)
{
	uint8_t *tx_buf = 0;
	tx_buf = (uint8_t*) list_entry(lprf->spi_message.transfers.next, struct spi_transfer, transfer_list)->tx_buf;

	tx_buf[0] = 0x20;  // Frame_Read_commmand

	PRINT_DEBUG("Do Frame Read");

	return spi_async(lprf->spi_device, &lprf->spi_message);
}

static int read_lprf_fifo(struct lprf *lprf)
{
	uint8_t *rx_buf = 0;
	uint8_t *tx_buf = 0;
	struct spi_transfer *transfer = 0;

	// Freeing memory is taken care of by __lprf_read_frame_complete
	rx_buf = kzalloc(LPRF_MAX_BUF + 2, GFP_KERNEL);
	if (rx_buf == 0)
		return -ENOMEM;

	tx_buf = kzalloc(LPRF_MAX_BUF + 2, GFP_KERNEL);
	if (tx_buf == 0)
		goto free_rx_buf;

	transfer = kzalloc(sizeof(*transfer), GFP_KERNEL);
	if (transfer == 0)
		goto free_tx_buf;

	// todo better to do this spi message initialization once in the probe function
	spi_message_init(&lprf->spi_message);
	lprf->spi_message.complete = __lprf_read_frame_complete;
	lprf->spi_message.context = lprf;
	lprf->spi_message.spi = lprf->spi_device;

	transfer->rx_buf = rx_buf;
	transfer->tx_buf = tx_buf;
	transfer->len = LPRF_MAX_BUF + 2;
	spi_message_add_tail(transfer, &lprf->spi_message);

	__lprf_read_frame(lprf);
	atomic_set(&lprf->is_reading_from_fifo, 1);

	return 0;

	PRINT_DEBUG("Error during memory allocation in read_lprf_fifo");
free_tx_buf:
	kfree(tx_buf);
free_rx_buf:
	kfree(rx_buf);

	return -ENOMEM;
}


static int lprf_start_ieee802154(struct ieee802154_hw *hw)
{
	PRINT_DEBUG("Called unimplemented function lprf_start_ieee802154()");
	return 0;
}

static void lprf_stop_ieee802154(struct ieee802154_hw *hw)
{

	PRINT_DEBUG("Called unimplemented function lprf_stop_ieee802154()");
}

static int lprf_set_ieee802154_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{

	PRINT_DEBUG("Called unimplemented function lprf_set_ieee802154_channel()");
	return 0;
}

static int lprf_set_ieee802154_addr_filter(struct ieee802154_hw *hw,
		    struct ieee802154_hw_addr_filt *filt,
		    unsigned long changed)
{

	PRINT_DEBUG("Called unimplemented function lprf_set_ieee802154_addr_filter()");
	return 0;
}

static int lprf_xmit_ieee802154_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	PRINT_DEBUG("Called unimplemented function lprf_xmit_ieee802154_async()");
	return 0;
}

static int lprf_ieee802154_energy_detection(struct ieee802154_hw *hw, u8 *level)
{
	PRINT_DEBUG("Called unimplemented function lprf_ieee802154_energy_detection()");
	return 0;
}


int lprf_open_char_device(struct inode *inode, struct file *filp)
{
	struct lprf *lprf = 0;
	int ret = 0;
	lprf = container_of(inode->i_cdev, struct lprf, my_char_dev);    //http://stackoverflow.com/questions/15832301/understanding-container-of-macro-in-linux-kerne0;

	filp->private_data = lprf;

	 // todo: put this in a function with better error management
	lprf_write_subreg(lprf, SR_DEM_RESETB, 0);
	lprf_write_subreg(lprf, SR_DEM_RESETB, 1);
	lprf_write_subreg(lprf, SR_FIFO_RESETB, 0);
	lprf_write_subreg(lprf, SR_FIFO_RESETB, 1);
	lprf_write_subreg(lprf, SR_SM_RESETB, 0);
	lprf_write_subreg(lprf, SR_SM_RESETB, 1);
	lprf_write_subreg(lprf, SR_SM_COMMAND, STATE_CMD_RX);
	lprf_write_subreg(lprf, SR_SM_COMMAND, STATE_CMD_NONE);

	ret = read_lprf_fifo(lprf);
	if (ret)
		return ret;

	PRINT_DEBUG("LPRF successfully opened as char device");
	return 0;


}


int lprf_release_char_device(struct inode *inode, struct file *filp)
{
	struct lprf *lprf;
	lprf = container_of(inode->i_cdev, struct lprf, my_char_dev);


	PRINT_DEBUG("LPRF char device successfully released");
	return 0;
}

ssize_t lprf_read_char_device(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int buffer_length = 0;
	int bytes_to_copy = 0;
	int bytes_copied = 0;
	int ret = 0;
	struct lprf *lprf = filp->private_data;

	PRINT_DEBUG("Read from user space with buffer size %d requested", count);

	if( kfifo_is_empty(&lprf->spi_buffer) &&
			atomic_read(&lprf->is_reading_from_fifo))
	{
		PRINT_DEBUG("Read_char_device goes to sleep because of empty buffer.");
		ret = wait_event_interruptible_timeout(
				lprf->wait_for_fifo_data,
				!(kfifo_is_empty(&lprf->spi_buffer) &&
				atomic_read(&lprf->is_reading_from_fifo)),
				HZ);
		if (ret < 0)
			return ret;
		PRINT_DEBUG("Returned from sleep in read_char_device.");
	}

	buffer_length = kfifo_len(&lprf->spi_buffer);
	bytes_to_copy = (count < buffer_length) ? count : buffer_length;

	ret = kfifo_to_user(&lprf->spi_buffer, buf, bytes_to_copy, &bytes_copied);
	if(ret)
		return ret;

	PRINT_DEBUG("%d/%d bytes copied to user.",
			bytes_copied, buffer_length);

	return bytes_copied;
}


static int allocate_spi_buffer(struct lprf *lprf)
{
	int ret = 0;

	ret = kfifo_alloc(&lprf->spi_buffer, 2048, GFP_KERNEL);
	if (ret)
		return ret;
	return 0;
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
		PRINT_DEBUG("Chip with invalid Chip ID %X found", chip_id);
		return -ENODEV;
	}
	PRINT_INFO("LPRF Chip found with Chip ID %X", chip_id);
	return 0;

}

/**
 * Defines the callback functions for file operations when the lprf device is used
 * with the char driver interface
 */
static const struct file_operations lprf_fops = {
	.owner =             THIS_MODULE,
	.read =              lprf_read_char_device,
	.write =             0, //  needs to be implemented for tx case
	.unlocked_ioctl =    0, // can be implemented for additional control
	.open =              lprf_open_char_device,
	.release =           lprf_release_char_device,
};

/**
 * Registers the LPRF-Chip as char-device. Make sure to execute this function only
 * after the chip has been fully initialized and is ready to use.
 */
static int register_char_device(struct lprf *lprf)
{
	int ret = 0;
	dev_t dev_number = 0;

	ret = alloc_chrdev_region(&dev_number, 0, 1, "lprf_rx");
	if (ret)
	{
		PRINT_DEBUG("Dynamic Device number allocation failed");
		return ret;
	}

	cdev_init(&lprf->my_char_dev, &lprf_fops);
	lprf->my_char_dev.owner = THIS_MODULE;
	ret = cdev_add (&lprf->my_char_dev, dev_number, 1);
	if (ret)
	{
		goto unregister;
	}
	PRINT_DEBUG("Successfully added char driver to system");
	return ret;

unregister:
	unregister_chrdev_region(dev_number, 1);
	return ret;
}

static inline void unregister_char_device(struct lprf *lprf)
{
	dev_t dev_number = lprf->my_char_dev.dev;
	cdev_del(&lprf->my_char_dev);
	unregister_chrdev_region(dev_number, 1);
	PRINT_DEBUG( "Removed Char Device");
}

static int init_lprf_hardware(struct lprf *lprf)
{
	// todo evaluate return values
	int rx_counter_length = get_rx_length_counter_H(KBIT_RATE, FRAME_LENGTH);

	lprf_write_register(lprf, RG_GLOBAL_RESETB, 0xFF);
	lprf_write_register(lprf, RG_GLOBAL_RESETB, 0x00);
	lprf_write_register(lprf, RG_GLOBAL_RESETB, 0xFF);
	lprf_write_register(lprf, RG_GLOBAL_initALL, 0xFF);
	lprf_write_subreg(lprf, SR_CTRL_CLK_CDE_OSC, 0);
	lprf_write_subreg(lprf, SR_CTRL_CLK_CDE_PAD, 1);
	lprf_write_subreg(lprf, SR_CTRL_CLK_DIG_OSC, 0);
	lprf_write_subreg(lprf, SR_CTRL_CLK_DIG_PAD, 1);
	lprf_write_subreg(lprf, SR_CTRL_CLK_PLL_OSC, 0);
	lprf_write_subreg(lprf, SR_CTRL_CLK_PLL_PAD, 1);
	lprf_write_subreg(lprf, SR_CTRL_CLK_C3X_OSC, 0);
	lprf_write_subreg(lprf, SR_CTRL_CLK_C3X_PAD, 1);
	lprf_write_subreg(lprf, SR_CTRL_CLK_FALLB, 0);

	// activate 2.4GHz Band
	lprf_write_subreg(lprf, SR_RX_RF_MODE, 0);
	lprf_write_subreg(lprf, SR_RX_LO_EXT, 1);


	//lprf_write_subreg(lprf, SR_LNA24_CTRIM, 255);
	lprf_write_subreg(lprf, SR_PPF_TRIM, 5);

	lprf_write_subreg(lprf, SR_PPF_HGAIN, 1);
	lprf_write_subreg(lprf, SR_PPF_LLIF, 0);
	lprf_write_subreg(lprf, SR_LNA24_ISETT, 7);
	lprf_write_subreg(lprf, SR_LNA24_SPCTRIM, 15);

	// ADC_CLK
	lprf_write_subreg(lprf, SR_CTRL_CDE_ENABLE, 0);
	lprf_write_subreg(lprf, SR_CTRL_C3X_ENABLE, 1);
	lprf_write_subreg(lprf, SR_CTRL_CLK_ADC, 1);
	lprf_write_subreg(lprf, SR_CTRL_C3X_LTUNE, 1);


	lprf_write_subreg(lprf, SR_CTRL_ADC_MULTIBIT, 0);
	//lprf_write_subreg(lprf, SR_ADC_D_EN, 1);
	lprf_write_subreg(lprf, SR_CTRL_ADC_ENABLE, 1);

	lprf_write_subreg(lprf, SR_LDO_A_VOUT, 0x11);
	lprf_write_subreg(lprf, SR_LDO_D_VOUT, 0x12);




	// initial gain settings
	lprf_write_subreg(lprf, SR_DEM_GC1, 0);
	lprf_write_subreg(lprf, SR_DEM_GC2, 0);
	lprf_write_subreg(lprf, SR_DEM_GC3, 1);
	lprf_write_subreg(lprf, SR_DEM_GC4, 0);
	lprf_write_subreg(lprf, SR_DEM_GC5, 0);
	lprf_write_subreg(lprf, SR_DEM_GC6, 1);
	lprf_write_subreg(lprf, SR_DEM_GC7, 4);


	lprf_write_subreg(lprf, SR_DEM_CLK96_SEL, 1);
	lprf_write_subreg(lprf, SR_DEM_AGC_EN, 1);
	lprf_write_subreg(lprf, SR_DEM_FREQ_OFFSET_CAL_EN, 0);
	lprf_write_subreg(lprf, SR_DEM_OSR_SEL, 0);
	lprf_write_subreg(lprf, SR_DEM_BTLE_MODE, 1);

	lprf_write_subreg(lprf, SR_DEM_IF_SEL, 2);
	lprf_write_subreg(lprf, SR_DEM_DATA_RATE_SEL, 3);

	lprf_write_subreg(lprf, SR_PPF_M0, 0);
	lprf_write_subreg(lprf, SR_PPF_M1, 0);
	lprf_write_subreg(lprf, SR_PPF_TRIM, 0);
	lprf_write_subreg(lprf, SR_PPF_HGAIN, 1);
	lprf_write_subreg(lprf, SR_PPF_LLIF, 0);

	lprf_write_subreg(lprf, SR_CTRL_ADC_BW_SEL, 1);
	lprf_write_subreg(lprf, SR_CTRL_ADC_BW_TUNE, 4);
	lprf_write_subreg(lprf, SR_CTRL_ADC_DR_SEL, 2);
	//lprf_write_subreg(lprf, SR_CTRL_ADC_DWA, 1);


	lprf_write_subreg(lprf, SR_DEM_IQ_CROSS, 1);
	lprf_write_subreg(lprf, SR_DEM_IQ_INV, 0);

	//lprf_write_subreg(lprf, SR_CTRL_C3X_LTUNE, 0);

	// STATE MASCHINE CONFIGURATION

	lprf_write_subreg(lprf, SR_FIFO_MODE_EN, 1);

	// SM TX
	lprf_write_subreg(lprf, SR_TX_MODE, 0);
	lprf_write_subreg(lprf, SR_INVERT_FIFO_CLK, 0);
	lprf_write_subreg(lprf, SR_DIRECT_RX, 0);
	lprf_write_subreg(lprf, SR_TX_ON_FIFO_IDLE, 0);
	lprf_write_subreg(lprf, SR_TX_ON_FIFO_SLEEP, 0);
	lprf_write_subreg(lprf, SR_TX_IDLE_MODE_EN, 0);

	// SM RX
	lprf_write_subreg(lprf, SR_DIRECT_TX, 0);
	lprf_write_subreg(lprf, SR_DIRECT_TX_IDLE, 0);
	lprf_write_subreg(lprf, SR_RX_HOLD_MODE_EN, 0);
	lprf_write_subreg(lprf, SR_RX_TIMEOUT_EN, 1);
	lprf_write_subreg(lprf, SR_RX_HOLD_ON_TIMEOUT, 0);
	lprf_write_subreg(lprf, SR_AGC_AUTO_GAIN, 0);

	// lprf_write_subreg(lprf, SR_RSSI_THRESHOLD, 2);  //--> default value

	lprf_write_subreg(lprf, SR_RX_LENGTH_H, COUNTER_H_BYTE(rx_counter_length));
	lprf_write_subreg(lprf, SR_RX_LENGTH_M, COUNTER_M_BYTE(rx_counter_length));
	lprf_write_subreg(lprf, SR_RX_LENGTH_L, COUNTER_L_BYTE(rx_counter_length));

	lprf_write_subreg(lprf, SR_RX_TIMEOUT_H, 0xFF);
	lprf_write_subreg(lprf, SR_RX_TIMEOUT_M, 0xFF);
	lprf_write_subreg(lprf, SR_RX_TIMEOUT_L, 0xFF);

	lprf_write_subreg(lprf, SR_WAKEUPONSPI, 1);
	lprf_write_subreg(lprf, SR_WAKEUPONRX, 0);
	lprf_write_subreg(lprf, SR_WAKEUP_MODES_EN, 0);

	// -> PLL Configuration

	lprf_write_subreg(lprf, SR_FIFO_RESETB, 0);
	lprf_write_subreg(lprf, SR_FIFO_RESETB, 1);

	lprf_write_subreg(lprf, SR_SM_EN, 1);
	lprf_write_subreg(lprf, SR_SM_RESETB, 0);
	lprf_write_subreg(lprf, SR_SM_RESETB, 1);

	return 0;
}

static int lprf_probe(struct spi_device *spi)
{
	u32 custom_value = 0;
	int ret = 0;
	struct lprf_platform_data *pdata = 0;
	struct lprf *lprf = 0;
	struct ieee802154_hw *ieee802154_hw = 0;
	PRINT_DEBUG( "call lprf_probe");

	pdata = spi->dev.platform_data;

	// Get platform data
	if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) {
		if (!pdata)
			return -ENOENT;
	}

	PRINT_DEBUG( "successfully parsed platform data");


	ret = of_property_read_u32(spi->dev.of_node, "some-custom-value", &custom_value);
	PRINT_DEBUG( "returned value:\t%d, custom value:\t%d", ret, custom_value);

	ieee802154_hw = ieee802154_alloc_hw(sizeof(*lprf), &ieee802154_lprf_callbacks);
	if( ieee802154_hw == 0)
		return -ENOMEM;
	PRINT_DEBUG("Successfully allocated ieee802154_hw structure");

	lprf = ieee802154_hw->priv;
	lprf->ieee802154_hw = ieee802154_hw;

	lprf->spi_device = spi;
	spi_set_drvdata(spi, lprf);

	init_waitqueue_head(&lprf->wait_for_fifo_data);

	ieee802154_hw->flags = 0;
	ieee802154_hw->parent = &lprf->spi_device->dev;
	ieee802154_random_extended_addr(&ieee802154_hw->phy->perm_extended_addr);

	lprf->regmap = devm_regmap_init_spi(spi, &lprf_regmap_spi_config);
	if (IS_ERR(lprf->regmap)) {
		PRINT_DEBUG( "Failed to allocate register map: %d", (int) PTR_ERR(lprf->regmap) );
	}

	PRINT_DEBUG( "successfully initialized Register map");

	ret = lprf_detect_device(lprf);
	if(ret)
		goto free_lprf;

	ret = init_lprf_hardware(lprf);
	if(ret)
		goto free_lprf;
	PRINT_DEBUG("Hardware successfully initialized");

	ret = allocate_spi_buffer(lprf);
	if (ret)
		goto free_lprf;
	PRINT_DEBUG("Spi Buffer successfully allocated and data receive started");

	ret = register_char_device(lprf);
	if(ret)
		goto free_spi_buffer;

	ret = ieee802154_register_hw(ieee802154_hw);
	if (ret)
		goto unregister_char_device;
	PRINT_DEBUG("Successfully registered IEEE 802.15.4 device");

	return ret;

unregister_char_device:
	unregister_char_device(lprf);
free_spi_buffer:
	kfifo_free(&lprf->spi_buffer);
free_lprf:
	ieee802154_free_hw(ieee802154_hw);

	return ret;
}

static int lprf_remove(struct spi_device *spi)
{
	struct lprf *lprf = spi_get_drvdata(spi);

	kfifo_free(&lprf->spi_buffer);
	unregister_char_device(lprf);

	ieee802154_unregister_hw(lprf->ieee802154_hw);
	ieee802154_free_hw(lprf->ieee802154_hw);
	PRINT_DEBUG("Successfully removed LPRF SPI Device");

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



