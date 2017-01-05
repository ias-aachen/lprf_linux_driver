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
#include <linux/hrtimer.h>
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

struct lprf_local;

static int init_lprf_hardware(struct lprf_local *lprf);
static inline void lprf_start_rx_polling_timer(struct lprf_local *lprf,
		ktime_t first_interval);
static inline void lprf_stop_rx_polling(struct lprf_local *lprf);
static int lprf_start_frame_write(struct lprf_local *lprf);
static void read_lprf_fifo(struct lprf_local *lprf);

static int lprf_start_ieee802154(struct ieee802154_hw *hw);
static void lprf_stop_ieee802154(struct ieee802154_hw *hw);
static int lprf_set_ieee802154_channel(struct ieee802154_hw *hw, u8 page, u8 channel);
static int lprf_set_ieee802154_addr_filter(struct ieee802154_hw *hw,
		    struct ieee802154_hw_addr_filt *filt,
		    unsigned long changed);
static int lprf_xmit_ieee802154_async(struct ieee802154_hw *hw, struct sk_buff *skb);
static int lprf_ieee802154_energy_detection(struct ieee802154_hw *hw, u8 *level);

static const uint8_t PHY_HEADER[] = {0x55, 0x55, 0x55, 0x55, 0xe5};

struct lprf_platform_data {
	int some_custom_value;
};


/*
 * lprf_phy_status is used for the async spi tranfer to get status
 * information of the chip in lprf_phy_status_async().
 *
 * @spi_device: spi device to use
 * @spi_message: spi_message for the spi transfer
 * @buf: buffer for the spi transfer
 */
struct lprf_phy_status {
        struct spi_device *spi_device;
        struct spi_message spi_message;
        struct spi_transfer spi_transfer;
        uint8_t rx_buf[1];
        uint8_t tx_buf[1];
        bool is_active;
        void (*complete)(void *context);
};

/*
 * @ in_progress: is true if a state change or rx_poll is currently
 * 	in progress.
 */
struct lprf_state_change {
	struct lprf_local *lprf;

	struct hrtimer timer;
        struct spi_message spi_message;
        struct spi_transfer spi_transfer;
        uint8_t rx_buf[MAX_SPI_BUFFER_SIZE];
        uint8_t tx_buf[MAX_SPI_BUFFER_SIZE];

        uint8_t to_state;
        bool transition_in_progress;

        uint8_t sm_main_value;
        uint8_t dem_main_value;
};

/**
 * @ spi_message: And spi_message struct that can be used for asynchronous
 * 	spi transfers. Make sure to lock spi_mutex and set the correct callback
 * 	when using spi_message.
 */
struct lprf_local {
	struct spi_device *spi_device;
	struct regmap *regmap;
	struct mutex spi_mutex;
	struct cdev my_char_dev;
	struct hrtimer rx_polling_timer;
	DECLARE_KFIFO_PTR(rx_buffer, uint8_t);
	DECLARE_KFIFO_PTR(tx_buffer, uint8_t);
	struct ieee802154_hw *ieee802154_hw;
	atomic_t rx_polling_active;

	struct lprf_phy_status phy_status;
	struct lprf_state_change state_change;
};

struct lprf_char_driver_interface {
	atomic_t is_open;
	DECLARE_KFIFO_PTR(data_buffer, uint8_t);
	wait_queue_head_t wait_for_fifo_data;

} lprf_char_driver_interface;

static bool lprf_reg_writeable(struct device *dev, unsigned int reg)
{
	if(((reg >= 0) && (reg < 53)) ||
			((reg >= 56) && (reg < 70)) ||
			((reg >= 80) && (reg < 176)) ||
			((reg >= 192) && (reg <= 243)))
		return true;
	else
		return false;
}

static bool lprf_is_read_only_reg(unsigned int reg)
{
	switch (reg) {
	case RG_PLL_TPM_GAIN_OUT_L:
	case RG_PLL_TPM_GAIN_OUT_M:
	case RG_PLL_TPM_GAIN_OUT_H:
	case RG_DEM_PD_OUT:
	case RG_DEM_GC_AOUT:
	case RG_DEM_GC_BOUT:
	case RG_DEM_GC_COUT:
	case RG_DEM_GC_DOUT:
	case RG_DEM_FREQ_OFFSET_OUT:
	case RG_SM_STATE:
	case RG_SM_FIFO:
	case RG_SM_GLOBAL:
	case RG_SM_POWER:
	case RG_SM_RX:
	case RG_SM_WAKEUP_EN:
	case RG_SM_DEM_ADC:
	case RG_SM_PLL_TX:
	case RG_SM_PLL_CHAN_INT:
	case RG_SM_PLL_CHAN_FRAC_H:
	case RG_SM_PLL_CHAN_FRAC_M:
	case RG_SM_PLL_CHAN_FRAC_L:
	case RG_SM_TX433:
	case RG_SM_TX800:
	case RG_SM_TX24:
		return true;
	default:
		return false;
	}
}

static bool lprf_reg_readable(struct device *dev, unsigned int reg)   //part of struct regmap_config lprf_regmap_spi_config
{

	return lprf_reg_writeable(dev, reg) ||
			lprf_is_read_only_reg(reg);
}

static bool lprf_reg_volatile(struct device *dev, unsigned int reg)      //part of struct regmap_config lprf_regmap_spi_config
{
	// All Read Only Registers are volatile
	if (lprf_is_read_only_reg(reg))
		return true;

	switch (reg) {
	case RG_GLOBAL_RESETB:
	case RG_GLOBAL_initALL:
	case RG_ACTIVATE_ALL:
		return true;
	default:
		return false;
	}
}

static bool lprf_reg_precious(struct device *dev, unsigned int reg)       //part of struct regmap_config lprf_regmap_spi_config
{
	// The LPRF-Chip has no precious register
	return false;
}

static const struct regmap_config lprf_regmap_spi_config = {
	.reg_bits = 16,
	.reg_stride = 1,
	.pad_bits = 0,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	.write_flag_mask = 0xc0,
	.fast_io = 0, // use mutex or spinlock for locking
	.max_register = 0xF3,
	.use_single_rw = 1, // single read write commands or bulk read write
	.can_multi_write = 0,
	.cache_type = REGCACHE_RBTREE,
	.writeable_reg = lprf_reg_writeable,
	.readable_reg = lprf_reg_readable,
	.volatile_reg = lprf_reg_volatile,
	.precious_reg = lprf_reg_precious,
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
static inline void reverse_bit_order_and_invert_bits(uint8_t *byte)
{
	*byte = ((*byte & 0xaa) >> 1) | ((*byte & 0x55) << 1);
	*byte = ((*byte & 0xcc) >> 2) | ((*byte & 0x33) << 2);
	*byte = ~((*byte >> 4) | (*byte << 4));
}

static inline uint8_t lprf_subreg(uint8_t reg_val, uint8_t addr, uint8_t mask,
		uint8_t shift, uint8_t data)
{
	return (reg_val & ~mask) | (data << shift);
}

/**
 * Every function calling lprf_read_register needs to hold the
 * lprf.spi_mutex.
 */
static inline int lprf_read_register(struct lprf_local *lprf, unsigned int address, unsigned int *value)
{
	int ret = 0;
	ret = regmap_read(lprf->regmap, address, value);
	// PRINT_KRIT( "Read value %X from LPRF register %X", *value, address);
	return ret;
}

/**
 * Every function calling lprf_write_register needs to hold the
 * lprf.spi_mutex.
 */
static inline int lprf_write_register(struct lprf_local *lprf, unsigned int address, unsigned int value)
{
	int ret = 0;
	ret = regmap_write(lprf->regmap, address, value);
	// PRINT_KRIT( "Write value %X to LPRF register %X", value, address);
	return ret;
}

/**
 * Every function calling lprf_read_subreg needs to hold the
 * lprf.spi_mutex.
 */
static inline int lprf_read_subreg(struct lprf_local *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int *data)
{
	int ret = 0;
	ret = lprf_read_register(lprf, addr, data);
	*data = (*data & mask) >> shift;
	return ret;
}

/**
 * Every function calling lprf_write_subreg needs to hold the
 * lprf.spi_mutex.
 */
static inline int lprf_write_subreg(struct lprf_local *lprf,
		unsigned int addr, unsigned int mask,
		unsigned int shift, unsigned int data)
{
	return regmap_update_bits(lprf->regmap, addr, mask, data << shift);
}


/**
 * Read the phy_status byte. The calling function needs to hold the
 * lprf.spi_mutex
 */
static inline int lprf_read_phy_status(struct lprf_local *lprf)
{
	uint8_t rx_buf[] = {0};
	int ret = 0;

	ret = spi_read(lprf->spi_device, rx_buf, sizeof(rx_buf));
	if (ret)
		return ret;

	return rx_buf[0];

}

static void lprf_phy_status_complete(void *context)
{
	struct lprf_local *lprf = context;
	lprf->phy_status.complete(context);
	lprf->phy_status.is_active = false;
}

static int lprf_phy_status_async(struct lprf_phy_status *phy_status,
		void (*complete)(void *context))
{
	int ret = 0;

	if (phy_status->is_active) {
		PRINT_KRIT("Warning: PHY_STATUS read is already in progress. Abort...");
		return -EBUSY;
	}
	phy_status->is_active = true;

	phy_status->complete = complete;
	phy_status->spi_message.complete = lprf_phy_status_complete;
	ret = spi_async(phy_status->spi_device, &phy_status->spi_message);
	if (ret)
		PRINT_DEBUG("Async_spi returned with error code %d", ret);
	// TODO handle spi async error
	return 0;
}

static void lprf_async_write_register(struct lprf_state_change *state_change,
		uint8_t address, uint8_t value,
		void (*complete)(void *context))
{
	int ret = 0;
	state_change->tx_buf[0] = REGW;
	state_change->tx_buf[1] = address;
	state_change->tx_buf[2] = value;
	state_change->spi_transfer.len = 3;
	state_change->spi_message.complete = complete;
	ret = spi_async(state_change->lprf->spi_device, &state_change->spi_message);
	if (ret)
		PRINT_DEBUG("Async_spi returned with error code %d", ret);
	// TODO handle spi async error
}

/*
 * Calculates a proper vco tune value, that is needed for the pll.
 * The vco tune value is dependent on the pll frequency and therefore
 * needs to be changed every time the pll frequency gets changed.
 *
 * Returns the vco tune value or zero for invalid channel_number
 */
static int calc_vco_tune(int channel_number)
{
    switch(channel_number)
    {
    case 11: return 237;
    case 12: return 235;
    case 13: return 234;
    case 14: return 232;
    case 15: return 231;
    case 16: return 223;
    case 17: return 222;
    case 18: return 220;
    case 19: return 213;
    case 20: return 212;
    case 21: return 210;
    case 22: return 209;
    case 23: return 207;
    case 24: return 206;
    case 25: return 206;
    case 26: return 204;
    default: return 0;
    }
}


/*
 * Calculates the channel center frequency from the channel number as
 * specified in the IEEE 802.15.4 standard. The channel page is assumed
 * to be zero. Returns the frequency in HZ or zero for invalid channel number.
 */
static inline uint32_t calculate_rf_center_freq(int channel_number)
{
	if (channel_number >= 11 && channel_number <= 26)
	{
		uint32_t f_rf_MHz = 2405 + 5 * (channel_number - 11);
		return f_rf_MHz * 1000000U;
	}

	// TODO 800MHz support

	return 0;
}


/*
 * Calculates the PLL values from the rf_frequency and the
 * if_frequency. The rf_frequency can be calculated with
 * calculate_rf_center_freq(). The if_frequency should usually be
 * 1000000 for RX case and zero for TX case.
 *
 * Returns zero on success or -EINVAL for invalid parameters.
 */
static int lprf_calculate_pll_values(uint32_t rf_frequency,
		uint32_t if_frequency,
		int *int_val, int *frac_val)
{
	uint32_t f_lo = 0;

	if (rf_frequency > 2000000000) // use 2.4 GHz frontend
	{
		f_lo = ( rf_frequency - if_frequency) / 3 * 2;
		*int_val = f_lo  / 16000000;

		/*
		 * The exact formula would actually be
		 * frac = (f_lo % 16MHz) * 65536 / 1MHz.
		 * However, this would result in an integer overflow
		 * and the linux system seems not to support 64bit
		 * modulo operation. (228/3479) is the smallest error
		 * possible without integer overflow.
		 */
		*frac_val = (f_lo % 16000000) * 228 / 3479;
		return 0;
	}

	// TODO 800MHz support

	return -EINVAL;
}

static void lprf_tx_change_complete(void *context)
{
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;

	state_change->transition_in_progress = false;
	lprf_start_rx_polling_timer(lprf, TX_RX_INTERVAL);

	PRINT_DEBUG("State change to TX completed successfully");
}

/**
 * Calling function must hold lprf.spi_mutex
 */
static void __lprf_frame_write_complete(void *context)
{
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;

	PRINT_DEBUG("Spi Frame Write completed");

	if (!kfifo_is_empty(&lprf->tx_buffer))
	{
		PRINT_DEBUG("Still data in TX buffer to be transmitted.");
		lprf_start_frame_write(lprf);
		return;
	}

	lprf_async_write_register(state_change, RG_SM_MAIN,
			lprf_subreg(state_change->sm_main_value,
			SR_SM_COMMAND, STATE_CMD_TX),
			lprf_tx_change_complete);
	PRINT_DEBUG("Change state to TX");
}

/**
 * Calling function must hold lprf.spi_mutex
 */
static int lprf_start_frame_write(struct lprf_local *lprf)
{
	int ret = 0;
	int total_length = 0;
	int bytes_copied = 0;
	uint8_t buffer_length = 0;
	struct lprf_state_change *state_change = &lprf->state_change;

	total_length = (kfifo_len(&lprf->tx_buffer));
	buffer_length = (uint8_t) (total_length < LPRF_MAX_BUF - 1 ?
			total_length : LPRF_MAX_BUF - 1);


	state_change->tx_buf[0] = FRMW;
	state_change->tx_buf[1] = buffer_length;
	bytes_copied = kfifo_out(&lprf->tx_buffer, state_change->tx_buf + 2,
			buffer_length);
	if (bytes_copied != buffer_length)
		PRINT_DEBUG("Warning: Only %d/%d bytes copied from Tx buffer",
				bytes_copied, buffer_length);

	state_change->spi_message.complete = __lprf_frame_write_complete;

	state_change->spi_transfer.len = buffer_length;

	ret = spi_async(lprf->spi_device, &state_change->spi_message);
	if (ret)
		PRINT_DEBUG("Async_spi returned with error code %d", ret);
	return 0;


}

void lprf_rx_change_complete(void *context)
{
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;

	state_change->transition_in_progress = false;

	lprf_start_rx_polling_timer(lprf, RX_RX_INTERVAL);
}

/*
 * Unfortunately the digital part of the LPRF-chip has some issues.
 * The internal statemaschine does not reset some internal parts
 * correctly after changing from RX-mode to another mode. Therefore
 * before every statechange some manual resets have to be done to
 * avoid data corruption.
 */
static void lprf_rx_resets(void *context)
{
	static int reset_counter = 0;
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;

	switch (reset_counter)
	{
	case 0:
		lprf_async_write_register(state_change, RG_SM_MAIN,
				0x05, lprf_rx_resets);
		reset_counter++;
		return;
	case 1:
		lprf_async_write_register(state_change, RG_SM_MAIN,
				0x0F, lprf_rx_resets);
		reset_counter++;
		return;
	case 2:
		lprf_async_write_register(state_change, RG_DEM_MAIN,
				lprf_subreg(state_change->dem_main_value, SR_DEM_RESETB, 0),
				lprf_rx_resets);
		reset_counter++;
		return;
	case 3:
		lprf_async_write_register(state_change, RG_DEM_MAIN,
				lprf_subreg(state_change->dem_main_value, SR_DEM_RESETB, 1),
				lprf_rx_resets);
		reset_counter++;
		return;
	case 4:
		if (state_change->to_state == STATE_CMD_TX) {
			lprf_start_frame_write(lprf);
			reset_counter = 0;
		}
		else {
			lprf_async_write_register(state_change, RG_SM_MAIN,
					lprf_subreg(state_change->sm_main_value,
					SR_SM_COMMAND, STATE_CMD_RX),
					lprf_rx_resets);
			reset_counter++;
		}
		return;
	case 5:
		lprf_async_write_register(state_change, RG_SM_MAIN,
			lprf_subreg(state_change->sm_main_value,
			SR_SM_COMMAND, STATE_CMD_NONE),
			lprf_rx_change_complete);
		reset_counter = 0;
		return;
	default:
		PRINT_DEBUG("Internal error in lprf_rx_resets");
	}
}

static void lprf_async_state_change(void *context)
{
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;
	uint8_t phy_status = lprf->phy_status.rx_buf[0];

	PRINT_KRIT("In phy_status in state_change 0x%X", phy_status);

	if (!PHY_FIFO_EMPTY(phy_status) ||
			PHY_SM_STATUS(phy_status) == PHY_SM_BUSY ||
			PHY_SM_STATUS(phy_status) == PHY_SM_SENDING ||
			state_change->transition_in_progress)
	{
		PRINT_KRIT("Warning: LPRF Chip Busy, abort state change...");
		return;
	}

	// If TX data available change to TX
	if (!kfifo_is_empty(&lprf->tx_buffer))
	{
		//lprf_stop_rx_polling(lprf);
		state_change->transition_in_progress = true;
		state_change->to_state = STATE_CMD_TX;
		lprf_async_write_register(state_change, RG_SM_MAIN,
				lprf_subreg(state_change->sm_main_value,
				SR_SM_COMMAND, STATE_CMD_SLEEP),
				lprf_rx_resets);

		PRINT_KRIT("Changed state to sleep, will change to TX");
		return;
	}

	// change to RX if not already RX
	if (!(PHY_SM_STATUS(phy_status) == PHY_SM_RECEIVING))
	{
		state_change->to_state = STATE_CMD_RX;
		state_change->transition_in_progress = true;
		lprf_rx_resets(lprf);
		PRINT_KRIT("Change state to RX");
		return;
	}
}


static enum hrtimer_restart lprf_delayed_state_change(struct hrtimer *timer)
{
	int rc = 0;
	struct lprf_state_change *state_change = container_of(
			timer, struct lprf_state_change, timer);
	struct lprf_local *lprf = state_change->lprf;

	rc = lprf_phy_status_async(&lprf->phy_status,
			lprf_async_state_change);
	if (rc)
		hrtimer_start(&state_change->timer, STATE_RETRY_INTERVAL,
				HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}


/**
 * Compares number_of_bits bits starting from the LSB and returns
 * the number of equal bits.
 */
static inline int number_of_equal_bits(uint32_t x1, uint32_t x2,
		int number_of_bits)
{
	int i = 0;
	int counter = 0;
	uint32_t combined = ~(x1 ^ x2);

	for (i = 0; i < number_of_bits; ++i)
	{
		counter += combined & 1;
		combined >>= 1;
	}
	return counter;
}

/**
 * Calculates the data shift from the start of frame delimiter
 * @data: rx_data received from chip. Bit order and polarity needs to be
 * 	already corrected
 * @data_length: number of bytes in data.
 * @uint8_t sfd: start of frame delimiter, i.e. 0xe5
 * @preamble_length: number of octets in the preamble. The maximum supported
 * 	length is 4 octets
 * @is_first_preamble_bit_one: For FSK-modulation based data transfers
 * 	there are basically two types of preambles 010101... and 101010...
 * 	This parameter selects the type currently used.
 *
 * Returns the shift value (6/7/8) or zero if sfd was not found.
 *
 * The lprf chip has a hardware preamble detection. However, the hardware
 * preamble detection is limited to 8 bit (or optionally 5 bit). In the
 * IEEE 802.15.4 standard the preambles typically have a length of 4 byte.
 * Additionally the data from the chip is sometimes misaligned by one bit.
 * Therefore the additional preamble bits have to be removed and the
 * misalignment has to be adjusted in software.
 */
static int find_SFD_and_shift_data(uint8_t *data, int *data_length,
		uint8_t sfd, int preamble_length)
{
	int i;
	int sfd_start_postion = preamble_length - 1;
	int data_start_position = sfd_start_postion + 1;
	int shift = 8;
	int no_shift, one_bit_shift, two_bit_shift;

	if (*data_length < sfd_start_postion + 2)
		return -EFAULT;

	no_shift = number_of_equal_bits(sfd, data[sfd_start_postion], 8);
	one_bit_shift = number_of_equal_bits(sfd,
				((data[sfd_start_postion] << 8) |
				data[sfd_start_postion+1]) >> 7, 8);
	two_bit_shift = number_of_equal_bits(sfd,
				((data[sfd_start_postion] << 8) |
				data[sfd_start_postion+1]) >> 6, 8);

	if(no_shift < 7 && one_bit_shift < 7 && two_bit_shift < 7)
	{
		PRINT_KRIT("SFD not found.");
		return 0;
	}

	if (one_bit_shift >= 7)
		shift -= 1;
	else if (two_bit_shift >= 7)
		shift -= 2;

	PRINT_KRIT("Data will be shifted by %d bits to the right", shift);

	for (i = 0; i < *data_length - sfd_start_postion - 1; ++i)
	{
		data[i] = ((data[i + data_start_position] << 8) |
				data[i + data_start_position + 1]) >> shift;
	}
	*data_length -= (sfd_start_postion + 1);

	return shift;
}

static int lprf_receive_ieee802154_data(struct lprf_local *lprf)
{
	int buffer_length = 0;
	int frame_length = 0;
	uint8_t *buffer = 0;
	struct sk_buff *skb;
	int ret = 0;
	int lqi = 0;

	buffer_length = kfifo_len(&lprf->rx_buffer);
	buffer = kzalloc(buffer_length * sizeof(*buffer), GFP_KERNEL); // TODO should not be called in asynchronous context, because kzalloc can sleep
	if (buffer == 0)
		return -ENOMEM;

	if(kfifo_out(&lprf->rx_buffer, buffer, buffer_length) != buffer_length) {
		ret = -EINVAL;
		goto free_data;
	}

	if (find_SFD_and_shift_data(buffer, &buffer_length, 0xe5, 4) == 0) {
		PRINT_KRIT("SFD not found, ignoring frame");
		ret = -EINVAL;
		goto free_data;
	}

	frame_length = buffer[0];

	if (!ieee802154_is_valid_psdu_len(frame_length)) {
		PRINT_KRIT("Frame with invalid length %d received", frame_length);
		ret = -EINVAL;
		goto free_data;
	}

	if (frame_length > buffer_length) {
		PRINT_KRIT("frame length greater than received data length");
		ret = -EINVAL;
		goto free_data;
	}
	PRINT_KRIT("Length of received frame is %d", frame_length);

	skb = dev_alloc_skb(frame_length);
	if (!skb) {
		ret = -ENOMEM;
		goto free_data;
	}

	memcpy(skb_put(skb, frame_length), buffer + 1, frame_length);
	ieee802154_rx_irqsafe(lprf->ieee802154_hw, skb, lqi);

free_data:
	kfree(buffer);
	return ret;
}

static void preprocess_received_data(uint8_t *data, int length)
{
	int i = 0;
	for (i = 0; i < length; ++i)
	{
		reverse_bit_order_and_invert_bits(&data[i]);
	}
}

static void write_data_to_char_driver(uint8_t *data, int length)
{
	if (!atomic_read(&lprf_char_driver_interface.is_open))
		return;

	kfifo_in(&lprf_char_driver_interface.data_buffer, data, length);
}

static void __lprf_read_frame_complete(void *context)
{
	uint8_t *data_buf = 0;
	uint8_t phy_status = 0;
	int rc;
	int length = 0;
	int bytes_copied = 0;
	static const uint8_t SM_mask = 0xe0;
	struct lprf_local *lprf = context;
	struct lprf_state_change *state_change = &lprf->state_change;

	PRINT_KRIT("Frame read via SPI completed");

	data_buf = state_change->rx_buf + 2; // first two bytes of rx_buf do not contain data

	phy_status = state_change->rx_buf[0];
	length = state_change->rx_buf[1];

	preprocess_received_data(data_buf, length);
	write_data_to_char_driver(data_buf, length);
	bytes_copied = kfifo_in(&lprf->rx_buffer, data_buf, length);

	wake_up(&lprf_char_driver_interface.wait_for_fifo_data);
	PRINT_KRIT("Copied %d bytes of %d received bytes to ring buffer", bytes_copied, length);

	// LPRF FIFO not empty or still receiving -> get more data
	if (length == FIFO_PACKET_SIZE-1 || (phy_status & SM_mask) == 0xe0)
	{
		// note that buffers get reused without deleting old data
		read_lprf_fifo(lprf);
		return;
	}

	lprf_receive_ieee802154_data(lprf);
	rc = lprf_phy_status_async(&lprf->phy_status, lprf_async_state_change);
	if (rc)
		hrtimer_start(&state_change->timer, STATE_RETRY_INTERVAL,
				HRTIMER_MODE_REL);

}


static void read_lprf_fifo(struct lprf_local *lprf)
{
	int ret = 0;
	struct lprf_state_change *state_change = &lprf->state_change;
	state_change->spi_message.complete = __lprf_read_frame_complete;
	state_change->spi_transfer.len = LPRF_MAX_BUF + 2;

	state_change->tx_buf[0] = FRMR;


	PRINT_KRIT("Will start async SPI read for frame read");

	ret = spi_async(lprf->spi_device, &state_change->spi_message);
	if (ret)
		PRINT_DEBUG("Async_spi returned with error code %d", ret);
	// TODO handle SPI error
}


static void lprf_poll_rx(void *context)
{
	struct lprf_local *lprf = context;
	uint8_t phy_status = lprf->phy_status.rx_buf[0];

	PRINT_KRIT("Poll RX... phy_status = 0x%.2X", phy_status);

	if (lprf->state_change.transition_in_progress)
		return;

	if (PHY_SM_STATUS(phy_status) == PHY_SM_RECEIVING &&
			!PHY_FIFO_EMPTY(phy_status))
	{
		lprf_start_rx_polling_timer(lprf, RX_MAX_RECEIVE_TIME);
		return;
	}

	if(PHY_SM_STATUS(phy_status) == PHY_SM_SLEEP) {

		if(!PHY_FIFO_EMPTY(phy_status)) {
			PRINT_KRIT("Data Received, Timer will not be restarted.");
			//lprf_stop_rx_polling(lprf);
			read_lprf_fifo(lprf);
			return;
		}
		else {
			PRINT_DEBUG("Warning: poll rx but sleep mode: abort...");
			return; // TODO warning or state change
		}
	}
	lprf_start_rx_polling_timer(lprf, RX_POLLING_INTERVAL);
}

static enum hrtimer_restart lprf_start_poll_rx(struct hrtimer *timer)
{
	int rc = 0;
	struct lprf_local *lprf = container_of(
			timer, struct lprf_local, rx_polling_timer);

	rc = lprf_phy_status_async(&lprf->phy_status, lprf_poll_rx);
	if (rc)
		lprf_start_rx_polling_timer(lprf, RX_POLLING_INTERVAL);
	//hrtimer_forward_now(timer, RX_POLLING_INTERVAL);
	return HRTIMER_NORESTART;
}


static inline void lprf_start_rx_polling_timer(struct lprf_local *lprf,
		ktime_t first_interval)
{
	if (atomic_read(&lprf->rx_polling_active))
		hrtimer_start(&lprf->rx_polling_timer, first_interval,
				HRTIMER_MODE_REL);
}

static inline void lprf_stop_rx_polling(struct lprf_local *lprf)
{
	atomic_set(&lprf->rx_polling_active, 0);
	hrtimer_cancel(&lprf->rx_polling_timer);
	PRINT_KRIT("RX Data Polling stopped.");
}

static int lprf_start_ieee802154(struct ieee802154_hw *hw)
{
	struct lprf_local *lprf = hw->priv;

	PRINT_DEBUG("Call lprf_start_ieee802154...");

	atomic_set(&lprf->rx_polling_active, 1);
	lprf_phy_status_async(&lprf->phy_status, lprf_async_state_change);

	return 0;
}

static void lprf_stop_ieee802154(struct ieee802154_hw *hw)
{
	struct lprf_local *lprf = hw->priv;
	lprf_stop_rx_polling(lprf);

	/* Wait some time to make sure all pending communication finished*/
	usleep_range(900, 1000);

	lprf_write_subreg(lprf, SR_SM_COMMAND, STATE_CMD_SLEEP);
	lprf_write_subreg(lprf, SR_SM_COMMAND, STATE_CMD_NONE);
	lprf_write_subreg(lprf, SR_DEM_RESETB,  0);
	lprf_write_subreg(lprf, SR_DEM_RESETB,  1);
	lprf_write_subreg(lprf, SR_FIFO_RESETB, 0);
	lprf_write_subreg(lprf, SR_FIFO_RESETB, 1);
	lprf_write_subreg(lprf, SR_SM_RESETB,   0);
	lprf_write_subreg(lprf, SR_SM_RESETB,   1);
}

static int lprf_set_ieee802154_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	int pll_int = 0;
	int pll_frac = 0;
	int rf_freq = 0;
	int ret = 0;
	int vco_tune = 0;
	struct lprf_local *lprf = hw->priv;

	if (page != 0)
	{
		PRINT_DEBUG("Invalid channel page %d.", page);
		return -EINVAL;
	}

	rf_freq = calculate_rf_center_freq(channel);
	PRINT_DEBUG("RF-freq = %u", rf_freq);

	mutex_lock(&lprf->spi_mutex);

	if (ret)
		goto unlock_mutex;


	// for RX
	ret = lprf_calculate_pll_values(rf_freq, 1000000, &pll_int, &pll_frac);

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_CHAN_INT, pll_int) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_RX_CHAN_FRAC_H, BIT24_H_BYTE(pll_frac)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_RX_CHAN_FRAC_M, BIT24_M_BYTE(pll_frac)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_RX_CHAN_FRAC_L, BIT24_L_BYTE(pll_frac)) );
	PRINT_DEBUG("Set RX PLL values to int=%d and frac=0x%.6x",
			pll_int, pll_frac);

	// for TX
	ret = lprf_calculate_pll_values(rf_freq, 0, &pll_int, &pll_frac);

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_CHAN_INT, pll_int) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_TX_CHAN_FRAC_H, BIT24_H_BYTE(pll_frac)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_TX_CHAN_FRAC_M, BIT24_M_BYTE(pll_frac)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf,
			SR_TX_CHAN_FRAC_L, BIT24_L_BYTE(pll_frac)) );
	PRINT_DEBUG("Set TX PLL values to int=%d and frac=0x%.6x",
			pll_int, pll_frac);

	vco_tune = calc_vco_tune(channel);
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PLL_VCO_TUNE, vco_tune) );
	PRINT_DEBUG("Set VCO TUNE to %d", vco_tune);

unlock_mutex:
	mutex_unlock(&lprf->spi_mutex);
	return ret;
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
	int bytes_copied = 0;
	int rc = 0;
	struct lprf_local *lprf = hw->priv;
	if (!kfifo_is_empty(&lprf->tx_buffer))
	{
		PRINT_DEBUG("ERROR in xmit, buffer not empty yet");
	}

	bytes_copied = kfifo_in(&lprf->tx_buffer, PHY_HEADER, sizeof(PHY_HEADER));
	if (bytes_copied != sizeof(PHY_HEADER))
		return -EFBIG;

	bytes_copied = kfifo_in(&lprf->tx_buffer, skb->data, skb->len);
	if (bytes_copied != skb->len)
		return -EFBIG;

	rc = lprf_phy_status_async(&lprf->phy_status, lprf_async_state_change);
	if (rc)
		hrtimer_start(&lprf->state_change.timer, STATE_RETRY_INTERVAL,
						HRTIMER_MODE_REL);

	PRINT_KRIT("Wrote %d bytes to TX buffer", bytes_copied);
	return 0;
}

static int lprf_ieee802154_energy_detection(struct ieee802154_hw *hw, u8 *level)
{
	PRINT_DEBUG("Called unimplemented function lprf_ieee802154_energy_detection()");
	return 0;
}


int lprf_open_char_device(struct inode *inode, struct file *filp)
{
	struct lprf_local *lprf = 0;
	int ret = 0;
	lprf = container_of(inode->i_cdev, struct lprf_local, my_char_dev);    //http://stackoverflow.com/questions/15832301/understanding-container-of-macro-in-linux-kerne0;
	filp->private_data = lprf;

	if (atomic_inc_return(&lprf_char_driver_interface.is_open) != 1)
	{
		atomic_dec(&lprf_char_driver_interface.is_open);
		return -EMFILE;
	}

	ret = kfifo_alloc(&lprf_char_driver_interface.data_buffer,
			2024, GFP_KERNEL);
	if (ret)
		return ret;

	PRINT_DEBUG("LPRF successfully opened as char device");
	return 0;


}


int lprf_release_char_device(struct inode *inode, struct file *filp)
{
	struct lprf_local *lprf;
	lprf = container_of(inode->i_cdev, struct lprf_local, my_char_dev);

	kfifo_free(&lprf_char_driver_interface.data_buffer);
	atomic_dec(&lprf_char_driver_interface.is_open);

	PRINT_DEBUG("LPRF char device successfully released");
	return 0;
}

ssize_t lprf_read_char_device(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int buffer_length = 0;
	int bytes_to_copy = 0;
	int bytes_copied = 0;
	int ret = 0;

	PRINT_KRIT("Read from user space with buffer size %d requested", count);

	if( kfifo_is_empty(&lprf_char_driver_interface.data_buffer) )
	{
		PRINT_KRIT("Read_char_device goes to sleep because of empty buffer.");
		ret = wait_event_interruptible(
				lprf_char_driver_interface.wait_for_fifo_data,
			!kfifo_is_empty(&lprf_char_driver_interface.data_buffer));
		if (ret < 0)
			return ret;
		PRINT_KRIT("Returned from sleep in read_char_device.");
	}

	buffer_length = kfifo_len(&lprf_char_driver_interface.data_buffer);
	bytes_to_copy = (count < buffer_length) ? count : buffer_length;

	ret = kfifo_to_user(&lprf_char_driver_interface.data_buffer,
			buf, bytes_to_copy, &bytes_copied);
	if(ret)
		return ret;

	PRINT_KRIT("%d/%d bytes copied to user.",
			bytes_copied, buffer_length);

	return bytes_copied;
}


ssize_t lprf_write_char_device(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int free_buffer_size = 0;
	int bytes_to_copy = 0;
	int bytes_copied = 0;
	int ret = 0;
	struct lprf_local *lprf = filp->private_data;

	free_buffer_size = kfifo_avail(&lprf->tx_buffer);
	bytes_to_copy = free_buffer_size < count ? free_buffer_size : count;

	PRINT_DEBUG("Char driver write: TX buffer has %d bytes available",
			free_buffer_size);

	ret = kfifo_from_user(&lprf->tx_buffer, buf, bytes_to_copy,
			&bytes_copied);
	if (ret)
		return ret;
	PRINT_DEBUG("Copied %d/%d files to TX buffer", bytes_copied, count);

	ret = lprf_phy_status_async(&lprf->phy_status, lprf_async_state_change);
	if (ret)
		hrtimer_start(&lprf->state_change.timer, STATE_RETRY_INTERVAL,
						HRTIMER_MODE_REL);

	return bytes_copied;
}


static int allocate_spi_buffer(struct lprf_local *lprf)
{
	int ret = 0;

	ret = kfifo_alloc(&lprf->rx_buffer, 2024, GFP_KERNEL);
	if (ret)
		return ret;

	ret = kfifo_alloc(&lprf->tx_buffer, 2024, GFP_KERNEL);
	if (ret)
	{
		kfifo_free(&lprf->rx_buffer);
		return ret;
	}
	return 0;
}


/**
 * Detect LPRF Chip
 *
 * @lprf: lprf struct containing hardware information of lprf chip
 *
 * returns 0 on success, a negative error number on error.
 */
static int lprf_detect_device(struct lprf_local *lprf)
{
	int rx_buf = 0, ret=0, chip_id = 0;

	mutex_lock(&lprf->spi_mutex);

	HANDLE_SPI_ERROR( lprf_read_register(lprf, RG_CHIP_ID_H, &rx_buf) );
	chip_id |= (rx_buf << 8);

	HANDLE_SPI_ERROR( lprf_read_register(lprf, RG_CHIP_ID_L, &rx_buf) );
	chip_id |= rx_buf;

	mutex_unlock(&lprf->spi_mutex);

	if (chip_id != 0x1a51)
	{
		PRINT_DEBUG("Chip with invalid Chip ID %X found", chip_id);
		return -ENODEV;
	}


	lprf->ieee802154_hw->flags = 0;

	lprf->ieee802154_hw->phy->flags = 0; // TODO WPAN_PHY_FLAG_TXPOWER;

	lprf->ieee802154_hw->phy->supported.cca_modes = 0;
	lprf->ieee802154_hw->phy->supported.cca_opts = 0;
	lprf->ieee802154_hw->phy->supported.cca_ed_levels = 0;
	lprf->ieee802154_hw->phy->supported.cca_ed_levels_size = 0;
	lprf->ieee802154_hw->phy->cca.mode = NL802154_CCA_ENERGY;

	lprf->ieee802154_hw->phy->supported.channels[0] = 0x7FFF800;
	lprf->ieee802154_hw->phy->current_channel = 11;
	lprf->ieee802154_hw->phy->symbol_duration = 16;
	lprf->ieee802154_hw->phy->supported.tx_powers = 0;
	lprf->ieee802154_hw->phy->supported.tx_powers_size = 0;

	lprf->ieee802154_hw->phy->cca_ed_level = 42;
	lprf->ieee802154_hw->phy->transmit_power = 42;

	PRINT_DEBUG("LPRF Chip found with Chip ID %X", chip_id);
	return 0;

}

/**
 * Defines the callback functions for file operations when the lprf device is used
 * with the char driver interface
 */
static const struct file_operations lprf_fops = {
	.owner =             THIS_MODULE,
	.read =              lprf_read_char_device,
	.write =             lprf_write_char_device,
	.unlocked_ioctl =    0, // can be implemented for additional control
	.open =              lprf_open_char_device,
	.release =           lprf_release_char_device,
};

/**
 * Registers the LPRF-Chip as char-device. Make sure to execute this function only
 * after the chip has been fully initialized and is ready to use.
 */
static int register_char_device(struct lprf_local *lprf)
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

static inline void unregister_char_device(struct lprf_local *lprf)
{
	dev_t dev_number = lprf->my_char_dev.dev;
	cdev_del(&lprf->my_char_dev);
	unregister_chrdev_region(dev_number, 1);
	PRINT_DEBUG( "Removed Char Device");
}

static int init_lprf_hardware(struct lprf_local *lprf)
{
	int ret = 0;
	unsigned int value = 0;
	int rx_counter_length = get_rx_length_counter_H(KBIT_RATE, FRAME_LENGTH);

	mutex_lock(&lprf->spi_mutex);
	HANDLE_SPI_ERROR( lprf_write_register(lprf, RG_GLOBAL_RESETB, 0xFF) );
	HANDLE_SPI_ERROR( lprf_write_register(lprf, RG_GLOBAL_RESETB, 0x00) );
	HANDLE_SPI_ERROR( lprf_write_register(lprf, RG_GLOBAL_RESETB, 0xFF) );
	HANDLE_SPI_ERROR( lprf_write_register(lprf, RG_GLOBAL_initALL, 0xFF) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_CDE_OSC, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_CDE_PAD, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_DIG_OSC, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_DIG_PAD, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_PLL_OSC, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_PLL_PAD, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_C3X_OSC, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_C3X_PAD, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_FALLB, 0) );

	// -> PLL Configuration
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LDO_PLL_VOUT,   31) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LDO_VCO_VOUT,   31) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_IREF_PLL_CTRLB, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PLL_VCO_TUNE,   235) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PLL_LPF_C,      0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PLL_LPF_R,      9  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_CHAN_INT,    100) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_CHAN_FRAC_H, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_CHAN_FRAC_M, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_CHAN_FRAC_L, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_CHAN_INT,    100) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_CHAN_FRAC_H, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_CHAN_FRAC_M, 0  ) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_CHAN_FRAC_L, 0  ) );

	// activate 2.4GHz Band
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_RF_MODE, 0) );
	// TODO set to zero, when receiving with pll actually works
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_LO_EXT, 1) );


	//RETURN_ON_ERROR( lprf_write_subreg(lprf, SR_LNA24_CTRIM, 255) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_TRIM, 5) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_HGAIN, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_LLIF, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LNA24_ISETT, 7) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LNA24_SPCTRIM, 15) );

	// ADC_CLK
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CDE_ENABLE, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_C3X_ENABLE, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_CLK_ADC, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_C3X_LTUNE, 1) );


	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_MULTIBIT, 0) );
	//RETURN_ON_ERROR( lprf_write_subreg(lprf, SR_ADC_D_EN, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_ENABLE, 1) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LDO_A_VOUT, 0x11) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_LDO_D_VOUT, 0x12) );




	// initial gain settings
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC1, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC2, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC3, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC4, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC5, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC6, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_GC7, 4) );


	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_CLK96_SEL, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_AGC_EN, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_FREQ_OFFSET_CAL_EN, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_OSR_SEL, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_BTLE_MODE, 1) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_IF_SEL, 2) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_DATA_RATE_SEL, 3) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_M0, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_M1, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_TRIM, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_HGAIN, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_PPF_LLIF, 0) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_BW_SEL, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_BW_TUNE, 4) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_DR_SEL, 2) );
	//RETURN_ON_ERROR( lprf_write_subreg(lprf, SR_CTRL_ADC_DWA, 1) );


	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_IQ_CROSS, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DEM_IQ_INV, 0) );

	//RETURN_ON_ERROR( lprf_write_subreg(lprf, SR_CTRL_C3X_LTUNE, 0) );

	// STATE MASCHINE CONFIGURATION

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_FIFO_MODE_EN, 1) );

	// SM TX
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_MODE, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_INVERT_FIFO_CLK, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DIRECT_RX, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_ON_FIFO_IDLE, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_ON_FIFO_SLEEP, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_TX_IDLE_MODE_EN, 0) );

	// SM RX
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DIRECT_TX, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_DIRECT_TX_IDLE, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_HOLD_MODE_EN, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_TIMEOUT_EN, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_HOLD_ON_TIMEOUT, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_AGC_AUTO_GAIN, 0) );

	// lprf_write_subreg(lprf, SR_RSSI_THRESHOLD, 2);  //--> default value

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_LENGTH_H,
			BIT24_H_BYTE(rx_counter_length)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_LENGTH_M,
			BIT24_M_BYTE(rx_counter_length)) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_LENGTH_L,
			BIT24_L_BYTE(rx_counter_length)) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_TIMEOUT_H, 0xFF) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_TIMEOUT_M, 0xFF) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_RX_TIMEOUT_L, 0xFF) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_WAKEUPONSPI, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_WAKEUPONRX, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_WAKEUP_MODES_EN, 0) );


	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_FIFO_RESETB, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_FIFO_RESETB, 1) );

	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_SM_EN, 1) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_SM_RESETB, 0) );
	HANDLE_SPI_ERROR( lprf_write_subreg(lprf, SR_SM_RESETB, 1) );

	// The configuration values of SM_MAIN and DEM_MAIN are needed
	// for async spi transfers and are therefore saved here

	lprf_read_register(lprf, RG_SM_MAIN, &value);
	lprf->state_change.sm_main_value = value & 0x0f;
	lprf_read_register(lprf, RG_DEM_MAIN, &value);
	lprf->state_change.dem_main_value = value;

	mutex_unlock(&lprf->spi_mutex);

	lprf_set_ieee802154_channel(lprf->ieee802154_hw,
			lprf->ieee802154_hw->phy->current_page,
			lprf->ieee802154_hw->phy->current_channel);

	return 0;
}

static int lprf_probe(struct spi_device *spi)
{
	u32 custom_value = 0;
	int ret = 0;
	struct lprf_platform_data *pdata = 0;
	struct lprf_local *lprf = 0;
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

	hrtimer_init(&lprf->rx_polling_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	lprf->rx_polling_timer.function = lprf_start_poll_rx;

	mutex_init(&lprf->spi_mutex);
	lprf->spi_device = spi;
	spi_set_drvdata(spi, lprf);

	lprf->state_change.lprf = lprf;

	// init spi message

	lprf->phy_status.spi_device = spi;
	spi_message_init(&lprf->phy_status.spi_message);
	lprf->phy_status.spi_message.context = lprf;
	lprf->phy_status.spi_message.spi = spi;
	lprf->phy_status.spi_transfer.len = 1;
	lprf->phy_status.spi_transfer.tx_buf = lprf->phy_status.tx_buf;
	lprf->phy_status.spi_transfer.rx_buf = lprf->phy_status.rx_buf;
	spi_message_add_tail(&lprf->phy_status.spi_transfer,
			&lprf->phy_status.spi_message);

	spi_message_init(&lprf->state_change.spi_message);
	lprf->state_change.spi_message.context = lprf;
	lprf->state_change.spi_message.spi = spi;
	lprf->state_change.spi_transfer.len = 3;
	lprf->state_change.spi_transfer.tx_buf = lprf->state_change.tx_buf;
	lprf->state_change.spi_transfer.rx_buf = lprf->state_change.rx_buf;
	spi_message_add_tail(&lprf->state_change.spi_transfer,
			&lprf->state_change.spi_message);

	hrtimer_init(&lprf->state_change.timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	lprf->state_change.timer.function = lprf_delayed_state_change;

	init_waitqueue_head(&lprf_char_driver_interface.wait_for_fifo_data);

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
	kfifo_free(&lprf->rx_buffer);
	kfifo_free(&lprf->tx_buffer);
free_lprf:
	ieee802154_free_hw(ieee802154_hw);

	return ret;
}

static int lprf_remove(struct spi_device *spi)
{
	struct lprf_local *lprf = spi_get_drvdata(spi);

	// TODO call lprf_stop_ieee802154
	kfifo_free(&lprf->rx_buffer);
	kfifo_free(&lprf->tx_buffer);
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



