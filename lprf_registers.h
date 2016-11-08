/*
 * lprf_registers.h
 *
 * Copyright (C) 2016 IAS, RWTH Aachen University
 *  Moritz Schrey <mschrey@ias.rwth.-aachen.de>
 *
 * This file defines register names and addresses with
 *     #define RG_register-name    (hex_address)
 * as well as individual bit fields with
 *     #define SR_bit-field-name    register-address, bitmask, LSB of bit field
 *
 * 2016-07.04 - Version 1.1
 * 2016-09-15 - Version 1.2 - Removed Tabs, unnecessary white space and IQ_INV issue, added "t" at 0x24, removed "_IN" at 0x25 and 0x26, added "SR_" at 0x40

 *
  */

//Main
#define RG_GLOBAL_RESETB       (0x00)

#define RG_GLOBAL_initALL      (0x01)

#define RG_ACTIVATE_ALL        (0xFF)

#define RG_GLOBAL              (0x02)
#define SRf_WUR_DEBUG_EN        0x02, 0x10, 4
#define SR_BANDGAP_PD          0x02, 0x08, 3
#define SR_BANDGAP_EN          0x02, 0x04, 2
#define SR_BIAS_EXT            0x02, 0x02, 1
#define SR_IMEAS_ENB           0x02, 0x01, 0

#define RG_LDO                 (0x03)
#define SR_LDO_A               0x03, 0x20, 5
#define SR_LDO_PLL             0x03, 0x10, 4
#define SR_LDO_VCO             0x03, 0x08, 3
#define SR_LDO_TX433           0x03, 0x04, 2
#define SR_LDO_TX800           0x03, 0x02, 1
#define SR_LDO_TX24            0x03, 0x01, 0

#define RG_CHIP_ID_L           (0x04)
#define SR_CHIP_ID_L           0x04, 0xFF, 0

#define RG_CHIP_ID_H           (0x05)
#define SR_CHIP_ID_H           0x05, 0xFF, 0

#define RG_LDO_D_VOUT          (0x06)
#define SR_LDO_D_VOUT          0x06, 0x1F, 0

#define RG_LDO_OSCI_VOUT       (0x07)
#define SR_LDO_OSCI_VOUT       0x07, 0x1F, 0

#define RG_LDO_A_VOUT          (0x08)
#define SR_LDO_A_VOUT          0x08, 0x1F, 0

#define RG_LDO_PLL_VOUT        (0x09)
#define SR_LDO_PLL_VOUT        0x09, 0x1F, 0

#define RG_LDO_VCO_VOUT        (0x0A)
#define SR_LDO_VCO_VOUT        0x0A, 0x1F, 0

#define RG_LDO_TX433_VOUT      (0x0B)
#define SR_LDO_TX433_VOUT      0x0B, 0x1F, 0

#define RG_LDO_TX800_VOUT      (0x0C)
#define SR_LDO_TX800_VOUT      0x0C, 0x1F, 0

#define RG_LDO_TX24_VOUT       (0x0D)
#define SR_LDO_TX24_VOUT       0x0D, 0x1F, 0

#define RG_IREF_LDO_PLL        (0x10)
#define SR_IREF_LDO_CTRLB      0x10, 0x70, 4
#define SR_IREF_PLL_CTRLB      0x10, 0x07, 0

#define RG_IREF_CRY_C3X        (0x11)
#define SR_IREF_CRY_CTRLB      0x11, 0x70, 4
#define SR_IREF_C3X_CTRLB      0x11, 0x07, 0

#define RG_IREF_ADC_ADCMB      (0x12)
#define SR_IREF_ADC_CTRLB      0x12, 0x70, 4
#define SR_IREF_ADCMB_CTRLB    0x12, 0x07, 0

#define RG_IREF_FE             (0x13)
#define SR_IREF_FE_CTRLB       0x13, 0x07, 0

#define RG_IREF_WURX5_WURX10   (0x14)
#define SR_IREF_WURX5_CTRLB    0x14, 0x70, 4
#define SR_IREF_WURX10_CTRLB   0x14, 0x07, 0

#define RG_IREF_TX433_TX800    (0x15)
#define SR_IREF_TX433_CTRLB    0x15, 0x70, 4
#define SR_IREF_TX800_CTRLB    0x15, 0x07, 0

#define RG_IREF_TXMIX_TX24     (0x16)
#define SR_IREF_TXMIX_CTRLB    0x16, 0x70, 4
#define SR_IREF_TX24_CTRLB     0x16, 0x07, 0


//PLL
#define RG_PLL_MAIN            (0x20)
#define SR_PLL_BUFFER_EN       0x20, 0x80, 7
#define SR_PLL_VCO_CURRENTB    0x20, 0x60, 5
#define SR_PLL_EN              0x20, 0x10, 4
#define SR_PLL_RESETB          0x20, 0x08, 3
#define SR_PLL_REF96_SEL       0x20, 0x04, 2
#define SR_PLL_LPF_EN          0x20, 0x02, 1
#define SR_PLL_EXT_BIAS        0x20, 0x01, 0

#define RG_PLL_MOD             (0x21)
#define SR_PLL_MOD_EN          0x21, 0x10, 4
#define SR_PLL_OFFCHIP_MOD_EN  0x21, 0x08, 3
#define SR_PLL_TPM_EN          0x21, 0x04, 2
#define SR_PLL_MOD_DATA_RATE   0x21, 0x03, 0

#define RG_PLL_FREQ_DEV        (0x22)
#define SR_PLL_MOD_FREQ_DEV    0x22, 0x1F, 0

#define RG_PLL_CAL             (0x23)
#define SR_PLL_DAC_EN          0x23, 0x08, 3
#define SR_PLL_DAC_CP_DEFAULT  0x23, 0x04, 2
#define SR_PLL_LMS_EN          0x23, 0x02, 1
#define SR_PLL_LMS_HOLD_ON     0x23, 0x01, 0

#define RG_PLL_CHN_INT         (0x24)
#define SR_PLL_CHN_INT         0x24, 0x7F, 0

#define RG_PLL_CHN_FRAC_H      (0x25)
#define SR_PLL_CHN_FRAC_H      0x25, 0x0F, 0

#define RG_PLL_CHN_FRAC_M      (0x26)
#define SR_PLL_CHN_FRAC_M      0x26, 0xFF, 0

#define RG_PLL_CHN_FRAC_L      (0x27)
#define SR_PLL_CHN_FRAC_L      0x27, 0xFF, 0

#define RG_PLL_LPP_C           (0x28)
#define SR_PLL_LPF_C           0x28, 0x1F, 0

#define RG_PLL_LPP_R           (0x29)
#define SR_PLL_LPF_R           0x29, 0x0F, 0

#define RG_PLL_VCO_TUNE        (0x2A)
#define SR_PLL_VCO_TUNE        0x2A, 0xFF, 0

#define RG_PLL_TPM             (0x2B)
#define SR_PLL_TPM_LOW_RES          0x2B, 0x80, 7
#define SR_PLL_CTRL_TPM_LSB_EN      0x2B, 0x40, 6
#define SR_PLL_TPM_CTRL_GAIN_SHIFT  0x2B, 0x20, 5
#define SR_PLL_TPM_CTRL_CAL_EN      0x2B, 0x10, 4
#define SR_PLL_TPM_CTRL_COMP_EN     0x2B, 0x08, 3
#define SR_PLL_TPM_BW               0x2B, 0x06, 1
#define SR_PLL_TPM_COMP_EN          0x2B, 0x01, 0

#define RG_PLL_TPM_CTRL_DELAY (0x2C)
#define SR_PLL_TPM_CTRL_DELAY   0x2C, 0x1F, 0

#define RG_PLL_TPM_CTRL_GAIN_L (0x2D)
#define SR_PLL_TPM_CTRL_GAIN_L  0x2D, 0xFF, 0

#define RG_PLL_TPM_CTRL_GAIN_M (0x2E)
#define SR_PLL_TPM_CTRL_GAIN_M  0x2E, 0xFF, 0

#define RG_PLL_TPM_GAIN_FREQ_H (0x2F)
#define SR_PLL_TPM_CTRL_FREQ_H  0x2F, 0xF0, 4
#define SR_PLL_TPM_CTRL_GAIN_H  0x2F, 0x07, 0

#define RG_PLL_TPM_CTRL_FREQ_L (0x30)
#define SR_PLL_TPM_CTRL_FREQ_L  0x30, 0xFF, 0

#define RG_PLL_TPM_CTRL_FREQ_M (0x31)
#define SR_PLL_TPM_CTRL_FREQ_M  0x31, 0xFF, 0

#define RG_PLL_TPM_FREQ_OFFSET        (0x32)
#define SR_PLL_TPM_CTRL_FREQ_OFFSET   0x32, 0x1F, 0

#define RG_PLL_TPM_LOCK_CYCLES        (0x33)
#define SR_PLL_TPM_CTRL_LOCK_CYCLES   0x33, 0xFF, 0

#define RG_PLL_TPM_COMP_CYCLES        (0x34)
#define SR_PLL_TPM_CTRL_COMP_CYCLES   0x34, 0xFF, 0

#define RG_PLL_TPM_GAIN_OUT_L         (0x35)
#define SR_PLL_TPM_CTRL_GAIN_OUT_L    0x35, 0xFF, 0

#define RG_PLL_TPM_GAIN_OUT_M         (0x36)
#define SR_PLL_TPM_CTRL_GAIN_OUT_M    0x36, 0xFF, 0

#define RG_PLL_TPM_GAIN_OUT_H         (0x37)
#define SR_PLL_TPM_CTRL_GAIN_OUT_H    0x37, 0x07, 0

#define RG_PLL_TPM_CTRL_TUNE          (0x38)
#define SR_PLL_TPM_CTRL_OFFSET_H      0x38, 0x10, 4
#define SR_PLL_TPM_I_CTRL             0x38, 0x0E, 1
#define SR_PLL_TPM_CTRL_VTUNE_INV     0x38, 0x01, 0

#define RG_PLL_TPM_CTRL_OFFSET_L      (0x39)
#define SR_PLL_TPM_CTRL_OFFSET_L      0x39, 0xFF, 0

#define RG_PLL_TPM_CTRL_MEAS          (0x3A)
#define SR_PLL_TPM_CTRL_MEASURE_EN    0x3A, 0x80, 7
#define SR_PLL_TPM_CTRL_MEASURE_VAL_H 0x3A, 0x40, 6
#define PLL_TPM_CTRL_MEASURE_FREQ     0x3A, 0x3F, 0

#define RG_PLL_TPM_CTRL_MEASVA (0x3B)
#define SR_PLL_TPM_CTRL_MEASURE_VAL_L 0x3B, 0xFF, 0


//on-chip demodulator
#define RG_DEM_MAIN            (0x40)
#define SR_DEM_RESETB          0x40, 0x80, 7
#define SR_DEM_EN                 0x40, 0x40, 6
#define SR_DEM_CLK96_SEL          0x40, 0x20, 5
#define SR_DEM_PD_EN              0x40, 0x10, 4
#define SR_DEM_AGC_EN             0x40, 0x08, 3
#define SR_DEM_FREQ_OFFSET_CAL_EN 0x40, 0x04, 2
#define SR_DEM_OSR_SEL            0x40, 0x02, 1
#define SR_DEM_BTLE_MODE          0x40, 0x01, 0

#define RG_DEM_SET             (0x41)
#define SR_DEM_IQ_INV          0x41, 0x60, 5
#define SR_DEM_IQ_CROSS        0x41, 0x10, 4
#define SR_DEM_IF_SEL          0x41, 0x0C, 2
#define SR_DEM_DATA_RATE_SEL   0x41, 0x03, 0

#define RG_DEM_GC_1_2          (0x42)
#define SR_DEM_GC1             0x42, 0xF0, 4
#define SR_DEM_GC2             0x42, 0x0F, 0

#define RG_DEM_GC_3_4          (0x43)
#define SR_DEM_GC3             0x043, 0xF0, 4
#define SR_DEM_GC4             0x043, 0x0F, 0

#define RG_DEM_GC_5_6          (0x44)
#define SR_DEM_GC5             0x044, 0xF0, 4
#define SR_DEM_GC6             0x044, 0x0F, 0

#define RG_DEM_GC_7            (0x45)
#define SR_DEM_GC7             0x45, 0x0F, 0

#define RG_DEM_PD_OUT          (0x46)
#define SR_DEM_PD_OUT          0x46, 0x01, 0

#define RG_DEM_GC_AOUT         (0x47)
#define SR_DEM_GC1_OUT         0x47, 0xF0, 4
#define SR_DEM_GC2_OUT         0x47, 0x0F, 0

#define RG_DEM_GC_BOUT         (0x48)
#define SR_DEM_GC3_OUT         0x48, 0xF0, 4
#define SR_DEM_GC4_OUT         0x48, 0x0F, 0

#define RG_DEM_GC_COUT         (0x49)
#define SR_DEM_GC5_OUT         0x49, 0xF0, 4
#define SR_DEM_GC6_OUT         0x49, 0x0F, 0

#define RG_DEM_GC_DOUT         (0x4A)
#define SR_DEM_GC7_OUT         0x4A, 0x0F, 0

#define RG_DEM_FREQ_OFFSET_OUT (0x4B)
#define SR_DEM_FREQ_OFFSET_OUT 0x4B, 0xF0, 0

//receiver frontend
#define RG_RX_MAIN             (0x50)
#define SR_RX_FE_EN            0x50, 0x40, 6
#define SR_RX_RF_MODE          0x50, 0x30, 4
#define SR_RX_LO_EXT           0x50, 0x08, 3
#define SR_RX24_PON            0x50, 0x04, 2
#define SR_RX800_PON           0x50, 0x02, 1
#define SR_RX433_PON           0x50, 0x01, 0

#define RG_LNA24_CTRIM         (0x51)
#define SR_LNA24_CTRIM         0x51, 0x01F, 0

#define RG_LNA24_SPCTRIM       (0x52)
#define SR_LNA24_SPCTRIM       0x52, 0x0F, 0

#define RG_LNA24_ISETT         (0x53)
#define SR_LNA24_ISETT         0x53, 0x07, 0

#define RG_LNA800_SET          (0x54)
#define SR_LNA433_HGAIN        0x54, 0x02, 1
#define SR_LNA800_HGAIN        0x54, 0x01, 0

#define RG_PPF                 (0x55)
#define SR_PPF_M1              0x55, 0x40, 6
#define SR_PPF_M0              0x55, 0x20, 5
#define SR_PPF_TRIM            0x55, 0x1C, 2
#define SR_PPF_HGAIN           0x55, 0x02, 1
#define SR_PPF_LLIF            0x55, 0x01, 0

//ADC and clock
#define RG_CLK_MAIN            (0x60)
#define SR_CTRL_CLK_ADC        0x60, 0x80, 7
#define SR_CTRL_CD3_ENABLE     0x60, 0x40, 6
#define SR_CTRL_CDE_ENABLE     0x60, 0x20, 5
#define SR_CTRL_C3X_ENABLE     0x60, 0x10, 4
#define SR_CTRL_CLK_C3X_OSC    0x60, 0x08, 3
#define SR_CTRL_CLK_C3X_PAD    0x60, 0x04, 2
#define SR_CTRL_CLK_CDE_OSC    0x60, 0x02, 1
#define SR_CTRL_CLK_CDE_PAD    0x60, 0x01, 0

#define RG_CLK_FALLBACK        (0x61)
#define SR_CTRL_CLK_FALLB      0x61, 0x04, 2
#define SR_CTRL_CLK_DIG_OSC    0x61, 0x02, 1
#define SR_CTRL_CLK_DIG_PAD    0x61, 0x01, 0

#define RG_CLK_SET             (0x62)
#define SR_CTRL_CLK_IREF       0x62, 0x70, 4
#define SR_CTRL_CLK_PLL_OSC    0x62, 0x08, 3
#define SR_CTRL_CLK_PLL_PAD    0x62, 0x04, 2
#define SR_CTRL_C3X_LTUNE      0x62, 0x03, 0

#define RG_CLK_TUNE            (0x63)
#define SR_CTRL_C3X_ICP        0x63, 0xF0, 4
#define SR_CTRL_CDE_TUNE       0x63, 0x0F, 0

#define RG_CLK_IVCD            (0x64)
#define SR_CTRL_C3X_IVCD       0x64, 0x7F, 0

#define RG_ADC_MAIN            (0x65)
#define SR_CTRL_ADC_D_BYPASS   0x65, 0x20, 5
#define SR_ADC_CTRL_RESETB     0x65, 0x10, 4
#define SR_CTRL_ADC_MULTIBIT   0x65, 0x02, 1
#define SR_CTRL_ADC_ENABLE     0x65, 0x01, 0

#define RG_ADC_TUNE1           (0x66)
#define SR_CTRL_ADC_DWA        0x66, 0x80, 6
#define SR_CTRL_ADC_DR_SEL     0x66, 0x60, 5
#define SR_CTRL_ADC_BW_TUNE    0x66, 0x1C, 2
#define SR_CTRL_ADC_BW_SEL     0x66, 0x03, 0

#define RG_ADC_TUNE2           (0x67)
#define SR_ADC_D_EN            0x67, 0x80, 7
#define SR_CTRL_DSM_MB2SB      0x67, 0x40, 6
#define SR_CTRL_DSM_DOE        0x67, 0x20, 5
#define SR_CTRL_DSM_SDWA       0x67, 0x10, 4
#define SR_CTRL_ADC_IOP        0x67, 0x03, 0

#define RG_ADC_TUNE3           (0x68)
#define SR_CTRL_ADC_RES0       0x68, 0x0F, 0

#define RG_ADC_IDAC            (0x69)
#define SR_CTRL_ADC_IADC       0x69, 0x30, 4
#define SR_CTRL_ADC_IDAC       0x69, 0x07, 0

#define RG_ADC_RES_BOT_N       (0x6A)
#define SR_CTRL_ADC_RESBOT     0x6A, 0xF0, 4
#define SR_CTRL_ADC_RESN       0x6A, 0x0F, 0

#define RG_ADC_RES_TOP_P       (0x6B)
#define SR_CTRL_ADC_RESTOP     0x6B, 0xF0, 4
#define SR_CTRL_ADC_RESP       0x6B, 0x0F, 0


//TX path
#define RG_TX_MAIN             (0x70)
#define SR_TX_AMPLI_OUT_MAN_H  0x70, 0x40, 6
#define SR_TX_RESETB           0x70, 0x20, 5
#define SR_TX_EN               0x70, 0x10, 4
#define SR_TX_ON_CHIP_MOD      0x70, 0x08, 3
#define SR_TX_SPI_FIFO_OUT_EN  0x70, 0x04, 2
#define SR_TX_UPS              0x70, 0x03, 0

#define RG_TX_ON_CHIP_MOD_SP   (0x71)
#define SR_TX_ON_CHIP_MOD_SP   0x71, 0x0F, 0

#define RG_TX_PHASE_SEL        (0x72)
#define SR_TX_PHASE_SEL        0x72, 0x0F, 0

#define RG_TX_GAIN_1_2         (0x73)
#define SR_TX_GAIN_AMPLI_F1    0x73, 0xF0, 4
#define SR_TX_GAIN_AMPLI_F2    0x73, 0x0F, 0

#define RG_TX_GAIN_3_BOOST     (0x74)
#define SR_TX_GAIN_AMPLI_F3    0x74, 0xF0, 4
#define SR_TX_BOOST_AMPLI      0x74, 0x0E, 1

#define RG_TX_DELAY            (0x75)
#define SR_TX_DELAY_AMPLI      0x75, 0x38, 3
#define SR_TX_DELAY_FREQU      0x75, 0x07, 0

#define RG_TX_AMPLI_OUT_MAN_L  (0x76)
#define SR_TX_AMPLI_OUT_MAN_L  0x76, 0xFF, 0

//TX frontend
#define RG_TX433               (0x7A)
#define SR_TX433_FREQ_DIV2_EN  0x7A, 0x40, 6
#define SR_TX433_EN            0x7A, 0x20, 5
#define SR_TX433_MAXAMP        0x7A, 0x10, 4
#define SR_TX433_PWR_CTRL      0x7A, 0x0F, 0

#define RG_TX433_TUNE          (0x7B)
#define SR_TX433_I_SUB_CTRL    0x7B, 0xF0, 4
#define SR_TX433_CAS_BIAS_EN   0x7B, 0x08, 3
#define SR_TX433_CAS_BIAS_CTRL 0x7B, 0x07, 0

#define RG_TX800               (0x7C)
#define SR_TX800_FREQ_DIV2_EN  0x7C, 0x40, 6
#define SR_TX800_EN            0x7C, 0x20, 5
#define SR_TX800_MAXAMP        0x7C, 0x10, 4
#define SR_TX800_PWR_CTRL      0x7C, 0x0F, 0

#define RG_TX800_TUNE          (0x7D)
#define SR_TX800_I_SUB_CTRL    0x7D, 0xF0, 4
#define SR_TX800_CAS_BIAS_EN   0x7D, 0x08, 3
#define SR_TX800_CAS_BIAS_CTRL 0x7D, 0x07, 0

#define RG_TX24                (0x7E)
#define SR_TX24_MIXER_PON      0x7E, 0x40, 6
#define SR_TX24_EN             0x7E, 0x20, 5
#define SR_TX24_MAXAMP         0x7E, 0x10, 4
#define SR_TX24_PWR_CTRL       0x7E, 0x0F, 0

#define RG_TX24_TUNE           (0x7F)
#define SR_TX24_I_SUB_CTRL     0x7F, 0xF0, 4
#define SR_TX24_CAS_BIAS_EN    0x7F, 0x08, 3
#define SR_TX24_CAS_BIAS_CTRL  0x7F, 0x07, 0

//statemachine
#define RG_SM_MAIN             (0x80)
#define SR_SM_COMMAND          0x80, 0xF0, 4
#define SR_FIFO_RESETB         0x80, 0x08, 3
#define SR_SM_EN               0x80, 0x04, 2
#define SR_SM_RESETB           0x80, 0x02, 1
#define SR_FIFO_MODE_EN        0x80, 0x01, 0

#define RG_SM_TX_SET           (0x81)
#define SR_INVERT_FIFO_CLK     0x81, 0x80, 7
#define SR_TX_MODE             0x81, 0x30, 4
#define SR_DIRECT_RX           0x81, 0x08, 3
#define SR_TX_ON_FIFO_IDLE     0x81, 0x04, 2
#define SR_TX_ON_FIFO_SLEEP    0x81, 0x02, 1
#define SR_TX_IDLE_MODE_EN     0x81, 0x01, 0

#define RG_SM_RX_SET           (0x82)
#define SR_DIRECT_TX           0x82, 0x20, 5
#define SR_DIRECT_TX_IDLE      0x82, 0x10, 4
#define SR_RX_HOLD_MODE_EN     0x82, 0x08, 3
#define SR_RX_TIMEOUT_EN       0x82, 0x04, 2
#define SR_RX_HOLD_ON_TIMEOUT  0x82, 0x02, 1
#define SR_AGC_AUTO_GAIN       0x82, 0x01, 0

#define RG_SM_RSSI_THRESHOLD   (0x83)
#define SR_RSSI_THRESHOLD      0x83, 0x0F, 0

#define RG_SM_RX_LENGTH_H      (0x84)
#define SR_RX_LENGTH_H         0x84, 0x7F, 0

#define RG_SM_RX_LENGTH_M      (0x85)
#define SR_RX_LENGTH_M         0x85, 0xFF, 0

#define RG_SM_RX_LENGTH_L      (0x86)
#define SR_RX_LENGTH_L         0x86, 0xFF, 0

#define RG_SM_RX_TIMEOUT_H     (0x87)
#define SR_RX_TIMEOUT_H        0x87, 0xFF, 0

#define RG_SM_RX_TIMEOUT_M     (0x88)
#define SR_RX_TIMEOUT_M        0x88, 0xFF, 0

#define RG_SM_RX_TIMEOUT_L     (0x89)
#define SR_RX_TIMEOUT_L        0x89, 0xFF, 0

#define RG_SM_WAKEUP           (0x8A)
#define SR_WAKEUPONSPI         0x8A, 0x10, 4
#define SR_WAKEUPONRX          0x8A, 0x08, 3
#define SR_WAKEUP_MODES_EN     0x8A, 0x03, 0

#define RG_SM_TX_POWER_CTRL    (0x8B)
#define SR_TX_PWR_CTRL         0x8B, 0x1E, 1
#define SR_TX_MAXAMP           0x8B, 0x01, 0

#define RG_SM_RX_CHAN_INT      (0x8C)
#define SR_RX_CHAN_INT         0x8C, 0x7F, 0

#define RG_SM_RX_CHAN_FRAC_H   (0x8D)
#define SR_RX_CHAN_FRAC_H      0x8D, 0x0F, 0

#define RG_SM_RX_CHAN_FRAC_M   (0x8E)
#define SR_RX_CHAN_FRAC_M      0x8E, 0xFF, 0

#define RG_SM_RX_CHAN_FRAC_L   (0x8F)
#define SR_RX_CHAN_FRAC_L      0x8F, 0xFF, 0

#define RG_SM_TX_CHAN_INT      (0x90)
#define SR_TX_CHAN_INT         0x90, 0x7F, 0

#define RG_SM_TX_CHAN_FRAC_H   (0x91)
#define SR_TX_CHAN_FRAC_H      0x91, 0x0F, 0

#define RG_SM_TX_CHAN_FRAC_M   (0x92)
#define SR_TX_CHAN_FRAC_M      0x92, 0xFF, 0

#define RG_SM_TX_CHAN_FRAC_L   (0x93)
#define SR_TX_CHAN_FRAC_L      0x93, 0xFF, 0

//statemachine timing
#define RG_SM_TIME_POWER_TX    (0xA0)
#define SR_SM_TIME_POWER_TX    0xA0, 0xFF, 0

#define RG_SM_TIME_POWER_RX    (0xA1)
#define SR_SM_TIME_POWER_RX    0xA1, 0xFF, 0

#define RG_SM_TIME_PLL_PON     (0xA2)
#define SR_SM_TIME_PLL_PON     0xA2, 0xFF, 0

#define RG_SM_TIME_PLL_SET     (0xA3)
#define SR_SM_TIME_PLL_SET     0xA3, 0xFF, 0

#define RG_SM_TIME_TX          (0xA4)
#define SR_SM_TIME_TX          0xA4, 0xFF, 0

#define RG_SM_TIME_RX_FE       (0xA5)
#define SR_SM_TIME_RX_FE       0xA5, 0xFF, 0

#define RG_SM_TIME_ADC         (0xA6)
#define SR_SM_TIME_ADC         0xA6, 0xFF, 0

#define RG_SM_TIME_PD_EN       (0xA7)
#define SR_SM_TIME_PD_EN       0xA7, 0xFF, 0

//statemachine outputs
#define RG_SM_STATE            (0xB0)
#define SR_SM_DEEPSLEEP        0xB0, 0x40, 6
#define SR_SM_SLEEP            0xB0, 0x20, 5
#define SR_SM_BUSY             0xB0, 0x10, 4
#define SR_SM_TX_RDY           0xB0, 0x08, 3
#define SR_SM_SENDING          0xB0, 0x04, 2
#define SR_SM_RX_RDY           0xB0, 0x02, 1
#define SR_SM_RECEIVING        0xB0, 0x01, 0

#define RG_SM_FIFO             (0xB1)
#define SR_SM_FIFO_OUT_EN        0xB1, 0x20, 5
#define SR_FIFO_ALMOST_FULL_BIT  0xB1, 0x10, 4
#define SR_FIFO_FULL_BIT         0xB1, 0x08, 3
#define SR_FIFO_ALMOST_EMPTY_BIT 0xB1, 0x04, 2
#define SR_FIFO_EMPTY_BIT        0xB1, 0x02, 1
#define SR_TX_FIFO_EMPTY_BIT     0xB1, 0x01, 0

#define RG_SM_GLOBAL           (0xB2)
#define SR_SM_SPI_ENABLE       0xB2, 0x02, 1
#define SR_SM_OSCI_EN          0xB2, 0x01, 0

#define RG_SM_POWER            (0xB3)
#define SR_SM_BANDGAP_EN       0xB3, 0x40, 6
#define SR_SM_LDO_A            0xB3, 0x20, 5
#define SR_SM_LDO_PLL          0xB3, 0x10, 4
#define SR_SM_LDO_VCO          0xB3, 0x08, 3
#define SR_SM_LDO_TX433        0xB3, 0x04, 2
#define SR_SM_LDO_TX800        0xB3, 0x02, 1
#define SR_SM_LDO_TX24         0xB3, 0x01, 0

#define RG_SM_RX               (0xB4)
#define SR_SM_LNA433_HGAIN     0xB4, 0x80, 7
#define SR_SM_RX_MODE_USE      0xB4, 0x60, 5
#define SR_SM_RX_FE_EN         0xB4, 0x10, 4
#define SR_SM_RX433_PON        0xB4, 0x08, 3
#define SR_SM_RX800_PON        0xB4, 0x04, 2
#define SR_SM_RX24_PON         0xB4, 0x02, 1
#define SR_SM_LNA800_HGAIN     0xB4, 0x01, 0

#define RG_SM_WAKEUP_EN        (0xB5)
#define SR_SM_WAKEUPSE_EN      0xB5, 0x02, 1
#define SR_SM_WAKEUPDIFF_EN    0xB5, 0x01, 0

#define RG_SM_DEM_ADC          (0xB6)
#define SR_SM_DEM_EN           0xB6, 0x04, 2
#define SR_SM_DEM_PD_EN        0xB6, 0x02, 1
#define SR_SM_ADC_PON          0xB6, 0x01, 0

#define RG_SM_PLL_TX           (0xB7)
#define SR_SM_PLL_EN           0xB7, 0x04, 2
#define SR_SM_PLL_MOD_EN       0xB7, 0x02, 1
#define SR_SM_TX_EN            0xB7, 0x01, 0

#define RG_SM_PLL_CHAN_INT     (0xB8)
#define SR_SM_PLL_CHAN_INT     0xB8, 0x7F, 0

#define RG_SM_PLL_CHAN_FRAC_H  (0xB9)
#define SR_SM_PLL_CHAN_FRAC_H  0xB9, 0x0F, 0

#define RG_SM_PLL_CHAN_FRAC_M  (0xBA)
#define SR_SM_PLL_CHAN_FRAC_M  0xBA, 0xFF, 0

#define RG_SM_PLL_CHAN_FRAC_L  (0xBB)
#define SR_SM_PLL_CHAN_FRAC_L  0xBB, 0xFF, 0

#define RG_SM_TX433            (0xBC)
#define SR_SM_TX433_EN         0xBC, 0x20, 5
#define SR_SM_TX433_PWR_CTRL   0xBC, 0x1E, 1
#define SR_SM_TX433_MAXAMP     0xBC, 0x01, 0

#define RG_SM_TX800            (0xBD)
#define SR_SM_TX800_EN         0xBD, 0x20, 5
#define SR_SM_TX800_PWR_CTRL   0xBD, 0x1E, 1
#define SR_SM_TX800_MAXAMP     0xBD, 0x01, 0

#define RG_SM_TX24             (0xBE)
#define SR_SM_TX24_MIXER_PON   0xBE, 0x40, 6
#define SR_SM_TX24_EN          0xBE, 0x20, 5
#define SR_SM_TX24_PWR_CTRL    0xBE, 0x1E, 1
#define SR_SM_TX24_MAXAMP      0xBE, 0x01, 0

//ULP
#define RG_ULP_MAIN            (0xC0)
#define SR_ULP_BG_EN            0xC0, 0x40, 6
#define SR_ULP_BUFFER_EN        0xC0, 0x20, 5
#define SR_ULP_RESET            0xC0, 0x10, 4
#define SR_ULP_EN               0xC0, 0x08, 3
#define SR_ULP_CTRL_BANGBANG_EN 0xC0, 0x04, 2
#define SR_ULP_CTRL_RESETB      0xC0, 0x02, 1
#define SR_ULP_CTRL_EN          0xC0, 0x01, 0

#define RG_ULP_CTRL_CAP_FREQ_H (0xC1)
#define SR_ULP_CTRL_FREQ_H     0xC1, 0x30, 4
#define SR_ULP_CTRL_CAP_INIT_H 0xC1, 0x01, 0

#define RG_ULP_CTRL_CAP_INIT_L (0xC2)
#define SR_ULP_CTRL_CAP_INIT_L 0xC2, 0xFF, 0

#define RG_ULP_CTRL_FREQ_L     (0xC3)
#define SR_ULP_CTRL_FREQ_L     0xC3, 0xFF, 0

#define RG_ULP_CTRL_FIR1_2     (0xC4)
#define SR_ULP_CTRL_FIR1       0xC4, 0x70, 4
#define SR_ULP_CTRL_FIR2       0xC4, 0x07, 0

#define RG_ULP_CTRL_FIR3       (0xC5)
#define SR_ULP_CTRL_FIR3       0xC5, 0x07, 0

#define RG_ULP_BG_CURRENT      (0xC6)
#define SR_ULP_BG_CURRENT      0xC6, 0xFF, 0

#define RG_ULP_BG_TUNE         (0xC7)
#define SR_ULP_BG_TUNE         0xC7, 0x3F, 0

//Crystal Oscillator
#define RG_OSCI_MAIN           (0xD0)
#define SR_OSCI_BUFFER_EN         0xD0, 0x40, 6
#define SR_OSCI_FREQ_INJECT_RESET 0xD0, 0x20, 5
#define SR_OSCI_FREQ_INJECT_EN    0xD0, 0x10, 4
#define SR_OSCI_NEG_RES2_EN       0xD0, 0x08, 3
#define SR_OSCI_NEG_RES1_EN       0xD0, 0x04, 2
#define SR_OSCI_R_FB_EN           0xD0, 0x02, 1
#define SR_OSCI_CAP_RESET         0xD0, 0x01, 0

#define RG_OSCI_CTRL           (0xD1)
#define SR_OSCI_CTRL_EN          0xD1, 0x10, 4
#define SR_OSCI_CTRL_USE_NEG_RES 0xD1, 0x08, 3
#define SR_OSCI_CTRL_USE_INV     0xD1, 0x04, 2
#define SR_OSCI_CTRL_USE_INJECT  0xD1, 0x02, 1
#define SR_OSCI_CTRL_RESETB      0xD1, 0x01, 0

#define RG_OSCI_INJECT_CYCLES  (0xD2)
#define SR_OSCI_CTRL_INJECT_CYCLES  0xD2, 0x7F, 0

#define RG_OSCI_STARTUP_CYCLES (0xD3)
#define SR_OSCI_CTRL_STARTUP_CYCLES  0xD4, 0x7F, 0

#define RG_OSCI_CAP_L          (0xD4)
#define SR_OSCI_CAP_L          0xD4, 0xFF, 0

#define RG_OSCI_CAP_H          (0xD5)
#define SR_OSCI_CAP_H          0xD5, 0x0F, 0

#define RG_OSCI_CURRENT        (0xD6)
#define SR_OSCI_CURRENT        0xD6, 0x3F, 0

#define RG_OSCI_INJECT_FREQ    (0xD7)
#define SR_OSCI_INJECT_FREQ    0xD7, 0x3F, 0

#define RG_OSCI_INJECT_CURRENT (0xD8)
#define SR_OSCI_INJECT_CURRENT 0xD8, 0x3F, 0

#define RG_OSCI_CHIRP          (0xD9)
#define SR_OSCI_CHIRP_SPEED    0xD9, 0x70, 4
#define SR_OSCI_CHIRP_RANGE    0xD9, 0x0F, 0

//Wakeup RX
#define RG_WAKEUP              (0xE0)
#define SR_WUR_DEBUG_PATH      0xE0, 0xC0, 6
#define SR_WAKEUP_MODE         0xE0, 0x30, 4
#define SR_WUR_RESETB          0xE0, 0x08, 3
#define SR_WAKEUP_VCO_ALW_ON   0xE0, 0x04, 2
#define SR_WAKEUPSE_EN         0xE0, 0x02, 1
#define SR_WAKEUPDIFF_EN       0xE0, 0x01, 0

#define RG_WAKEUP_DIV_RATIO    (0xE1)
#define SR_WUR_DIV_RATIO       0xE1, 0xFF, 0

#define RG_WAKEUP_HIGH_PHASE   (0xE2)
#define SR_WUR_HIGH_PHASE      0xE2, 0xFF, 0

#define RG_WAKEUP_CORR         (0xE3)
#define SR_WUR_CORR            0xE3, 0xFF, 0

#define RG_WAKEUP_TIMEOUT_CNT  (0xE4)
#define SR_WUR_TIMEOUT_CNT     0xE4, 0xFF, 0

#define RG_WAKEUP_PATTERN_0    (0xE5)
#define SR_WUR_PATTERN_0       0xE5, 0xFF, 0

#define RG_WAKEUP_PATTERN_1    (0xE6)
#define SR_WUR_PATTERN_1       0xE6, 0xFF, 0

#define RG_WAKEUP_PATTERN_2    (0xE7)
#define SR_WUR_PATTERN_2       0xE7, 0xFF, 0

#define RG_WAKEUP_PATTERN_3    (0xE8)
#define SR_WUR_PATTERN_3       0xE8, 0xFF, 0

#define RG_WUP_DIFF_LNA_CTRIM  (0xE9)
#define SR_WUP_LNA24_CTRIM     0xE9, 0x1F, 0

#define RG_WUP_DIFF_LNA_SET    (0xEA)
#define SR_WUP_LNA24_ISETT     0xEA, 0x70, 4
#define SR_WUP_LNA24_SPCTRM    0xEA, 0x0F, 0

#define RG_WUP_DIFF_PON_BLOCK  (0xEB)
#define SR_WUP_DIFF_PON_BLOCK  0xEB, 0x7F, 0

#define RG_WUP_DIFF_S_VREF     (0xEC)
#define SR_WUP_DIFF_S_VREF     0xEC, 0xFF, 0

#define RG_WUP_DIFF_EN_REF     (0xED)
#define SR_WUP_DIFF_REFF_RHI   0xED, 0x40, 6
#define SR_WUP_DIFF_REFF_RSH   0xED, 0x20, 5
#define SR_WUP_DIFF_REFF_SW    0xED, 0x10, 4
#define SR_WUP_DIFF_EN_LP_B    0xED, 0x08, 3
#define SR_WUP_DIFF_EN_ED      0xED, 0x04, 2
#define SR_WUP_DIFF_EN_VGA_B   0xED, 0x02, 1
#define SR_WUP_DIFF_EN_REF_B   0xED, 0x01, 0

#define RG_WUP_DIFF_VGA_LP     (0xEE)
#define SR_WUP_DIFF_VGA_RHI    0xEE, 0x40, 6
#define SR_WUP_DIFF_VGA_RSH    0xEE, 0x20, 5
#define SR_WUP_DIFF_LP_RHI     0xEE, 0x10, 4
#define SR_WUP_DIFF_LP_RSH     0xEE, 0x08, 3
#define SR_WUP_DIFF_LP_SW      0xEE, 0x04, 2
#define SR_WUP_DIFF_LP_FULL    0xEE, 0x02, 1
#define SR_WUP_DIFF_LP_HALF    0xEE, 0x01, 0

#define RG_WUP_SE_PON_BLOCK    (0xEF)
#define SR_WUP_SE_PON_FED      0xEF, 0x40, 6
#define SR_WUP_SE_PON_VGA      0xEF, 0x20, 5
#define SR_WUP_SE_PON_VREF     0xEF, 0x10, 4
#define SR_WUP_SE_PON_ST       0xEF, 0x0F, 0

#define RG_WUP_SE_PATH_S       (0xF0)
#define SR_WUP_SE_S165         0xF0, 0x80, 7
#define SR_WUP_SE_S210         0xF0, 0x40, 6
#define SR_WUP_SE_S255         0xF0, 0x20, 5
#define SR_WUP_SE_S300         0xF0, 0x10, 4
#define SR_WUP_SE_PATH         0xF0, 0x0F, 0

#define RG_WUP_SE_EN_B         (0xF1)
#define SR_WUP_SE_EN_B         0xF1, 0x3F, 0

#define RG_WUP_SE_VREF         (0xF2)
#define SR_WUP_SE_VREF         0xF2, 0xFF, 0

#define RG_WUP_SE_VREF_LPF     (0xF3)
#define SR_WUP_SE_VREF_RSH     0xF3, 0x40, 6
#define SR_WUP_SE_VREF_RHI     0xF3, 0x20, 5
#define SR_WUP_SE_VREF_SW      0xF3, 0x10, 4
#define SR_WUP_SE_LPF_RSH      0xF3, 0x04, 2
#define SR_WUP_SE_LPF_RHI      0xF3, 0x02, 1
#define SR_WUP_SE_LPF_SW       0xF3, 0x01, 0



// state machine
// commands, (SM_MAIN)
#define STATE_CMD_NONE      0
#define STATE_CMD_SLEEP     2
#define STATE_CMD_DEEPSLEEP 3
#define STATE_CMD_TX        4
#define STATE_CMD_TXIDLE    5
#define STATE_CMD_RX        6
#define STATE_CMD_RXHOLD    7

// state bits, SM_STATE
#define STATE_DEEPSLEEP     0x40
#define STATE_SLEEP         0x20
#define STATE_BUSY          0x10
#define STATE_TX_RDY        0x08
#define STATE_SENDING       0x04
#define STATE_RX_RDY        0x02
#define STATE_RECEIVING     0x01

// settling times PLL
#define T_POWER_TX_TIME     0xFF
#define T_POWER_RX_TIME     0x20
#define T_PLL_PON_TIME      0xFF
#define T_PLL_SET_TIME      0x60
#define T_TX_TIME           0xFF
#define T_PD_EN_TIME        0x20


