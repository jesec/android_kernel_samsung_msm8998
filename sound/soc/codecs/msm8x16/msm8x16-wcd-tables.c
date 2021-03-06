/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm8x16-wcd.h"

const u8 msm89xx_pmic_cdc_reg_readable[MSM89XX_PMIC_CDC_CACHE_SIZE] = {
		[MSM89XX_PMIC_DIGITAL_REVISION1] = 1,
		[MSM89XX_PMIC_DIGITAL_REVISION2] = 1,
		[MSM89XX_PMIC_DIGITAL_PERPH_TYPE] = 1,
		[MSM89XX_PMIC_DIGITAL_PERPH_SUBTYPE] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_RT_STS] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_SET_TYPE] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_POLARITY_HIGH] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_POLARITY_LOW] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_EN_SET] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_EN_CLR] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_LATCHED_STS] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_PENDING_STS] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_MID_SEL] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_PRIORITY] = 1,
		[MSM89XX_PMIC_DIGITAL_GPIO_MODE] = 1,
		[MSM89XX_PMIC_DIGITAL_PIN_CTL_OE] = 1,
		[MSM89XX_PMIC_DIGITAL_PIN_CTL_DATA] = 1,
		[MSM89XX_PMIC_DIGITAL_PIN_STATUS] = 1,
		[MSM89XX_PMIC_DIGITAL_HDRIVE_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_RST_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_TOP_CLK_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_ANA_CLK_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_DIG_CLK_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_TX1_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_TX2_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_HPHR_DAC_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_RX1_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_RX2_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_RX3_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_CONN_RX_LB_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_RX_CTL1] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_RX_CTL2] = 1,
		[MSM89XX_PMIC_DIGITAL_CDC_RX_CTL3] = 1,
		[MSM89XX_PMIC_DIGITAL_DEM_BYPASS_DATA0] = 1,
		[MSM89XX_PMIC_DIGITAL_DEM_BYPASS_DATA1] = 1,
		[MSM89XX_PMIC_DIGITAL_DEM_BYPASS_DATA2] = 1,
		[MSM89XX_PMIC_DIGITAL_DEM_BYPASS_DATA3] = 1,
		[MSM89XX_PMIC_DIGITAL_DIG_DEBUG_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_SPARE_0] = 1,
		[MSM89XX_PMIC_DIGITAL_SPARE_1] = 1,
		[MSM89XX_PMIC_DIGITAL_SPARE_2] = 1,
		[MSM89XX_PMIC_ANALOG_REVISION1] = 1,
		[MSM89XX_PMIC_ANALOG_REVISION2] = 1,
		[MSM89XX_PMIC_ANALOG_REVISION3] = 1,
		[MSM89XX_PMIC_ANALOG_REVISION4] = 1,
		[MSM89XX_PMIC_ANALOG_PERPH_TYPE] = 1,
		[MSM89XX_PMIC_ANALOG_PERPH_SUBTYPE] = 1,
		[MSM89XX_PMIC_ANALOG_INT_RT_STS] = 1,
		[MSM89XX_PMIC_ANALOG_INT_SET_TYPE] = 1,
		[MSM89XX_PMIC_ANALOG_INT_POLARITY_HIGH] = 1,
		[MSM89XX_PMIC_ANALOG_INT_POLARITY_LOW] = 1,
		[MSM89XX_PMIC_ANALOG_INT_EN_SET] = 1,
		[MSM89XX_PMIC_ANALOG_INT_EN_CLR] = 1,
		[MSM89XX_PMIC_ANALOG_INT_LATCHED_STS] = 1,
		[MSM89XX_PMIC_ANALOG_INT_PENDING_STS] = 1,
		[MSM89XX_PMIC_ANALOG_INT_MID_SEL] = 1,
		[MSM89XX_PMIC_ANALOG_INT_PRIORITY] = 1,
		[MSM89XX_PMIC_ANALOG_MICB_1_EN] = 1,
		[MSM89XX_PMIC_ANALOG_MICB_1_VAL] = 1,
		[MSM89XX_PMIC_ANALOG_MICB_1_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MICB_1_INT_RBIAS] = 1,
		[MSM89XX_PMIC_ANALOG_MICB_2_EN] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_ATEST_CTL_2] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_DET_CTL_1] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_DET_CTL_2] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_FSM_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_DBNC_TIMER] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN0_ZDETL_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN1_ZDETM_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN2_ZDETH_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN3_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN4_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_BTN_RESULT] = 1,
		[MSM89XX_PMIC_ANALOG_MBHC_ZDET_ELECT_RESULT] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_EN] = 1,
		[MSM89XX_PMIC_ANALOG_TX_2_EN] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_TEST_CTL_1] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_TEST_CTL_2] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_ATEST_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_OPAMP_BIAS] = 1,
		[MSM89XX_PMIC_ANALOG_TX_1_2_TXFE_CLKDIV] = 1,
		[MSM89XX_PMIC_ANALOG_TX_3_EN] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_EN] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_CLK] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_DEGLITCH] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_FBCTRL] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_BIAS] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_VCTRL] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_TEST] = 1,
		[MSM89XX_PMIC_ANALOG_RX_CLOCK_DIVIDER] = 1,
		[MSM89XX_PMIC_ANALOG_RX_COM_OCP_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_RX_COM_OCP_COUNT] = 1,
		[MSM89XX_PMIC_ANALOG_RX_COM_BIAS_DAC] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_BIAS_PA] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_BIAS_LDO_OCP] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_BIAS_CNP] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_CNP_EN] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_CNP_WG_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_CNP_WG_TIME] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_L_TEST] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_L_PA_DAC_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_R_TEST] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_R_PA_DAC_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_RX_EAR_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_RX_ATEST] = 1,
		[MSM89XX_PMIC_ANALOG_RX_HPH_STATUS] = 1,
		[MSM89XX_PMIC_ANALOG_RX_EAR_STATUS] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DAC_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DRV_CLIP_DET] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DRV_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_ANA_BIAS_SET] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_OCP_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_PWRSTG_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DRV_MISC] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DRV_DBG] = 1,
		[MSM89XX_PMIC_ANALOG_CURRENT_LIMIT] = 1,
		[MSM89XX_PMIC_ANALOG_OUTPUT_VOLTAGE] = 1,
		[MSM89XX_PMIC_ANALOG_BYPASS_MODE] = 1,
		[MSM89XX_PMIC_ANALOG_BOOST_EN_CTL] = 1,
		[MSM89XX_PMIC_ANALOG_SLOPE_COMP_IP_ZERO] = 1,
		[MSM89XX_PMIC_ANALOG_RDSON_MAX_DUTY_CYCLE] = 1,
		[MSM89XX_PMIC_ANALOG_BOOST_TEST1_1] = 1,
		[MSM89XX_PMIC_ANALOG_BOOST_TEST_2] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_SAR_STATUS] = 1,
		[MSM89XX_PMIC_ANALOG_SPKR_DRV_STATUS] = 1,
		[MSM89XX_PMIC_ANALOG_PBUS_ADD_CSR] = 1,
		[MSM89XX_PMIC_ANALOG_PBUS_ADD_SEL] = 1,
		[MSM89XX_PMIC_ANALOG_MASTER_BIAS_CTL] = 1,
		[MSM89XX_PMIC_DIGITAL_INT_LATCHED_CLR] = 1,
		[MSM89XX_PMIC_ANALOG_INT_LATCHED_CLR] = 1,
		[MSM89XX_PMIC_ANALOG_NCP_CLIM_ADDR] = 1,
		[MSM89XX_PMIC_DIGITAL_SEC_ACCESS] = 1,
		[MSM89XX_PMIC_DIGITAL_PERPH_RESET_CTL3] = 1,
		[MSM89XX_PMIC_ANALOG_SEC_ACCESS] = 1,
};

const u8 msm89xx_cdc_core_reg_readable[MSM89XX_CDC_CORE_CACHE_SIZE] = {
		[MSM89XX_CDC_CORE_CLK_RX_RESET_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_TX_RESET_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_DMIC_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_RX_I2S_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_TX_I2S_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_OTHR_RESET_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_TX_CLK_EN_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_OTHR_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_RX_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_MCLK_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_PDM_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_SD_CTL] = 1,
		[MSM89XX_CDC_CORE_CLK_WSA_VI_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B5_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B5_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B5_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_B6_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_B6_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_B6_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_VOL_CTL_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_VOL_CTL_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_VOL_CTL_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_RX1_VOL_CTL_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_RX2_VOL_CTL_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_RX3_VOL_CTL_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_TOP_GAIN_UPDATE] = 1,
		[MSM89XX_CDC_CORE_TOP_CTL] = 1,
		[MSM89XX_CDC_CORE_DEBUG_DESER1_CTL] = 1,
		[MSM89XX_CDC_CORE_DEBUG_DESER2_CTL] = 1,
		[MSM89XX_CDC_CORE_DEBUG_B1_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_DEBUG_B2_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_DEBUG_B3_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B5_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B5_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B6_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B6_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B7_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B7_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_B8_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_B8_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_GAIN_TIMER_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_GAIN_TIMER_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_COEF_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_COEF_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR1_COEF_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_IIR2_COEF_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX1_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX1_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX1_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX2_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX2_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX2_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX3_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_RX3_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_TX_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ1_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ1_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ1_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ1_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ2_B1_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ2_B2_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ2_B3_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_EQ2_B4_CTL] = 1,
		[MSM89XX_CDC_CORE_CONN_TX_I2S_SD1_CTL] = 1,
		[MSM89XX_CDC_CORE_TX1_VOL_CTL_TIMER] = 1,
		[MSM89XX_CDC_CORE_TX2_VOL_CTL_TIMER] = 1,
		[MSM89XX_CDC_CORE_TX3_VOL_CTL_TIMER] = 1,
		[MSM89XX_CDC_CORE_TX4_VOL_CTL_TIMER] = 1,
		[MSM89XX_CDC_CORE_TX1_VOL_CTL_GAIN] = 1,
		[MSM89XX_CDC_CORE_TX2_VOL_CTL_GAIN] = 1,
		[MSM89XX_CDC_CORE_TX3_VOL_CTL_GAIN] = 1,
		[MSM89XX_CDC_CORE_TX4_VOL_CTL_GAIN] = 1,
		[MSM89XX_CDC_CORE_TX1_VOL_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_TX2_VOL_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_TX3_VOL_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_TX4_VOL_CTL_CFG] = 1,
		[MSM89XX_CDC_CORE_TX1_MUX_CTL] = 1,
		[MSM89XX_CDC_CORE_TX2_MUX_CTL] = 1,
		[MSM89XX_CDC_CORE_TX3_MUX_CTL] = 1,
		[MSM89XX_CDC_CORE_TX4_MUX_CTL] = 1,
		[MSM89XX_CDC_CORE_TX1_CLK_FS_CTL] = 1,
		[MSM89XX_CDC_CORE_TX2_CLK_FS_CTL] = 1,
		[MSM89XX_CDC_CORE_TX3_CLK_FS_CTL] = 1,
		[MSM89XX_CDC_CORE_TX4_CLK_FS_CTL] = 1,
		[MSM89XX_CDC_CORE_TX1_DMIC_CTL] = 1,
		[MSM89XX_CDC_CORE_TX2_DMIC_CTL] = 1,
		[MSM89XX_CDC_CORE_TX3_DMIC_CTL] = 1,
		[MSM89XX_CDC_CORE_TX4_DMIC_CTL] = 1,
};
