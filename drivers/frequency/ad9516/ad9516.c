/***************************************************************************//**
 *   @file   AD9516.c
 *   @brief  Implementation of AD9516 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *   @author Prerna Baranwal, modified for kcu116 and ad9144
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include <errno.h>
#include "no_os_delay.h"
#include "ad9516.h"


/***************************************************************************//**
 * @brief Initializes the AD9516.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
int32_t ad9516_setup(struct ad9516_dev **device,
		     struct ad9516_init_param *init_param)
{
	int32_t		    ret = 0;
	int8_t		    index = 0;
	uint16_t	    reg_address = 0;
	uint32_t	    reg_value = 0;
	struct ad9516_dev   *dev;

	static uint32_t lvepcl_out_ch[6] = {
		AD9516_REG_LVPECL_OUT0,
		AD9516_REG_LVPECL_OUT1,
		AD9516_REG_LVPECL_OUT2,
		AD9516_REG_LVPECL_OUT3,
		AD9516_REG_LVPECL_OUT4,
		AD9516_REG_LVPECL_OUT5
	};

	dev = (struct ad9516_dev *)malloc(sizeof(*dev));
	if (!dev) {
		printf("Memory allocation for 'dev' failed.\n");
		return -1;
	}

	dev->ad9516_st = init_param->ad9516_st;
	dev->ad9516_type = init_param->ad9516_type;

	/* Initializes the SPI peripheral */
	ret = no_os_spi_init(&dev->spi_desc, &init_param->spi_init);
	if (ret) {
		printf("SPI initialization failed with error code: %d\n", ret);
		return ret;
	}

	printf("Reading PART ID from register address: 0x%X\n", AD9516_REG_PART_ID);
    ret = ad9516_read(dev, AD9516_REG_PART_ID, &reg_value);
    if (ret) {
    printf("Failed to read PART ID register. Error code: %d\n", ret);
    return ret;
    }
printf("PART ID read: 0x%X\n", reg_value);

	ret = ad9516_read(dev, AD9516_REG_PART_ID, &reg_value);
	if (ret) {
		printf("Failed to read PART ID register. Error code: %d\n", ret);
		return ret;
	}
	if (reg_value != dev->ad9516_type) {
		printf("PART ID mismatch. Expected: 0x%X, Got: 0x%X\n", dev->ad9516_type, reg_value);
		return -EFAULT;
	}

	/* Configure serial port for long instructions and reset the serial interface. */
	ret = ad9516_write(dev, AD9516_REG_SERIAL_PORT_CONFIG, AD9516_SOFT_RESET | AD9516_LONG_INSTRUCTION);
	if (ret < 0) {
		printf("Failed to write to SERIAL PORT CONFIG register. Error code: %d\n", ret);
		return ret;
	}
	ret = ad9516_update(dev);
	if (ret < 0) {
		printf("Failed to update serial port configuration. Error code: %d\n", ret);
		return ret;
	}

	/* Clear AD9516_SOFT_RESET bit to complete reset operation. */
	ret = ad9516_write(dev, AD9516_REG_SERIAL_PORT_CONFIG, AD9516_LONG_INSTRUCTION);
	if (ret < 0) {
		printf("Failed to clear SOFT_RESET bit. Error code: %d\n", ret);
		return ret;
	}
	ret = ad9516_update(dev);
	if (ret < 0) {
		printf("Failed to update serial port configuration after clearing reset. Error code: %d\n", ret);
		return ret;
	}

	/* Selects the PLL reference mode. */
	reg_value = dev->ad9516_st.pdata->diff_ref_en * AD9516_DIFF_REF |
		    dev->ad9516_st.pdata->ref_1_power_on * AD9516_REF1_POWER_ON |
		    dev->ad9516_st.pdata->ref_2_power_on * AD9516_REF2_POWER_ON |
		    dev->ad9516_st.pdata->ref_sel_pin_en * AD9516_USE_REF_SEL_PIN |
		    dev->ad9516_st.pdata->ref_2_en * AD9516_SELECT_REF2;

	ret = ad9516_write(dev, AD9516_REG_PLL_CTRL_7, reg_value);
	if (ret < 0) {
		printf("Failed to write PLL reference mode. Error code: %d\n", ret);
		return ret;
	}

	/* Select CLK input. */
	reg_value = dev->ad9516_st.pdata->vco_clk_sel * AD9516_SEL_VCO_CLK |
		    dev->ad9516_st.pdata->power_down_vco_clk * AD9516_POWER_DOWN_VCO_CLK;

	ret = ad9516_write(dev, AD9516_REG_INPUT_CLKS, reg_value);
	if (ret < 0) {
		printf("Failed to select CLK input. Error code: %d\n", ret);
		return ret;
	}

	/* Update the device with user settings for the LVPECL output channels. */
	for (index = 0; index < 4; index++) {
		reg_address = lvepcl_out_ch[index];

		reg_value = dev->ad9516_st.lvpecl_channels[index].out_invert_en * AD9516_OUT_LVPECL_INVERT |
		            AD9516_OUT_LVPECL_DIFF_VOLTAGE(dev->ad9516_st.lvpecl_channels[index].out_diff_voltage);

		ret = ad9516_write(dev, reg_address, reg_value);
		if (ret < 0) {
			printf("Failed to configure LVPECL output channel %d. Error code: %d\n", index, ret);
			return ret;
		}
	}

	/* Update the device with user settings for the LVDS/CMOS output channels. */
	for (index = 0; index < 4; index++) {
		reg_address = AD9516_REG_LVDS_CMOS_OUT6 + index;
		reg_value = AD9516_OUT_LVDS_CMOS_INVERT(dev->ad9516_st.lvds_cmos_channels[index].out_invert) |
		            dev->ad9516_st.lvds_cmos_channels[index].logic_level * AD9516_OUT_LVDS_CMOS |
		            dev->ad9516_st.lvds_cmos_channels[index].cmos_b_en * AD9516_OUT_CMOS_B |
		            AD9516_OUT_LVDS_OUTPUT_CURRENT(dev->ad9516_st.lvds_cmos_channels[index].out_lvds_current);

		ret = ad9516_write(dev, reg_address, reg_value);
		if (ret < 0) {
			printf("Failed to configure LVDS/CMOS output channel %d. Error code: %d\n", index, ret);
			return ret;
		}
	}

	/* Check if VCO is selected as input. */
	if (dev->ad9516_st.pdata->vco_clk_sel) {
		/* Sets the VCO frequency. */
		ad9516_vco_frequency(dev, dev->ad9516_st.pdata->int_vco_freq);

		/* Activate PLL */
		reg_value = AD9516_PLL_POWER_DOWN(0x0) | AD9516_CP_MODE(0x3) | AD9516_CP_CURRENT(0x7) | 0 * AD9516_PFD_POLARITY;
		ret = ad9516_write(dev, AD9516_REG_PFD_CHARGE_PUMP, reg_value);
		if (ret < 0) {
			printf("Failed to activate PLL. Error code: %d\n", ret);
			return ret;
		}

		/* Start VCO Calibration */
		ret = ad9516_read(dev, AD9516_REG_PLL_CTRL_3, &reg_value);
		if (ret < 0) {
			printf("Failed to read PLL control register for VCO calibration. Error code: %d\n", ret);
			return ret;
		}

		reg_value &= ~AD9516_VCO_CAL_NOW;

		ret = ad9516_write(dev, AD9516_REG_PLL_CTRL_3, reg_value);
		if (ret < 0) {
			printf("Failed to clear VCO_CAL_NOW bit. Error code: %d\n", ret);
			return ret;
		}

		ret = ad9516_update(dev);
		if (ret < 0) {
			printf("Failed to update VCO calibration configuration. Error code: %d\n", ret);
			return ret;
		}

		reg_value |= AD9516_VCO_CAL_NOW;

		ret = ad9516_write(dev, AD9516_REG_PLL_CTRL_3, reg_value);
		if (ret < 0) {
			printf("Failed to set VCO_CAL_NOW bit. Error code: %d\n", ret);
			return ret;
		}

		ret = ad9516_update(dev);
		if (ret < 0) {
			printf("Failed to complete VCO calibration update. Error code: %d\n", ret);
			return ret;
		}

		/* Time to complete a VCO calibration (Table 29, datasheet). */
		no_os_mdelay(88);
	}

	*device = dev;

	return ret;
}


/***************************************************************************//**
 * @brief Free the resources allocated by ad9516_setup().
 *
 * @param dev - The device structure.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad9516_remove(struct ad9516_dev *dev)
{
	int32_t ret;

	ret = no_os_spi_remove(dev->spi_desc);

	free(dev);

	return ret;
}

/***************************************************************************//**
* @brief Writes data into a register.
*
* @param dev      - The device structure.
* @param reg_addr - The address of the register to be written.
* @param reg_val  - The value to be written into the register.
*
* @return Returns 0 in case of success or negative error code.
*******************************************************************************/
int32_t ad9516_write(struct ad9516_dev *dev,
		     uint32_t reg_addr,
		     uint16_t reg_val)
{
	uint8_t i = 0;
	int32_t ret = 0;
	uint16_t reg_address = 0;
	int8_t reg_value = 0;
	uint8_t tx_buffer[3] = {0, 0, 0};

	reg_address = AD9516_WRITE + AD9516_ADDR(reg_addr);
	for(i = 0; i < AD9516_TRANSF_LEN(reg_addr); i++) {
		reg_value    = (reg_val >> ((AD9516_TRANSF_LEN(reg_addr) -
					     i - 1) * 8)) & 0xFF;
		tx_buffer[0] = (reg_address & 0xFF00) >> 8;
		tx_buffer[1] = reg_address & 0x00FF;
		tx_buffer[2] = reg_value;

		ret = no_os_spi_write_and_read(dev->spi_desc, tx_buffer, 3);

		if(ret < 0)
			return ret;
		reg_address--;
	}

	return ret;
}

/***************************************************************************//**
* @brief Reads data from a register.
*
* @param dev      - The device structure.
* @param reg_addr - The address of the register to be read.
* @param reg_value - Pointer to the value to be read from the register.
*
* @return Returns the read data or negative error code.
*******************************************************************************/
int32_t ad9516_read(struct ad9516_dev *dev,
		    uint32_t reg_addr,
		    uint32_t *reg_value)
{
	uint32_t reg_address = 0;
	int32_t ret = 0;
	uint8_t tx_buffer[3] = {0, 0, 0};
	uint8_t i = 0;
	*reg_value = 0;

	reg_address = AD9516_READ + AD9516_ADDR(reg_addr);
	for(i = 0; i < AD9516_TRANSF_LEN(reg_addr); i++) {
		tx_buffer[0] = (reg_address & 0xFF00) >> 8;
		tx_buffer[1] = reg_address & 0x00FF;
		tx_buffer[2] = 0;

		ret = no_os_spi_write_and_read(dev->spi_desc, tx_buffer, 3);

		reg_address--;

		*reg_value <<= 8;

		*reg_value |= tx_buffer[2];
	}

	return ret;
}

/***************************************************************************//**
 * @brief Transfers the contents of the buffer registers into the active
 *        registers.
 *
 * @param dev - The device structure.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
int32_t ad9516_update(struct ad9516_dev *dev)
{
	return ad9516_write(dev,
			    AD9516_REG_UPDATE_ALL_REGS,
			    AD9516_UPDATE_ALL_REGS);
}


/***************************************************************************//**
 * @brief Sets the VCO frequency.
 *
 * @param dev       - The device structure.
 * @param frequency - The desired frequency value.
 *
 * @return vco_freq - The actual frequency value that was set.
*******************************************************************************/
int64_t ad9516_vco_frequency(struct ad9516_dev *dev,
			     int64_t frequency)
{
	uint32_t ref_freq = 0;
	uint32_t pfd_freq = 0;
	int32_t n_divider = 0;
	int32_t prescaler_value[5] = {2, 4, 8, 16, 32};
	int64_t prescaler_limit[5] = {200000000ul, 1000000000ul, 2400000000ul,
				      3000000000ul, 3000000000ul
				     };
	int32_t index = 0;
	uint32_t vco_freq = 0;
	int32_t good_values = 0;
	uint8_t prescaler = 0;
	uint32_t reg_value = 0;

	switch(dev->ad9516_type) {
	case AD9516_1:
		if((frequency < AD9516_1_MIN_VCO_FREQ) ||
		    (frequency > AD9516_1_MAX_VCO_FREQ))
			return -1;
		break;
	case AD9516_2:
		if((frequency < AD9516_2_MIN_VCO_FREQ) ||
		    (frequency > AD9516_2_MAX_VCO_FREQ))
			return -1;
		break;
	case AD9516_3:
		if((frequency < AD9516_3_MIN_VCO_FREQ) ||
		    (frequency > AD9516_3_MAX_VCO_FREQ))
			return -1;
		break;
	case AD9516_4:
		if((frequency < AD9516_4_MIN_VCO_FREQ) ||
		    (frequency > AD9516_4_MAX_VCO_FREQ))
			return -1;
		break;
	default:
		return -1;
	}
	if(dev->ad9516_st.pdata->ref_sel_pin_en)
		/* Reference selection is made using REF_SEL pin. */
		ref_freq = dev->ad9516_st.pdata->ref_sel_pin ?
			   dev->ad9516_st.pdata->ref_2_freq : dev->
			   ad9516_st.pdata->ref_1_freq;
	else
		/* Reference selection is made using bit AD9516_SELECT_REF2 from
		 * AD9516_REG_PLL_CTRL_7. */
		ref_freq = dev->ad9516_st.pdata->ref_2_en ?
			   dev->ad9516_st.pdata->ref_2_freq : dev->
			   ad9516_st.pdata->ref_1_freq;
	dev->ad9516_st.r_counter = 1;
	pfd_freq = ref_freq / dev->ad9516_st.r_counter;
	while(pfd_freq > AD9516_MAX_PFD_FREQ) {
		dev->ad9516_st.r_counter++;
		pfd_freq = ref_freq / dev->ad9516_st.r_counter;
	}
	/* Dual Modulus Mode */
	while(good_values == 0) {
		for(index = 0; index < 5; index++) {
			if(frequency <= prescaler_limit[index]) {
				n_divider = (int32_t)(frequency / pfd_freq);
				dev->
				ad9516_st.prescaler_p = prescaler_value[index];
				prescaler = index + 2;
				dev->ad9516_st.b_counter = n_divider /
							   dev->ad9516_st.prescaler_p;
				dev->ad9516_st.a_counter = n_divider %
							   dev->ad9516_st.prescaler_p;
				if((dev->ad9516_st.b_counter >= 3) &&
				    ((dev->ad9516_st.b_counter >
				      dev->ad9516_st.a_counter))) {
					good_values = 1;
					break;
				}
			}
		}
		if(good_values == 0) {
			dev->ad9516_st.r_counter++;
			pfd_freq = ref_freq / dev->ad9516_st.r_counter;
		}
	}
	if(pfd_freq > 50000000)
		/* This setting may be necessary if the PFD frequency >
		 * 50 MHz. */
		dev->ad9516_st.antibacklash_pulse_width = 1;
	ad9516_read(dev, AD9516_REG_PLL_CTRL_1, &reg_value);
	if((int32_t)reg_value < 0)
		return reg_value;
	reg_value &= ~AD9516_PRESCALER_P(0x7);
	reg_value |= AD9516_PRESCALER_P(prescaler);
	ad9516_write(dev, AD9516_REG_PLL_CTRL_1, reg_value);
	ad9516_write(dev,
		     AD9516_REG_A_COUNTER,
		     AD9516_A_COUNTER(dev->ad9516_st.a_counter));
	ad9516_write(dev,
		     AD9516_REG_B_COUNTER,
		     AD9516_B_COUNTER(dev->ad9516_st.b_counter));
	ad9516_write(dev,
		     AD9516_REG_R_COUNTER,
		     AD9516_R_COUNTER(dev->ad9516_st.r_counter));

	/* Compute the frequency obtained with the calculated values. */
	vco_freq = (ref_freq / dev->ad9516_st.r_counter) *
		   (dev->ad9516_st.prescaler_p * dev->ad9516_st.b_counter +
		    dev->ad9516_st.a_counter);

	/* Update vco_freq value. */
	dev->ad9516_st.pdata->int_vco_freq = vco_freq;

	return vco_freq;
}

/***************************************************************************//**
 * @brief Checks if the number can be decomposed into a product of two numbers
 *        smaller or equal to 32 each one.
 *
 * @param number - The number.
 *
 * @return Returns 1 if the number can't be decomposed or 0 otherwise.
*******************************************************************************/
int8_t dividers_checker(int32_t number)
{
	int32_t i = 0;

	for(i = 1; i*i <= number; i++) {
		if(number % i == 0) {
			if(i <= 32) {
				if((number / i) <= 32) {
					return 0;
				}
			}
		}
	}

	return 1;
}

/***************************************************************************//**
 * @brief Sets the frequency on the specified channel.
 *
 * @param dev       - The device structure.
 * @param channel   - The channel.
 * @param frequency - The desired frequency value.
 *
 * @return set_freq - The actual frequency value that was set.
*******************************************************************************/
int64_t ad9516_frequency(struct ad9516_dev *dev,
			 int32_t channel,
			 int64_t frequency)
{
	/* The frequency that is applied to the channel dividers. */
	int64_t freq_to_chan_div = 0;
	int64_t freq_to_chan_div_backup = 0;
	int64_t freq_0_value = 0;
	int32_t freq_0_divider = 0;
	int64_t freq_1_value = 0;
	int32_t freq_1_divider = 0;
	int32_t divider = 0;
	int32_t divider_max = 0;
	int32_t divider_1 = 0;
	int32_t divider_2 = 0;
	int64_t set_freq = 0;
	uint32_t reg_value = 0;
	int32_t reg_address = 0;
	int32_t ret = 0;

	if(dev->ad9516_st.pdata->vco_clk_sel) {
		/* VCO is selected as input. */
		freq_to_chan_div = dev->ad9516_st.pdata->int_vco_freq;
		freq_to_chan_div_backup = freq_to_chan_div;
		/* VCO divider cannot be bypassed when VCO is selected as
		 * input. */
		dev->ad9516_st.vco_divider = 2;
		freq_to_chan_div >>= 1;
	} else {
		/* External Clock is selected as input. */
		freq_to_chan_div = dev->ad9516_st.pdata->ext_clk_freq;
		freq_to_chan_div_backup = freq_to_chan_div;
		dev->ad9516_st.vco_divider = 1;
	}
	/* The maximum frequency that can be applied to the channel dividers
	 * is 1600 MHz. */
	while(freq_to_chan_div > 1600000000) {
		dev->ad9516_st.vco_divider++;
		freq_to_chan_div = freq_to_chan_div_backup /
				   dev->ad9516_st.vco_divider;
	}
	/* The range of division for the LVPECL outputs is 1 to 32. */
	if((channel >= 0) && (channel <= 5))
		divider_max = 32;
	/* The LVDS/CMOS outputs allow a range of divisions up to a maximum
	 * of 1024. */
	if((channel >= 6) && (channel <= 9))
		divider_max = 1024;
	/* Increase vco_divider if the divider is greater than divider_max. */
	while(dev->ad9516_st.vco_divider < 6) {
		divider = freq_to_chan_div / frequency;
		if(divider > divider_max) {
			dev->ad9516_st.vco_divider++;
			freq_to_chan_div = freq_to_chan_div_backup /
					   dev->ad9516_st.vco_divider;
		} else {
			break;
		}
	}
	divider = freq_to_chan_div / frequency;
	/* If the divider is still greater than divider_max assign it the
	 * divider_max value.  */
	if(divider > divider_max) {
		divider = divider_max;
		set_freq = freq_to_chan_div / divider;
	} else {
		divider = 0;
	}
	/* Write the VCO divider value. */
	ad9516_read(dev, AD9516_REG_INPUT_CLKS, &reg_value);
	if((int32_t)reg_value < 0)
		return reg_value;
	if((dev->ad9516_st.vco_divider == 1) &&
	    ((reg_value & AD9516_SEL_VCO_CLK) == 0)) {
		reg_value |= AD9516_BYPASS_VCO_DIVIDER;
		ret = ad9516_write(dev, AD9516_REG_INPUT_CLKS, reg_value);
		if(ret < 0) {
			return ret;
		}
	} else {
		ret = ad9516_write(dev,
				   AD9516_REG_VCO_DIVIDER,
				   AD9516_VCO_DIVIDER((dev->
						       ad9516_st.vco_divider - 2)));
		if(ret < 0) {
			return ret;
		}
	}
	freq_to_chan_div_backup = freq_to_chan_div;
	if((channel >= 0) && (channel <= 7)) {
		if(divider == 0) {
			divider = 1;
			freq_0_divider = divider;
			freq_0_value = freq_to_chan_div_backup;
			while(freq_to_chan_div >= frequency) {
				freq_0_value = freq_to_chan_div;
				freq_0_divider = divider;
				if(divider < 32) {
					divider++;
					freq_to_chan_div =
						freq_to_chan_div_backup / divider;
				} else {
					divider++;
					freq_to_chan_div =
						freq_to_chan_div_backup / divider;
					while(dividers_checker(divider)) {
						divider++;
						freq_to_chan_div =
							freq_to_chan_div_backup /
							divider;
					}
				}
			}
			freq_1_value = freq_to_chan_div;
			freq_1_divider = divider;
			/* Choose the frequency closer to desired frequency */
			if((frequency - freq_1_value) >
			    (freq_0_value - frequency)) {
				divider = freq_0_divider;
				set_freq = freq_0_value;
			} else {
				divider = freq_1_divider;
				set_freq = freq_1_value;
			}
		}
		if((channel >= 0) && (channel <= 5)) {
			/* LVPECL Channels. */
			if(divider == 1) {
				if(channel / 2) {
					reg_address = AD9516_REG_DIVIDER_1_1;
				} else {
					reg_address = AD9516_REG_DIVIDER_0_1;
				}
				ad9516_read(dev, reg_address, &reg_value);
				if((int32_t)reg_value < 0) {
					return reg_value;
				}
				reg_value |= AD9516_DIVIDER_BYPASS;
			} else {
				if(channel / 2) {
					reg_address = AD9516_REG_DIVIDER_1_0;
				} else {
					reg_address = AD9516_REG_DIVIDER_0_0;

					ret = ad9516_read(dev, AD9516_REG_DIVIDER_0_1, &reg_value);
					if (ret < 0)
						return ret;

					reg_value &= ~AD9516_DIVIDER_BYPASS;
					ret = ad9516_write(dev, AD9516_REG_DIVIDER_0_1, reg_value);
					if (ret < 0)
						return ret;
				}
				/* The duty cycle closest to 50% is selected. */
				reg_value =
					AD9516_DIVIDER_LOW_CYCLES(((divider / 2) - 1)) |
					AD9516_DIVIDER_HIGH_CYCLES(((divider / 2) +
								    (divider % 2) - 1));
			}
			ret = ad9516_write(dev, reg_address, reg_value);
			if(ret < 0) {
				return ret;
			}
		} else {
			/* LVDS/CMOS Channels. */
			if(divider == 1) {
				/* Bypass the dividers. */
				if(channel / 6) {
					reg_address =
						AD9516_REG_LVDS_CMOS_DIVIDER_4_0;
				} else {
					reg_address =
						AD9516_REG_LVDS_CMOS_DIVIDER_3_0;
				}
				ad9516_read(dev, reg_address, &reg_value);
				if((int32_t)reg_value < 0) {
					return reg_value;
				}
				reg_value |= (AD9516_BYPASS_DIVIDER_2 |
					      AD9516_BYPASS_DIVIDER_1);
				ret = ad9516_write(dev, reg_address, reg_value);
				if(ret < 0) {
					return ret;
				}
			} else {
				if(divider <= 32) {
					/* Bypass the divider 2. */
					if(channel / 6) {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_4_1;
					} else {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_3_1;
					}
					ad9516_read(dev,
						    reg_address,
						    &reg_value);
					if((int32_t)reg_value < 0) {
						return reg_value;
					}
					reg_value |= AD9516_BYPASS_DIVIDER_2;
					ret = ad9516_write(dev,
							   reg_address,
							   reg_value);
					if(ret < 0) {
						return ret;
					}
					if(channel / 6) {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_4_0;
					} else {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_3_0;
					}
					/* The duty cycle closest to 50% is selected. */
					reg_value =
						AD9516_LOW_CYCLES_DIVIDER_1(((divider /
									      2) - 1)) |
						AD9516_HIGH_CYCLES_DIVIDER_1(((divider /
									       2) +
									      (divider %
									       2) - 1));
					ret = ad9516_write(dev,
							   reg_address,
							   reg_value);
					if(ret < 0) {
						return ret;
					}
				} else {
					/* Find a good value smaller or equal to 32 for divider_2. */
					divider_2 = 32;
					do {
						divider_1 = divider;
						if((divider_1 % divider_2)) {
							divider_2--;
						} else {
							divider_1 /= divider_2;
						}
					} while(divider_1 > 32);
					if(channel / 6) {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_4_0;
					} else {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_3_0;
					}
					/* The duty cycle closest to 50% is selected. */
					reg_value =
						AD9516_LOW_CYCLES_DIVIDER_1(((divider_1 /
									      2) - 1)) |
						AD9516_HIGH_CYCLES_DIVIDER_1(((divider_1 /
									       2) +
									      (divider_1 %
									       2) - 1));
					ret = ad9516_write(dev,
							   reg_address,
							   reg_value);
					if(ret < 0) {
						return ret;
					}
					if(channel / 6) {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_4_2;
					} else {
						reg_address =
							AD9516_REG_LVDS_CMOS_DIVIDER_3_2;
					}
					/* The duty cycle closest to 50% is selected. */
					reg_value =
						AD9516_LOW_CYCLES_DIVIDER_2(((divider_2 /
									      2) - 1)) |
						AD9516_HIGH_CYCLES_DIVIDER_2(((divider_2 /
									       2) +
									      (divider_2 %
									       2) - 1));
					ret = ad9516_write(dev,
							   reg_address,
							   reg_value);
					if(ret < 0) {
						return ret;
					}
				}
			}
		}
	} else {
		/* Invalid channel number. */
		return -1;
	}

	return set_freq;
}

/***************************************************************************//**
 * @brief Sets the phase on the specified channel.
 *
 * @param dev     - The device structure.
 * @param channel - The channel.
 * @param phase   - The desired phase value.
 *
 * @return Returns the phase or negative error code.
*******************************************************************************/
int32_t ad9516_phase(struct ad9516_dev *dev, int32_t channel, int32_t phase)
{
	uint32_t reg_value = 0;
	int32_t reg_address = 0;
	int32_t ret = 0;

	if((channel >= 0) && (channel <= 9)) {
		if((channel >= 0) && (channel <= 3)) {
			if(channel / 2) {
				reg_address = AD9516_REG_DIVIDER_1_1;
			} else {
				reg_address = AD9516_REG_DIVIDER_0_1;
			}

			ad9516_read(dev, reg_address, &reg_value);
			if((int32_t)reg_value < 0) {
				return reg_value;
			}
			reg_value &= ~AD9516_DIVIDER_PHASE_OFFSET(0xF);
			reg_value |= AD9516_DIVIDER_PHASE_OFFSET(phase);
		} else {
			if(channel / 6) {
				reg_address = AD9516_REG_LVDS_CMOS_DIVIDER_4_1;
			} else {
				reg_address = AD9516_REG_LVDS_CMOS_DIVIDER_3_1;
			}
			reg_value = AD9516_PHASE_OFFSET_DIVIDER_2(((phase / 2) +
					(phase % 2))) |
				    AD9516_PHASE_OFFSET_DIVIDER_1(phase / 2);
		}
		ret = ad9516_write(dev, reg_address, reg_value);
		if(ret < 0) {
			return ret;
		}
	}

	return phase;
}

/***************************************************************************//**
 * @brief Sets the power mode of the specified channel.
 *
 * @param dev     - The device structure.
 * @param channel - The channel.
 * @param mode    - Power mode.
 *
 * @return Returns the mode or negative error code.
*******************************************************************************/
int32_t ad9516_power_mode(struct ad9516_dev *dev, int32_t channel, int32_t mode)
{
	struct ad9516_lvpecl_channel_spec *lvpecl_channel;
	struct ad9516_lvds_cmos_channel_spec *lvds_cmos_channel;
	uint8_t reg_value = 0;
	int32_t reg_address = 0;
	uint32_t ret = 0;

	if((channel >= 0) && (channel <= 5)) {
		lvpecl_channel = &dev->ad9516_st.lvpecl_channels[channel];
		switch(channel) {
		case 0:
			reg_address = AD9516_REG_LVPECL_OUT0;
			break;
		case 1:
			reg_address = AD9516_REG_LVPECL_OUT1;
			break;
		case 2:
			reg_address = AD9516_REG_LVPECL_OUT2;
			break;
		default:
			reg_address = AD9516_REG_LVPECL_OUT3;
		}
		if((mode >= 0) && (mode <= 5)) {
			reg_value = lvpecl_channel->out_invert_en *
				    AD9516_OUT_LVPECL_INVERT |
				    AD9516_OUT_LVPECL_DIFF_VOLTAGE(lvpecl_channel->
						    out_diff_voltage) |
				    AD9516_OUT_LVPECL_POWER_DOWN(mode);
			ret = ad9516_write(dev, reg_address, reg_value);
			if((int32_t)ret < 0) {
				return ret;
			}
			return mode;
		} else {
			ad9516_read(dev, reg_address, &ret);
			if((int32_t)ret < 0) {
				return ret;
			}
			return (ret & AD9516_OUT_LVPECL_POWER_DOWN(0x3));
		}
	} else {
		if((channel >= 6) && (channel <= 10)) {
			lvds_cmos_channel = &dev->
					    ad9516_st.lvds_cmos_channels[0];
			switch(channel) {
			case 4:
				reg_address = AD9516_REG_LVDS_CMOS_OUT6;
				break;
			case 5:
				reg_address = AD9516_REG_LVDS_CMOS_OUT7;
				break;
			case 6:
				reg_address = AD9516_REG_LVDS_CMOS_OUT8;
				break;
			default:
				reg_address = AD9516_REG_LVDS_CMOS_OUT9;
			}
			if((mode >= 0) && (mode <= 1)) {
				reg_value =
					AD9516_OUT_LVDS_CMOS_INVERT(lvds_cmos_channel->
								    out_invert) |
					lvds_cmos_channel->cmos_b_en *
					AD9516_OUT_CMOS_B |
					lvds_cmos_channel->logic_level *
					AD9516_OUT_LVDS_CMOS |
					AD9516_OUT_LVDS_OUTPUT_CURRENT(lvds_cmos_channel
								       ->out_lvds_current) |
					mode * AD9516_OUT_LVDS_CMOS_POWER_DOWN;
				ret = ad9516_write(dev, reg_address, reg_value);
				if((int32_t)ret < 0) {
					return ret;
				}
				return mode;
			} else {
				ad9516_read(dev, reg_address, &ret);
				if((int32_t)ret < 0) {
					return ret;
				}
				return (ret & AD9516_OUT_LVDS_CMOS_POWER_DOWN);
			}
		} else {
			/* Invalid channel number. */
			return -1;
		}
	}
}


