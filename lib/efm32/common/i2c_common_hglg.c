/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/efm32/i2c.h>

/**
 * Set I2C Bus Frequency
 * @param[in] i2c I2C Bus (use I2C* ex. I2C0 , I2C1 ....)
 * @param[in] freqRef Peripheral CLK frequency.
 * @param[in] freqScl I2C CLK (SCL) frequency to be set.
 */
void i2c_bus_freq_set(uint32_t i2c, uint32_t freqRef, uint32_t freqScl){
	int32_t div;
	if (!freqScl || !freqRef){ return; }

	/* Set the CLHR (clock low to high ratio) to 4:4. */
	I2C_CTRL(i2c) &= ~I2C_CTRL_CLHR_MASK;

	/* SCL frequency is given by
	 * freqScl = freqRef/((Nlow + Nhigh) * (DIV + 1) + I2C_CR_MAX)z
	 * DIV = ((freqRef - (I2C_CR_MAX * freqScl))/((Nlow + Nhigh) * freqScl)) - 1
	 */

	div = ((freqRef - (I2C_CR_MAX * freqScl)) / (8 * freqScl)) - 1;
	I2C_CLKDIV(i2c) = (uint32_t)div;
}

void i2c_init(uint32_t i2c, bool isMaster){
	if (isMaster){
		I2C_CTRL(i2c) &= ~I2C_CTRL_SLAVE;
	}else{
		I2C_CTRL(i2c) |= I2C_CTRL_SLAVE;
	}
	i2c_enable(i2c, true);
}

void i2c_enable(uint32_t i2c, bool enable){
	if (enable){
		I2C_CTRL(i2c) |= I2C_CTRL_EN;
	}else {
		I2C_CTRL(i2c) &= ~I2C_CTRL_EN;
	}
}

void i2c_write(uint32_t i2c, uint8_t addr, uint8_t *data, uint16_t len){

	uint32_t flags;
	uint16_t rem = len;

	/* Check if in busy state. Since this SW assumes single master, we can */
	/* just issue an abort. The BUSY state is normal after a reset. */
	if (I2C_STATE(i2c) & I2C_STATE_BUSY){ I2C_CMD(i2c) = I2C_CMD_ABORT; }

	/* Ensure buffers are empty */
	I2C_CMD(i2c) = I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
	if (I2C_IF(i2c) & I2C_IF_RXDATAV) { I2C_RXDATA(i2c);}

	/* Clear all pending interrupts prior to starting transfer. */
	I2C_IFC(i2c) = I2C_IFC_MASK;

	/* Enable those interrupts we are interested in throughout transfer. */
	/* Notice that the I2C interrupt must also be enabled in the NVIC, but */
	/* that is left for an additional driver wrapper. */
	I2C_IEN(i2c) |= I2C_IF_NACK | I2C_IF_ACK | I2C_IF_MSTOP | I2C_IF_RXDATAV | I2C_IF_ERRORS;

	I2C_TXDATA(i2c)     = addr & 0xfe;
	I2C_CMD(i2c)        = I2C_CMD_START;

	flags = I2C_IF(i2c);
	while (! (flags | I2C_IF_ERRORS |  I2C_IF_NACK | I2C_IF_ACK)){
		flags = I2C_IF(i2c);
	}

	/* If some sort of fault, abort transfer. */
	if (flags & I2C_IF_ERRORS){
		I2C_CMD(i2c) = I2C_CMD_ABORT;
		return;
	}

	if (flags & I2C_IF_NACK) {
		I2C_CMD(i2c) = I2C_CMD_STOP;
		return;
	}

	I2C_IFC(i2c) = I2C_IFC_ACK;

	while (rem > 0){
		I2C_TXDATA(i2c) = data[len-rem] & 0xfe;
		flags = I2C_IF(i2c);
		while (! (flags |  I2C_IF_NACK | I2C_IF_ACK)){
			flags = I2C_IF(i2c);
		}
		if (flags & I2C_IF_NACK) {
			I2C_CMD(i2c) = I2C_CMD_STOP;
			return;
		}
		I2C_IFC(i2c) = I2C_IFC_ACK;
		rem--;
	}

	I2C_CMD(i2c) = I2C_CMD_STOP;
}

void i2c_read(uint32_t i2c, uint8_t addr, uint8_t *data, uint16_t len){

	if (len < 1) return;

	uint32_t flags;
	uint16_t rem = len;

	/* Check if in busy state. Since this SW assumes single master, we can */
	/* just issue an abort. The BUSY state is normal after a reset. */
	if (I2C_STATE(i2c) & I2C_STATE_BUSY){ I2C_CMD(i2c) = I2C_CMD_ABORT; }

	/* Ensure buffers are empty */
	I2C_CMD(i2c) = I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
	if (I2C_IF(i2c) & I2C_IF_RXDATAV) { I2C_RXDATA(i2c);}

	/* Clear all flags interrupts prior to starting transfer. */
	I2C_IFC(i2c) = I2C_IFC_MASK;

	/* Enable those interrupts we are interested in throughout transfer. */
	/* Notice that the I2C interrupt must also be enabled in the NVIC, but */
	/* that is left for an additional driver wrapper. */
	I2C_IEN(i2c) |= I2C_IF_NACK | I2C_IF_ACK | I2C_IF_MSTOP | I2C_IF_RXDATAV | I2C_IF_ERRORS;

	I2C_TXDATA(i2c)     = (addr & 0xfe) | 0x01; // Set last bit to 1 to denote read
	I2C_CMD(i2c)        = I2C_CMD_START;

	flags = I2C_IF(i2c);
	while (! (flags | I2C_IF_ERRORS |  I2C_IF_NACK | I2C_IF_ACK)){
		flags = I2C_IF(i2c);
	}

	/* If some sort of fault, abort transfer. */
	if (flags & I2C_IF_ERRORS){
		I2C_CMD(i2c) = I2C_CMD_ABORT;
		return;
	}

	if (flags & I2C_IF_NACK) {
		I2C_CMD(i2c) = I2C_CMD_STOP;
		return;
	}

	I2C_IFC(i2c) = I2C_IFC_ACK;

	while (rem > 0){
		flags = I2C_IF(i2c);
		while (! (flags | I2C_IF_ERRORS | I2C_IF_RXDATAV)){
			flags = I2C_IF(i2c);
		}

		if (flags & I2C_IF_ERRORS){
			I2C_CMD(i2c) = I2C_CMD_ABORT;
			return;
		}

		data[rem-len] = I2C_RXDATA(i2c);
		rem--;
		I2C_CMD(i2c) = I2C_CMD_ACK;
	}

	I2C_CMD(i2c) = I2C_CMD_STOP;
}
