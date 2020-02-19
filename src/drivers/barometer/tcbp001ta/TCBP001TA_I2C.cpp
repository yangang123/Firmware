/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tcbp001ta_spi.cpp
 *
 * SPI interface for TCBP001TA
 */

#include "tcbp001ta.h"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>

#if defined(PX4_I2C_OBDEV_TCBP001TA) || defined(PX4_I2C_EXT_OBDEV_TCBP001TA)

class TCBP001TA_I2C: public device::I2C, public tabp001ta::ITCBP001TA
{
public:
	TCBP001TA_I2C(uint8_t bus, uint32_t device);
	virtual ~TCBP001TA_I2C() override = default;

	int init() override { return I2C::init(); }

	uint8_t	get_reg(uint8_t addr) override;
	int	set_reg(uint8_t value, uint8_t addr) override;

	tabp001ta::data_s		*get_data(uint8_t addr) override;
	tabp001ta::calibration_s	*get_calibration(uint8_t addr) override;

	uint32_t get_device_id() const override { return device::I2C::get_device_id(); }

private:
	tabp001ta::calibration_s	_cal{};
	tabp001ta::data_s		_data{};
};

tabp001ta::ITCBP001TA *tabp001ta_i2c_interface(uint8_t busnum, uint32_t device)
{
	return new TCBP001TA_I2C(busnum, device);
}

TCBP001TA_I2C::TCBP001TA_I2C(uint8_t bus, uint32_t device) :
	I2C("TCBP001TA_I2C", nullptr, bus, device, 100 * 1000)
{
}

uint8_t
TCBP001TA_I2C::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);

	return cmd[1];
}

int
TCBP001TA_I2C::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

tabp001ta::data_s *
TCBP001TA_I2C::get_data(uint8_t addr)
{
	const uint8_t cmd = addr;

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_data, sizeof(tabp001ta::data_s)) == OK) {
		return (&_data);

	} else {
		return nullptr;
	}
}

tabp001ta::calibration_s *
TCBP001TA_I2C::get_calibration(uint8_t addr)
{
	const uint8_t cmd = addr;

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_cal, sizeof(tabp001ta::calibration_s)) == OK) {
		return &(_cal);

	} else {
		return nullptr;
	}
}

#endif /* PX4_I2C_OBDEV_TCBP001TA || PX4_I2C_EXT_OBDEV_TCBP001TA */
