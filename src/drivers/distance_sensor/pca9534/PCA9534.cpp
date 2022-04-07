/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "PCA9534.hpp"

/* PCA9534 Registers addresses */
#define PCA9534_IN_REG 0x00
#define PCA9534_OUT_REG 0x01
#define PCA9534_POLARITY_INVERSION_REG 0x02
#define PCA9534_CONFIGURATION_REG 0x03


#define PCA9534_US                                      1000    // 1ms
#define PCA9534_SEND_RATE                             50000   // 50ms

#define PCA9534_BUS_CLOCK                               400000 // 400kHz bus clock speed

PCA9534::PCA9534(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	// VL53L0X typical range 0-2 meters with 25 degree field of view
	// _px4_rangefinder.set_min_distance(0.f);
	// _px4_rangefinder.set_max_distance(2.f);
	// _px4_rangefinder.set_fov(math::radians(25.f));

	// Allow retries as the device typically misses the first measure attempts.
	I2C::_retries = 1;

	// _px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L0X);
}

PCA9534::~PCA9534()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int PCA9534::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ScheduleNow();
	return PX4_OK;
}

int PCA9534::collect()
{
	// Read from the sensor.
	uint8_t val[2] {};
	perf_begin(_sample_perf);

	_collect_phase = false;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer(nullptr, 0, &val[0], 2) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

	uint16_t distance_mm = (val[0] << 8) | val[1];
	float distance_m = distance_mm / 1000.f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	return PX4_OK;
}

int PCA9534::measure()
{
	uint8_t wait_for_measurement = 0;
	uint8_t system_start = 0;

	// Send the command to begin a measurement.

	if (_new_measurement) {
        	_new_measurement = false;
		writeRegister(PCA9534_OUT_REG, 0x01);
		if ((system_start & 0x01) == 1) {
			ScheduleDelayed(PCA9534_US);
			return PX4_OK;
		} else {
			_measurement_started = true;
		}
	}

	if (!_collect_phase && !_measurement_started) {
		if ((system_start & 0x01) == 1) {
			ScheduleDelayed(PCA9534_US);
			return PX4_OK;
		} else {
			_measurement_started = true;
		}
	}

	if ((wait_for_measurement & 0x07) == 0) {
		ScheduleDelayed(PCA9534_US); // reschedule every 1 ms until measurement is ready
		return PX4_OK;
	}

	_collect_phase = true;
	return PX4_OK;
}

void PCA9534::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}


int PCA9534::readRegister(const uint8_t reg_address, uint8_t &value)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, sizeof(reg_address), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int PCA9534::readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value[0], length);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

void PCA9534::RunImpl()
{
	measure();

	if (_collect_phase) {
		_collect_phase = false;
		_new_measurement = true;
		ScheduleDelayed(PCA9534_SEND_RATE);
	}
}


int PCA9534::writeRegister(const uint8_t reg_address, const uint8_t value)
{
	uint8_t cmd[2] {reg_address, value};
	int ret = transfer(&cmd[0], 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int PCA9534::writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length)
{
	if (length > 6 || length < 1) {
		DEVICE_LOG("VL53L0X::writeRegisterMulti length out of range");
		return PX4_ERROR;
	}

	/* be careful: for uint16_t to send first higher byte */
	uint8_t cmd[7] {};
	cmd[0] = reg_address;

	memcpy(&cmd[1], &value[0], length);

	int ret = transfer(&cmd[0], length + 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

void PCA9534::print_usage()
{
	PRINT_MODULE_USAGE_NAME("PCA9534", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x20);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int pca9534_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = PCA9534;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	//cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.i2c_address = PCA9534_BASEADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_PCA9534);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
