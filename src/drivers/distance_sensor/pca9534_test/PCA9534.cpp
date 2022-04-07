/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file airspeed.cpp
 * @author Simon Wilks <simon@px4.io>
 * @author Lorenz Meier <lorenz@px4.io>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/device.h>

#include <drivers/device/i2c.h>

#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>


#include <drivers/distance_sensor/pca9534/PCA9534.h>

PCA9534::PCA9534(int bus, int bus_frequency, int address, unsigned conversion_interval) :
	I2C(0, "PCA9534", bus, address, bus_frequency),
	_sensor_ok(false),
	_measure_interval(conversion_interval),
	_collect_phase(false),
	_diff_pres_offset(0.0f),
	_airspeed_orb_class_instance(-1),
	_class_instance(-1),
	_conversion_interval(conversion_interval),
	_sample_perf(perf_alloc(PC_ELAPSED, "aspd_read")),
	_comms_errors(perf_alloc(PC_COUNT, "aspd_com_err"))
{
}

PCA9534::~PCA9534()
{
	if (_class_instance != -1) {
		unregister_class_devname(AIRSPEED_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PCA9534::init()
{
	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(AIRSPEED_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	measure();

	return PX4_OK;
}

int
PCA9534::probe()
{
	/* on initial power up the device may need more than one retry
	   for detection. Once it is running the number of retries can
	   be reduced
	*/
	_retries = 1;
	int ret = measure();

	return ret;
}

int
PCA9534::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case AIRSPEEDIOCSSCALE: {
			struct airspeed_scale *s = (struct airspeed_scale *)arg;
			_diff_pres_offset = s->offset_pa;
			return OK;
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

extern "C" __EXPORT int pac9534_main(int argc, char *argv[])
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
		//ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_VL53L0X);

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
