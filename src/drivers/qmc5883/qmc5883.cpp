/****************************************************************************
 *
 *   Copyright (c) 2012-2015 FMX10 Development Team. All rights reserved.
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
 * 3. Neither the name FMX10 nor the names of its contributors may be
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
 * @file qmc5883.cpp
 *
 * Driver for the QMC5883 / HMC5983 magnetometer connected via I2C or SPI.
 */

#include <fmx10_config.h>
#include <fmx10_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include "qmc5883.h"

/*
 * QMC5883 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define QMC5883_CONVERSION_INTERVAL	(1000000 / 200)	/* microseconds */

#define ADDR_DATA_OUT_X_LSB		0x00
#define ADDR_DATA_OUT_X_MSB		0x01
#define ADDR_DATA_OUT_Z_LSB		0x02
#define ADDR_DATA_OUT_Z_MSB		0x03
#define ADDR_DATA_OUT_Y_LSB		0x04
#define ADDR_DATA_OUT_Y_MSB		0x05
#define ADDR_STATUS			0x06

/* temperature on hmc5983 only */
#define ADDR_TEMP_OUT_LSB		0x07
#define ADDR_TEMP_OUT_MSB		0x08

/* Control Register */
#define ADDR_CON_REG1			0x09
#define ADDR_CON_REG2			0x0A

/* Period Register */
#define ADDR_PER_REG			0x0B

/* modes not changeable outside of driver */
#define QMC5883_MODE_STANDBY		(0 << 0)
#define QMC5883_MODE_CONTINUOUS		(1 << 0)
#define QMC5883_ODR_10HZ		(0 << 2)
#define QMC5883_ODR_50HZ		(1 << 2)
#define QMC5883_ODR_100HZ		(2 << 2)
#define QMC5883_ODR_200HZ		(3 << 2)
#define QMC5883_RNG_2G			(0 << 4)
#define QMC5883_RNG_8G			(1 << 4)
#define QMC5883_OSR_512			(0 << 6)
#define QMC5883_OSR_256			(1 << 6)
#define QMC5883_OSR_128			(2 << 6)
#define QMC5883_OSR_64			(3 << 6)

#define QMC5883_STATUS_READY		(1 << 0)
#define QMC5883_STATUS_OVERFLOW		(1 << 1)
#define QMC5883_STATUS_SKIPREAD		(1 << 2)

#define QMC5883_INT_ENB			(0 << 0)
#define QMC5883_INT_DIS			(1 << 0)
#define QMC5883_ROL_PNT_NOR		(0 << 6)
#define QMC5883_ROL_PNT_ENB		(1 << 6)
#define QMC5883_SOFT_RST_NOR		(0 << 7)
#define QMC5883_SOFT_RST_ENB		(1 << 7)

enum QMC5883_BUS {
	QMC5883_BUS_ALL = 0,
	QMC5883_BUS_I2C_INTERNAL,
	QMC5883_BUS_I2C_EXTERNAL,
	QMC5883_BUS_SPI
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class QMC5883 : public device::CDev
{
public:
	QMC5883(device::Device *interface, const char *path, enum Rotation rotation);
	virtual ~QMC5883();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

private:
	work_s			_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	struct mag_calibration_s	_scale;
	float 			_range_scale;
	float 			_range_ga;
	bool			_collect_phase;
	int			_class_instance;
	int			_orb_class_instance;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */
	bool			_calibrated;		/**< the calibration is valid */

	enum Rotation		_rotation;

	struct mag_report	_last_report;           /**< used for info() */

	uint8_t			_range_bits;
	uint8_t			_conf_reg;
	uint8_t			_temperature_counter;
	uint8_t			_temperature_error_count;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned enable);

	/**
	 * enable hmc5983 temperature compensation
	 */
	int			set_temperature_compensation(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor range.
	 *
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void 			check_range(void);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int 			check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int 			check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int 			check_offset();

	/* this class has pointer data members, do not allow copying it */
	QMC5883(const QMC5883 &);
	QMC5883 operator=(const QMC5883 &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int qmc5883_main(int argc, char *argv[]);


QMC5883::QMC5883(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("QMC5883", path),
	_interface(interface),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_scale{},
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(1.9f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "qmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "qmc5883_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "qmc5883_buf_of")),
	_range_errors(perf_alloc(PC_COUNT, "qmc5883_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "qmc5883_conf_err")),
	_sensor_ok(false),
	_calibrated(false),
	_rotation(rotation),
	_last_report{0},
	_range_bits(0),
	_conf_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5883;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

QMC5883::~QMC5883()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
QMC5883::init()
{
	int ret = FMX10_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int QMC5883::set_range(unsigned range)
{
	if (range < 1) {
		_range_bits = 0x0D;
		_range_scale = 1.0f;
		_range_ga = 2.0f;

	} else if (range <= 1) {
		_range_bits = 0x0D;
		_range_scale = 12000.0f;
		_range_ga = 2.0f;

	} else {
		_range_bits = 0x1D;
		_range_scale = 3000.0f;
		_range_ga = 8.0f;

	} 

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CON_REG1, _range_bits);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CON_REG1, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == _range_bits);
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void QMC5883::check_range(void)
{
	int ret;

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CON_REG1, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (range_bits_in != _range_bits) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CON_REG1, _range_bits);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void QMC5883::check_conf(void)
{
    return;
}

ssize_t
QMC5883::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(QMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
QMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(QMC5883_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(QMC5883_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = fmx10_enter_critical_section();

			if (!_reports->resize(arg)) {
				fmx10_leave_critical_section(flags);
				return -ENOMEM;
			}

			fmx10_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	case MAGIOCSTEMPCOMP:
		return set_temperature_compensation(arg);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
QMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&QMC5883::cycle_trampoline, this, 1);
}

void
QMC5883::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}

}

int
QMC5883::reset()
{
	/* set range, ceil floating point number */
	return set_range(1);
}

void
QMC5883::cycle_trampoline(void *arg)
{
	QMC5883 *dev = (QMC5883 *)arg;
	dev->cycle();
}

void
QMC5883::cycle()
{
	if (_measure_ticks == 0) {
		return;
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(QMC5883_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&QMC5883::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(QMC5883_CONVERSION_INTERVAL));

			return;
		}
	}
#if 0
	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}
#endif
	/* next phase is collection */
	_collect_phase = true;

	if (_measure_ticks > 0) {
	    /* schedule a fresh cycle call when the measurement is done */
	    work_queue(HPWORK,
		       &_work,
		       (worker_t)&QMC5883::cycle_trampoline,
		       this,
		       USEC2TICK(QMC5883_CONVERSION_INTERVAL));
	}
}

int
QMC5883::measure()
{
    return set_range(1);
}

int
QMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)
	struct {
		int16_t		x, y, z;
	} report;

	int	ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.range_ga = _range_ga;
	new_report.scaling = _range_scale;
	new_report.device_id = _device_id.devid;

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = _interface->read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[1]) << 8) + hmc_report.x[0];
	report.y = (((int16_t)hmc_report.y[1]) << 8) + hmc_report.y[0];
	report.z = (((int16_t)hmc_report.z[1]) << 8) + hmc_report.z[0];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 32767) ||
	    (abs(report.y) > 32767) ||
	    (abs(report.z) > 32767)) {
		perf_count(_comms_errors);
		goto out;
	}

	/* get measurements from the device */
	new_report.temperature = 0;

	if (_conf_reg & 0x01) {
		/*
		  if temperature compensation is enabled read the
		  temperature too.

		  We read the temperature every 10 samples to avoid
		  excessive I2C traffic
		 */
		if (_temperature_counter++ == 10) {
			uint8_t raw_temperature[2];

			_temperature_counter = 0;

			ret = _interface->read(ADDR_TEMP_OUT_LSB,
					       raw_temperature, sizeof(raw_temperature));

			if (ret == OK) {
				int16_t temp16 = (((int16_t)raw_temperature[1]) << 8) +
						 raw_temperature[0];
				new_report.temperature = 25 + (temp16 / (16 * 8.0f));
				_temperature_error_count = 0;

			} else {
				_temperature_error_count++;

				if (_temperature_error_count == 10) {
					/*
					  it probably really is an old QMC5883,
					  and can't do temperature. Disable it
					*/
					_temperature_error_count = 0;
					DEVICE_DEBUG("disabling temperature compensation");
					set_temperature_compensation(0);
				}
			}

		} else {
			new_report.temperature = _last_report.temperature;
		}
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = report.y;
	new_report.y_raw = -report.x;
	/* z remains z */
	new_report.z_raw = report.z;

	/* scale values for output */
#if 0
	// XXX revisit for SPI part, might require a bus type IOCTL
	unsigned dummy;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);

	if (sensor_is_onboard) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
	}
#endif
	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	xraw_f = -report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_report.x = ((xraw_f / _range_scale) - _scale.x_offset) * _scale.x_scale;
	//new_report.x = xraw_f / _range_scale;
	/* flip axes and negate value for y */
	new_report.y = ((yraw_f / _range_scale) - _scale.y_offset) * _scale.y_scale;
	//new_report.y = yraw_f / _range_scale;
	/* z remains z */
	new_report.z = ((zraw_f / _range_scale) - _scale.z_offset) * _scale.z_scale;
	//new_report.z = zraw_f / _range_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	if (_reports->force(&new_report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 0) {
		check_range();
	}

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int QMC5883::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	uint8_t good_count = 0;

	// XXX do something smarter here
	int fd = (int)enable;

	struct mag_calibration_s mscale_previous;
	mscale_previous.x_offset = 0.0f;
	mscale_previous.x_scale = 1.0f;
	mscale_previous.y_offset = 0.0f;
	mscale_previous.y_scale = 1.0f;
	mscale_previous.z_offset = 0.0f;
	mscale_previous.z_scale = 1.0f;

	struct mag_calibration_s mscale_null;
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

        if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
                warn("FAILED: MAGIOCGSCALE 1");
                ret = 1;
                goto out;
        }

        if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
                warn("FAILED: MAGIOCSSCALE 1");
                ret = 1;
                goto out;
        }

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	float cal[3];
	cal[0] = cal[1] = cal[2] = 0;
	/* read the sensor up to 150x, stopping when we have 50 good values */
	for (uint8_t i = 0; i < 150; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		cal[0] += report.x;
		cal[1] += report.y;
		cal[2] += report.z;
		good_count++;
	}

	/* set scaling in device */
	mscale_previous.x_offset = cal[0] / good_count;
	mscale_previous.y_offset = cal[1] / good_count;
	mscale_previous.z_offset = cal[2] / good_count;

	ret = OK;
out:
	if (OK != ret)
	    return ret;
	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
		warn("FAILED: MAGIOCSSCALE 2");
	}

	if (ret == OK) {
		if (check_scale()) {
			/* failed */
			warnx("FAILED: SCALE");
			ret = FMX10_ERROR;
		}

	}
	return ret;
}

int QMC5883::check_scale()
{
	return OK;
}

int QMC5883::check_offset()
{
	return OK;
}

int QMC5883::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
		      (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int QMC5883::set_excitement(unsigned enable)
{
    _conf_reg |= (enable ? 0x10 : 0);
    return OK;
}


/*
  enable/disable temperature compensation on the HMC5983

  Unfortunately we don't yet know of a way to auto-detect the
  difference between the QMC5883 and HMC5983. Both of them do
  temperature sensing, but only the 5983 does temperature
  compensation. We have noy yet found a behaviour that can be reliably
  distinguished by reading registers to know which type a particular
  sensor is

  update: Current best guess is that many sensors marked QMC5883L on
  the package are actually 5983 but without temperature compensation
  tables. Reading the temperature works, but the mag field is not
  automatically adjusted for temperature. We suspect that there may be
  some early 5883L parts that don't have the temperature sensor at
  all, although we haven't found one yet. The code that reads the
  temperature looks for 10 failed transfers in a row and disables the
  temperature sensor if that happens. It is hoped that this copes with
  the genuine 5883L parts.
 */
int QMC5883::set_temperature_compensation(unsigned enable)
{
    _conf_reg |= (enable ? 1 : 0);
    return OK;
}

int
QMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
QMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
QMC5883::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
QMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f range_ga %.2f\n",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale), (double)_range_ga);
	printf("temperature %.2f\n", (double)_last_report.temperature);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace qmc5883
{

/*
  list of supported bus configurations
 */
struct qmc5883_bus_option {
	enum QMC5883_BUS busid;
	const char *devpath;
	QMC5883_constructor interface_constructor;
	uint8_t busnum;
	QMC5883	*dev;
} bus_options[] = {
	{ QMC5883_BUS_I2C_EXTERNAL, "/dev/qmc5883_ext", &QMC5883_I2C_interface, FMX10_I2C_BUS_EXPANSION, NULL },
#ifdef FMX10_I2C_BUS_HMC5883
	{ QMC5883_BUS_I2C_INTERNAL, "/dev/qmc5883_int", &QMC5883_I2C_interface, FMX10_I2C_BUS_HMC5883, NULL },
#endif
#ifdef FMX10_SPIDEV_HMC
	{ QMC5883_BUS_SPI, "/dev/qmc5883_spi", &QMC5883_SPI_interface, FMX10_SPI_BUS_MPU, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum QMC5883_BUS busid, enum Rotation rotation);
int	stop();
bool	start_bus(struct qmc5883_bus_option &bus, enum Rotation rotation);
struct qmc5883_bus_option &find_bus(enum QMC5883_BUS busid);
void	test(enum QMC5883_BUS busid);
void	reset(enum QMC5883_BUS busid);
int	info(enum QMC5883_BUS busid);
int	calibrate(enum QMC5883_BUS busid);
int	temp_enable(QMC5883_BUS busid, bool enable);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct qmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new QMC5883(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

	close(fd);

	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum QMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == QMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != QMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		exit(1);
	}
}

int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct qmc5883_bus_option &find_bus(enum QMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == QMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'qmc5883 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	warnx("device active: %s", ret ? "external" : "onboard");

	/* set the queue depth to 5 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}


/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the QMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int calibrate(enum QMC5883_BUS busid)
{
	int ret;
	struct qmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'qmc5883 start' if the driver is not running", path);
	}

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}


/**
 * enable/disable temperature compensation
 */
int
temp_enable(enum QMC5883_BUS busid, bool enable)
{
	struct qmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, MAGIOCSTEMPCOMP, (unsigned)enable) < 0) {
		err(1, "set temperature compensation failed");
	}

	close(fd);
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);

	warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
	warnx("    -X only external bus");
#if (FMX10_I2C_BUS_HMC5883 || FMX10_SPIDEV_HMC)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
qmc5883_main(int argc, char *argv[])
{
	int ch;
	enum QMC5883_BUS busid = QMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	bool temp_compensation = false;

	if (argc < 2) {
		qmc5883::usage();
		exit(0);
	}

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (FMX10_I2C_BUS_HMC5883 || FMX10_SPIDEV_HMC)

		case 'I':
			busid = QMC5883_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = QMC5883_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = QMC5883_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		case 'T':
			temp_compensation = true;
			break;

		default:
			qmc5883::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		qmc5883::start(busid, rotation);

		if (calibrate && qmc5883::calibrate(busid) != 0) {
			errx(1, "calibration failed");
		}

		if (temp_compensation) {
			// we consider failing to setup temperature
			// compensation as non-fatal
			qmc5883::temp_enable(busid, true);
		}

		exit(0);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		return qmc5883::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		qmc5883::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		qmc5883::reset(busid);
	}

	/*
	 * enable/disable temperature compensation
	 */
	if (!strcmp(verb, "tempoff")) {
		qmc5883::temp_enable(busid, false);
	}

	if (!strcmp(verb, "tempon")) {
		qmc5883::temp_enable(busid, true);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		qmc5883::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (qmc5883::calibrate(busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
}
