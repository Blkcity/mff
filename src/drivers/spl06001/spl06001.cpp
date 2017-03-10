#include <fmx10_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>
#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <platforms/fmx10_getopt.h>

#define SPL_DIR_READ			0x80
#define SPL_DIR_WRITE			0x00

#define SPL06001_BARO_DEVICE_PATH_EXT	"/dev/spl06001_ext"
#define SPL06001_BARO_DEVICE_PATH_INT	"/dev/spl06001_int"

#define ADD_PSR_B2                              0x00
#define ADD_PSR_B1                              0x01
#define ADD_PSR_B0                              0x02

#define ADD_TMP_B2                              0x03
#define ADD_TMP_B1                              0x04
#define ADD_TMP_B0                              0x05

#define ADD_PRS_CFG                             0x06
#define ADD_TMP_CFG                             0x07
#define ADD_MEAS_CFG                            0x08
#define ADD_CFG_REG                             0x09
#define ADD_INT_STS                             0x0A
#define ADD_FIFO_STS                            0x0B
#define ADD_RESET                               0x0C
#define ADD_ID                                  0x0D
#define ADD_COEF                                0x10
#define ADD_COEF_SRCE                           0x28

#define CONTINUOUS_PRESSURE                     1
#define CONTINUOUS_TEMPERATURE                  2
#define CONTINUOUS_P_AND_T                      3
#define PRESSURE_SENSOR                         0
#define TEMPERATURE_SENSOR                      1

#define SPL06001_LOW_BUS_SPEED					1000*1000
#define SPL06001_HIGH_BUS_SPEED					11*1000*1000

#define SPL06001_CONVERSION_INTERVAL			15625//25000	/* microseconds */

#define SPL06001_TIMER_REDUCTION				200

enum SPL06001_DEVICE_TYPES {
	SPL060XX_DEVICE   = 0,
	SPL06001_DEVICE	= 6001,
	SPL06002_DEVICE	= 6002,
};

enum SPL06001_BUS {
	SPL06001_BUS_ALL = 0,
	SPL06001_BUS_I2C_INTERNAL,
	SPL06001_BUS_I2C_EXTERNAL,
	SPL06001_BUS_SPI_INTERNAL,
	SPL06001_BUS_SPI_EXTERNAL
};


#pragma pack(push, 1)
	/**
	 * Report conversation within the SPL06001, including command byte and
	 * interrupt status.
	 */
	struct spl0601_calib_param_t {	
		int16_t c0;
		int16_t c1;
		int32_t c00;
		int32_t c10;
		int16_t c01;
		int16_t c11;
		int16_t c20;
		int16_t c21;
		int16_t c30;		 
	};
	
	struct spl0601_t {	
		struct spl0601_calib_param_t calib_param;/**<calibration data*/ 
		uint8_t chip_id; /**<chip id*/	
		int32_t i32rawPressure;
		int32_t i32rawTemperature;
		int32_t i32kP;	
		int32_t i32kT;
	};

#pragma pack(pop)

class SPL06001 : public device::SPI
{
public:
	SPL06001(int bus, const char *path_baro, spi_dev_e device);
	virtual ~SPL06001();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();
protected:
	virtual int 	probe();
	
private:
	uint8_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned		_call_interval;
	orb_advert_t		_baro_topic;
	int			_baro_orb_class_instance;
	int			_baro_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	struct spl0601_t 	spl0601;
	ringbuffer::RingBuffer *_baro_reports;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	bool			is_external() { return (_baro_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	static void		measure_trampoline(void *arg);
	/**
	 * Read a register from the SPL06001
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = SPL06001_LOW_BUS_SPEED);

	/**
	 * Write a register in the SPL06001
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	void 			rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl);

	void 			start_continuous(uint8_t mode);

	void			start_temperature(void);

	void			start_pressure(void);

	void			get_calib_param(void);
	
	void			get_raw_temp(void);

	void			get_raw_pressure(void);

	float			get_temperature(void);

	float			get_pressure(void);
	
};

extern "C" { __EXPORT int spl06001_main(int argc, char *argv[]); }

SPL06001::SPL06001(int bus, const char *path_baro, spi_dev_e device):
	SPI("SPL06001", path_baro, bus, device, SPIDEV_MODE3, SPL06001_LOW_BUS_SPEED),
	_product(0),
	_call{},
	_call_interval(0),
	_baro_topic(nullptr),
	_baro_orb_class_instance(-1),
	_baro_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "SPL06001_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "SPL06001_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "SPL06001_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "SPL06001_buf_of")),
	_msl_pressure(101325),
	spl0601{},
	_baro_reports(nullptr)
{
	_debug_enabled = false;
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_SPL06001;
}

SPL06001::~SPL06001()
{
	stop();

	if(_baro_reports != nullptr)
		delete _baro_reports;

	if(_baro_class_instance != -1)
		unregister_class_devname(get_devname(), _baro_class_instance);

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	memset(&_call, 0, sizeof(_call));
}

int SPL06001::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	_baro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));

	if (_baro_reports == nullptr) {
		DEVICE_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	_baro_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	struct baro_report brp;

	_baro_reports->flush();

	measure();

	_baro_reports->get(&brp);
	
	brp.device_id = _device_id.devid;
	
	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_baro_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		warnx("failed to create sensor_baro publication");
	}


out:
	return ret;
}

int SPL06001::reset()
{
	spl0601.i32rawPressure = 0;
	spl0601.i32rawTemperature = 0;
	spl0601.chip_id = 0x34;
	get_calib_param();
	rateset(64,8);
	start_continuous(CONTINUOUS_P_AND_T);

	return OK;
}

int SPL06001::probe()
{

	/* look for a product ID we recognise */
	uint8_t who = read_reg(ADD_ID);

//        log("WHOAMI  0x%02x", who);
	if (who != 0x10) {return -EIO;}

	return (OK);
}


ssize_t SPL06001::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;
	
	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_baro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_baro_reports->empty()) {
		return -EAGAIN;
	}


	ret = 0;

	while (count--) {
		if (!_baro_reports->get(brp)) {
			break;
		}

		ret++;
		brp++;
	}

	/* return the number of bytes transferred */
	return (ret * sizeof(baro_report));
}

int SPL06001::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();
		
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_call_interval = SPL06001_CONVERSION_INTERVAL;//USEC2TICK(SPL06001_CONVERSION_INTERVAL);
					_call.period = _call_interval - SPL06001_TIMER_REDUCTION;
					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = 1000000 / arg;//USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SPL06001_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;
					_call.period = _call_interval - SPL06001_TIMER_REDUCTION;
					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000000 / _call_interval);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = fmx10_enter_critical_section();

			if (!_baro_reports->resize(arg)) {
				fmx10_leave_critical_section(flags);
				return -ENOMEM;
			}

			fmx10_leave_critical_section(flags);
			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _baro_reports->size();

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return SPI::ioctl(filp, cmd, arg);
}

uint8_t SPL06001::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | SPL_DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void SPL06001::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | SPL_DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	set_frequency(SPL06001_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));
}

void SPL06001::rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    spl0601.i32kP = i32kPkT;
    write_reg(ADD_PRS_CFG, reg);
    if(u8OverSmpl > 8)
    {
        reg = read_reg(ADD_CFG_REG);
        write_reg(ADD_CFG_REG, reg | 0x04);
    }

    spl0601.i32kT = i32kPkT;
    write_reg(ADD_TMP_CFG, reg|0x80);  //Using mems temperature
    if(u8OverSmpl > 8)
    {
        reg = read_reg(ADD_CFG_REG);
        write_reg(ADD_CFG_REG, reg | 0x08);
    }

}

void SPL06001::start_continuous(uint8_t mode)
{
    write_reg(ADD_MEAS_CFG, mode+4);
}

void SPL06001::start_temperature(void)
{
    write_reg(ADD_MEAS_CFG, 0x02);
}

void SPL06001::start_pressure(void)
{
    write_reg(ADD_MEAS_CFG, 0x01);
}

void SPL06001::get_calib_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    h =  read_reg(ADD_COEF);
    l  =  read_reg(ADD_COEF + 1);
    spl0601.calib_param.c0 = (int16_t)h<<4 | l>>4;
    spl0601.calib_param.c0 = (spl0601.calib_param.c0&0x0800)?(0xF000|spl0601.calib_param.c0):spl0601.calib_param.c0;
    h =  read_reg(ADD_COEF + 1);
    l  =  read_reg(ADD_COEF + 2);
    spl0601.calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    spl0601.calib_param.c1 = (spl0601.calib_param.c1&0x0800)?(0xF000|spl0601.calib_param.c1):spl0601.calib_param.c1;
    h =  read_reg(ADD_COEF + 3);
    m =  read_reg(ADD_COEF + 4);
    l =  read_reg(ADD_COEF + 5);
    spl0601.calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    spl0601.calib_param.c00 = (spl0601.calib_param.c00&0x080000)?(0xFFF00000|spl0601.calib_param.c00):spl0601.calib_param.c00;
    h =  read_reg(ADD_COEF + 5);
    m =  read_reg(ADD_COEF + 6);
    l =  read_reg(ADD_COEF + 7);
    spl0601.calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    spl0601.calib_param.c10 = (spl0601.calib_param.c10&0x080000)?(0xFFF00000|spl0601.calib_param.c10):spl0601.calib_param.c10;
    h =  read_reg(ADD_COEF + 8);
    l  =  read_reg(ADD_COEF + 9);
    spl0601.calib_param.c01 = (int16_t)h<<8 | l;
    h =  read_reg(ADD_COEF + 0x0A);
    l  =  read_reg(ADD_COEF + 0x0B);
    spl0601.calib_param.c11 = (int16_t)h<<8 | l;
    h =  read_reg(ADD_COEF + 0x0C);
    l  =  read_reg(ADD_COEF + 0x0D);
    spl0601.calib_param.c20 = (int16_t)h<<8 | l;
    h =  read_reg(ADD_COEF + 0x0E);
    l  =  read_reg(ADD_COEF + 0x0F);
    spl0601.calib_param.c21 = (int16_t)h<<8 | l;
    h =  read_reg(ADD_COEF + 0x10);
    l  =  read_reg(ADD_COEF + 0x11);
    spl0601.calib_param.c30 = (int16_t)h<<8 | l;
}

void SPL06001::get_raw_temp(void)
{
    uint8_t h,m,l;
    h = read_reg(ADD_TMP_B2);
    m = read_reg(ADD_TMP_B1);
    l = read_reg(ADD_TMP_B0);
    spl0601.i32rawTemperature = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
    spl0601.i32rawTemperature= (spl0601.i32rawTemperature&0x800000) ? (0xFF000000|spl0601.i32rawTemperature) : spl0601.i32rawTemperature;
}

void SPL06001::get_raw_pressure(void)
{
    uint8_t h,m,l;
    h = read_reg(ADD_PSR_B2);
    m = read_reg(ADD_PSR_B1);
    l = read_reg(ADD_PSR_B0);
    
    spl0601.i32rawPressure = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
    spl0601.i32rawPressure= (spl0601.i32rawPressure&0x800000) ? (0xFF000000|spl0601.i32rawPressure) : spl0601.i32rawPressure;
}

float SPL06001::get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
    fTCompensate =  spl0601.calib_param.c0 * 0.5f + spl0601.calib_param.c1 * fTsc;
    return fTCompensate;
}

float SPL06001::get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
    fPsc = spl0601.i32rawPressure / (float)spl0601.i32kP;
    qua2 = spl0601.calib_param.c10 + fPsc * (spl0601.calib_param.c20 + fPsc* spl0601.calib_param.c30);
    qua3 = fTsc * fPsc * (spl0601.calib_param.c11 + fPsc * spl0601.calib_param.c21);

    fPCompensate = spl0601.calib_param.c00 + fPsc * qua2 + fTsc * spl0601.calib_param.c01 + qua3;
    return fPCompensate;
}

void SPL06001::start(void)
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_baro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       15625,
		       _call_interval - SPL06001_TIMER_REDUCTION,
		       (hrt_callout)&SPL06001::measure_trampoline, this);
}

void SPL06001::stop(void)
{
	hrt_cancel(&_call);

	/* discard unread data in the buffers */
	_baro_reports->flush();
}

void SPL06001::measure_trampoline(void *arg)
{
	SPL06001 *dev = reinterpret_cast<SPL06001 *>(arg);

	/* make another measurement */
	dev->measure();
}

void SPL06001::measure(void)
{
	uint8_t report[8];
	struct baro_report		brb;
	
	perf_begin(_sample_perf);

	report[0] = SPL_DIR_READ;
	set_frequency(SPL06001_HIGH_BUS_SPEED);

	if (OK != transfer(report, report, 7)) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	brb.timestamp = hrt_absolute_time();
	brb.error_count = perf_event_count(_comms_errors);
	brb.device_id = _device_id.devid;

	spl0601.i32rawPressure = (int32_t)report[1]<<16 | (int32_t)report[2]<<8 | (int32_t)report[3];
	spl0601.i32rawPressure = (spl0601.i32rawPressure&0x800000) ? (0xFF000000|spl0601.i32rawPressure) : spl0601.i32rawPressure;

	spl0601.i32rawTemperature = (int32_t)report[4]<<16 | (int32_t)report[5]<<8 | (int32_t)report[6];
	spl0601.i32rawTemperature = (spl0601.i32rawTemperature&0x800000) ? (0xFF000000|spl0601.i32rawTemperature) : spl0601.i32rawTemperature;

    float fTsc, fPsc;
    float qua2, qua3;

    fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
	brb.temperature = spl0601.calib_param.c0 * 0.5f + spl0601.calib_param.c1 * fTsc;

	fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
	fPsc = spl0601.i32rawPressure / (float)spl0601.i32kP;
	qua2 = spl0601.calib_param.c10 + fPsc * (spl0601.calib_param.c20 + fPsc* spl0601.calib_param.c30);
	qua3 = fTsc * fPsc * (spl0601.calib_param.c11 + fPsc * spl0601.calib_param.c21);

	brb.pressure = spl0601.calib_param.c00 + fPsc * qua2 + fTsc * spl0601.calib_param.c01 + qua3;

	fPsc = brb.pressure / 101325;
	qua2 = pow(fPsc,0.19026);
		
	brb.altitude = (1- qua2) * 44330.0f;

	if (!(_pub_blocked) && _baro_topic != nullptr) {
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &brb);
	}

	if (_baro_reports->force(&brb)) {
		perf_count(_buffer_overflows);
	}
	
	perf_end(_sample_perf);
}

void SPL06001::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	_baro_reports->print_info("report queue");
	printf("device:         %s\n", "spl06-001");
//	printf("TEMP:           %d\n", _TEMP);
//	printf("SENS:           %lld\n", _SENS);
//	printf("OFF:            %lld\n", _OFF);
//	printf("P:              %.3f\n", (double)_P);
//	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
}

namespace spl06001
{
	SPL06001 *g_dev_int; // on internal bus
	SPL06001 *g_dev_ext; // on external bus
	
	void	start(bool);
	void	stop(bool);
	void	test(bool);
	void	reset(bool);
	void	info(bool);
	void	usage();

void start(bool external_bus)
{
	int fd;
	SPL06001 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
	const char *path_baro  = external_bus ? SPL06001_BARO_DEVICE_PATH_EXT : SPL06001_BARO_DEVICE_PATH_INT;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(FMX10_SPI_BUS_EXT) && defined(FMX10_SPIDEV_EXT_SPL)
		*g_dev_ptr = new SPL06001(FMX10_SPI_BUS_EXT, path_baro, (spi_dev_e)FMX10_SPIDEV_EXT_SPL);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		*g_dev_ptr = new SPL06001(FMX10_SPI_BUS_SPL, path_baro, (spi_dev_e)FMX10_SPIDEV_SPL);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path_baro, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

void stop(bool external_bus)
{
	SPL06001 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

void test(bool external_bus)
{
	const char *path_baro  = external_bus ? SPL06001_BARO_DEVICE_PATH_EXT : SPL06001_BARO_DEVICE_PATH_INT;
	baro_report b_report;
	ssize_t sz;

	/* get the driver */
	int fd_baro = open(path_baro, O_RDONLY);

	if (fd_baro < 0)
		err(1, "%s open failed (try 'spl06001 start')",path_baro);

	sz = read(fd_baro, &b_report, sizeof(b_report));
	if (sz != sizeof(b_report)) {
	    warnx("ret: %d, expected: %d", sz, sizeof(b_report));
	    err(1, "immediate baro read failed");
	}

	warnx("single read");
	warnx("pressure:    %10.4f", (double)b_report.pressure);
	warnx("altitude:    %11.4f", (double)b_report.altitude);
	warnx("temperature: %8.4f", (double)b_report.temperature);
	warnx("time:        %d", (int)b_report.timestamp);

	/* reset to default polling */
	if (ioctl(fd_baro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}
	/* XXX add poll-rate tests here too */

//	reset(external_bus);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void reset(bool external_bus)
{
	const char *path_accel = external_bus ? SPL06001_BARO_DEVICE_PATH_EXT : SPL06001_BARO_DEVICE_PATH_INT;
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void info(bool external_bus)
{
	SPL06001 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

void usage(void)
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I 	 (intternal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s 	 (intternal SPI bus)");
}

}

int spl06001_main(int argc, char *argv[])
{
	enum SPL06001_BUS busid = SPL06001_BUS_ALL;

//	enum SPL06001_DEVICE_TYPES device_type = SPL06001_DEVICE;
	int ch;
	int myoptiond = 1;
	const char *myoptarg = NULL;

	while((ch = fmx10_getopt(argc,argv, "XI", &myoptiond,&myoptarg)) != EOF)
	{
		switch (ch)
		{
			case 'X':
				busid = SPL06001_BUS_I2C_EXTERNAL;
				break;
			case 'I':
				busid = SPL06001_BUS_I2C_INTERNAL;
				break;
			case 'S':
				busid = SPL06001_BUS_SPI_EXTERNAL;
				break;
			case 's':
				busid = SPL06001_BUS_SPI_INTERNAL;
				break;
			default:
				spl06001::usage();
				exit(0);
		}
	}

	const char *verb = argv[myoptiond];

	busid = SPL06001_BUS_ALL;
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		spl06001::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		spl06001::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		spl06001::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		spl06001::info(busid);
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");	
}


