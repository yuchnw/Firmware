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

#include "ADIS16497.hpp"
#include "ADIS16497_gyro.hpp"

#include <px4_config.h>
#include <ecl/geo/geo.h>

#define DIR_READ				0x00
#define DIR_WRITE				0x80

//  ADIS16497 registers

static constexpr uint8_t SYS_E_FLAG = 0x08; // Output, system error flags
static constexpr uint8_t DIAG_STS   = 0x0A; // Output, self test error flags

static constexpr uint8_t TEMP_OUT   = 0x0E; // Output, temperature

static constexpr uint8_t X_GYRO_LOW = 0x10; // Output, x-axis gyroscope, low word
static constexpr uint8_t X_GYRO_OUT = 0x12; // Output, x-axis gyroscope, high word
static constexpr uint8_t Y_GYRO_LOW = 0x14; // Output, y-axis gyroscope, low word
static constexpr uint8_t Y_GYRO_OUT = 0x16; // Output, y-axis gyroscope, high word
static constexpr uint8_t Z_GYRO_LOW = 0x18; // Output, z-axis gyroscope, low word
static constexpr uint8_t Z_GYRO_OUT = 0x1A; // Output, z-axis gyroscope, high word

static constexpr uint8_t X_ACCL_LOW = 0x1C; // Output, x-axis accelerometer, low word
static constexpr uint8_t X_ACCL_OUT = 0x1E; // Output, x-axis accelerometer, high word
static constexpr uint8_t Y_ACCL_LOW = 0x20; // Output, y-axis accelerometer, low word
static constexpr uint8_t Y_ACCL_OUT = 0x22; // Output, y-axis accelerometer, high word
static constexpr uint8_t Z_ACCL_LOW = 0x24; // Output, z-axis accelerometer, low word
static constexpr uint8_t Z_ACCL_OUT = 0x26; // Output, z-axis accelerometer, high word

static constexpr uint8_t TIME_STAMP = 0x28; // Output, time stamp

static constexpr uint8_t CRC_LWR    = 0x2A; // Output, CRC-32 (32 bits), lower word
static constexpr uint8_t CRC_UPR    = 0x2C; // Output, CRC-32, upper word

static constexpr uint8_t PAGE_ID  = 0x0; // Control, page ID selector

static constexpr uint8_t BURST_CMD  = 0x7C; // Cntrol, Burst-read command

static constexpr uint8_t FNCTIO_CTRL  = 0x06; // Cntrol, function

static constexpr uint8_t NULL_CNFG  = 0x0E; // Cntrol, bias est

static constexpr uint8_t GLOB_CMD   = 0x02; // Control, global commands

static constexpr uint8_t DEC_RATE   = 0x0C; // Control, output sample rate decimation
static constexpr uint8_t RANG_MDL   = 0x12; // Measurement range (model-specific) Identifier TODO use this

static constexpr uint8_t FILTR_BNK_0   = 0x16;
static constexpr uint8_t FILTR_BNK_1   = 0x18;

static constexpr uint8_t PROD_ID    = 0x7E;

static constexpr uint16_t PROD_ID_ADIS16497 = 0x4071; // ADIS16497 Identification, device number

static constexpr int T_STALL = 2;

using namespace time_literals;

ADIS16497::ADIS16497(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	SPI("ADIS16497", path_accel, bus, device, SPIDEV_MODE3, 5000000),
	_gyro(new ADIS16497_gyro(this, path_gyro)),
	_sample_perf(perf_alloc(PC_ELAPSED, "ADIS16497_read")),
	_sample_interval_perf(perf_alloc(PC_INTERVAL, "ADIS16497_read_int")),
	_bad_transfers(perf_alloc(PC_COUNT, "ADIS16497_bad_transfers")),
	_rotation(rotation)
{

	_debug_enabled = true;

#ifdef GPIO_SPI1_RESET_ADIS16497
	// ADIS16497 configure reset
	px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16497);
#endif /* GPIO_SPI1_RESET_ADIS16497 */

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16497;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16497;

	// set software low pass filter for controllers
	const param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
	float accel_cut = ADIS16497_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

	if (accel_cut_ph != PARAM_INVALID && param_get(accel_cut_ph, &accel_cut) == PX4_OK) {
		_accel_filter.set_cutoff_frequency(_sample_rate, accel_cut);

	} else {
		PX4_ERR("IMU_ACCEL_CUTOFF param invalid");
	}

	const param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
	float gyro_cut = ADIS16497_GYRO_DEFAULT_DRIVER_FILTER_FREQ;

	if (gyro_cut_ph != PARAM_INVALID && param_get(gyro_cut_ph, &gyro_cut) == PX4_OK) {
		_gyro_filter.set_cutoff_frequency(_sample_rate, gyro_cut);

	} else {
		PX4_ERR("IMU_GYRO_CUTOFF param invalid");
	}
}

ADIS16497::~ADIS16497()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_sample_interval_perf);
	perf_free(_bad_transfers);
}

int
ADIS16497::init()
{
	if (hrt_absolute_time() < 250_ms) {
		// power-on startup time (if needed)
		up_mdelay(250);
	}

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		/* if probe/setup failed, bail now */
		DEVICE_DEBUG("SPI setup failed");
		return PX4_ERROR;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0.0f;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0.0f;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0.0f;
	_gyro_scale.z_scale  = 1.0f;

	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	int ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* fetch an initial set of measurements for advertisement */
	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp = {};

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp, &_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
	}

	sensor_gyro_s grp = {};
	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp, &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
	}

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int ADIS16497::reset()
{
	DEVICE_DEBUG("resetting");

#ifdef GPIO_SPI1_RESET_ADIS16497
	// ADIS16497 reset
	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 0);

	// The RST line must be in a low state for at least 10 μs to ensure a proper reset initiation and recovery.
	up_udelay(10);

	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 1);
#else
	{
		// reset (global command bit 7)
		uint8_t value[2] = {};
		value[0] = (1 << 7);
		write_reg16(PAGE_ID, 0x03);
		write_reg16(GLOB_CMD, (uint16_t)value[0]);

		// Reset Recovery Time
		up_mdelay(210);

	}
#endif /* GPIO_SPI1_RESET_ADIS16497 */

	// Functional IO control
	static constexpr uint16_t FNCTIO_CTRL_DEFAULT = 0x000D;

	write_reg16(PAGE_ID, 0x03);
 	write_reg16(FNCTIO_CTRL, FNCTIO_CTRL_DEFAULT);
 		
 	up_udelay(340);

 	const uint16_t fnctio_ctrl = read_reg16(FNCTIO_CTRL);

 	if (fnctio_ctrl != FNCTIO_CTRL_DEFAULT) {
 		PX4_ERR("Invalid setup, FNCTIO_CTRL=%#X", fnctio_ctrl);
 		return PX4_ERROR;
 	}

 	// Continious bias estimation
 	static constexpr uint16_t NULL_CNFG_SETUP = 0x0000; // Disable continious bias estimation
	
	write_reg16(PAGE_ID, 0x03);
	write_reg16(NULL_CNFG, NULL_CNFG_SETUP);

	up_udelay(71);

 	const uint16_t null_cnfg = read_reg16(NULL_CNFG);
 	
 	if (null_cnfg != NULL_CNFG_SETUP) {
 		PX4_ERR("Invalid setup,NULL_CNFG=%#X", null_cnfg);
 		return PX4_ERROR;
 	}

	// Bartlett Window FIR Filter
	static constexpr uint16_t FILTR_BNK_0_SETUP = 0x0000; // (disabled: 0x0000)
	static constexpr uint16_t FILTR_BNK_1_SETUP = 0x0000; // (disabled: 0x0000)

	write_reg16(PAGE_ID, 0x03);
 	write_reg16(FILTR_BNK_0, FILTR_BNK_0_SETUP);
 	write_reg16(FILTR_BNK_1, FILTR_BNK_1_SETUP);

 	up_udelay(65);

 	const uint16_t filtr_bnk_0 = read_reg16(FILTR_BNK_0);

	if (filtr_bnk_0 != FILTR_BNK_0_SETUP) {
		PX4_ERR("Invalid setup, FILTR_BNK_0=%#X", filtr_bnk_0);
		return PX4_ERROR;
	}

 	const uint16_t filtr_bnk_1 = read_reg16(FILTR_BNK_1);

	if (filtr_bnk_1 != FILTR_BNK_1_SETUP) {
		PX4_ERR("Invalid setup, FILTR_BNK_1=%#X", filtr_bnk_1);
		return PX4_ERROR;
	}

	// Decimation Filter
	static constexpr uint16_t DEC_RATE_SETUP = 0x0003; //  4250/4 = 1062 samples per second
	
	write_reg16(PAGE_ID, 0x03);
	write_reg16(DEC_RATE, DEC_RATE_SETUP);

	up_udelay(340);

 	const uint16_t dec_rate = read_reg16(DEC_RATE);
 	
 	if (dec_rate != DEC_RATE_SETUP) {
 		PX4_ERR("Invalid setup, DEC_RATE=%#X", dec_rate);
 		return PX4_ERROR;
 	}

 	// TODO : CONFIG registers and all others

/*
	write_reg16(PAGE_ID, 0x03);
 	uint16_t filtr_cfg = read_reg16(FILTR_BNK_0);
 	PX4_INFO("filter 0: %#X", filtr_cfg);
 	filtr_cfg = read_reg16(FILTR_BNK_1);
 	PX4_INFO("filter 1: %#X", filtr_cfg);
 	
 	sensor_scale = read_reg16(0x06);
 	PX4_INFO("gyro Y scale: %#X", sensor_scale);
 	sensor_scale = read_reg16(0x08);
 	PX4_INFO("gyro Z scale: %#X", sensor_scale);
 	sensor_scale = read_reg16(0x1E);
 	PX4_INFO("accel X scale: %#X", sensor_scale);
 	sensor_scale = read_reg16(0x22);
 	PX4_INFO("accel Y scale: %#X", sensor_scale);
 	sensor_scale = read_reg16(0x26);
 	PX4_INFO("accel Z scale: %#X", sensor_scale);

 	// null_cfg registers
	write_reg16(PAGE_ID, 0x03);

 	uint16_t null_cfg = read_reg16(0x0E);
 	PX4_INFO("null_cfg: %#X", null_cfg);

 	uint8_t value[2] = {};
	value[0] = (1 << 3);
	write_reg16(PAGE_ID, 0x03);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);

	// save Recovery Time
	up_mdelay(1125);*/

	return OK;
}

int
ADIS16497::probe()
{
	DEVICE_DEBUG("probe");

	// read product id (5 attempts)
	for (int i = 0; i < 5; i++) {

		if (reset() != PX4_OK) {
			continue;
		}

		write_reg16(PAGE_ID, 0x00);
		_product = read_reg16(PROD_ID);

		if (_product == PROD_ID_ADIS16497) {

			if (self_test_sensor()) {
				return PX4_OK;

			} else {
				PX4_ERR("probe attempt %d: self test failed, resetting", i);
			}

		} else {
			PX4_ERR("probe attempt %d: read product id failed, resetting", i);
		}
	}

	return -EIO;
}

bool
ADIS16497::self_test_sensor()
{
	DEVICE_DEBUG("self test sensor");

	// self test (global command bit 1)
	uint8_t value[2] = {};
	value[0] = (1 << 1);
	write_reg16(PAGE_ID, 0x03);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
	up_mdelay(20); // Self Test Time

	// read DIAG_STS to check result
	write_reg16(PAGE_ID, 0x0);
	uint16_t sys_e_flag = read_reg16(SYS_E_FLAG);

	if (sys_e_flag != 0) {
		PX4_ERR("SYS_E_FLAG: %#X", sys_e_flag);

		// Read DIAG_STS to check self-test results
		write_reg16(PAGE_ID, 0x0);
		uint16_t diag_sts_flag = read_reg16(DIAG_STS);

		if (diag_sts_flag != 0) {
			PX4_ERR("DIAG_STS: %#X", diag_sts_flag);
		}

		return false;
	}

	return true;
}

int
ADIS16497::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCRESET:
		return reset();

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			memcpy(&_accel_scale, s, sizeof(_accel_scale));
			return OK;
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16497::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint16_t
ADIS16497::read_reg16(uint8_t reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16497::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t cmd[2];
	cmd[0] = reg | 0x8;
	cmd[1] = val;
	transfer(cmd, cmd, sizeof(cmd));
}

void
ADIS16497::write_reg16(uint8_t reg, uint16_t value)
{
	uint16_t cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

void
ADIS16497::start()
{
#ifdef GPIO_SPI1_DRDY1_ADIS16497
	// setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ADIS16497, true, false, true, &ADIS16497::data_ready_interrupt, this);
#else
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&ADIS16497::measure_trampoline, this);
#endif
}

void
ADIS16497::stop()
{
#ifdef GPIO_SPI1_DRDY1_ADIS16497
	// stop data ready
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ADIS16497, false, false, false, nullptr, nullptr);
#else
	hrt_cancel(&_call);
#endif
}

int
ADIS16497::data_ready_interrupt(int irq, void *context, void *arg)
{
	ADIS16497 *dev = reinterpret_cast<ADIS16497 *>(arg);

	/* make another measurement */
	dev->measure();

	return PX4_OK;
}

void
ADIS16497::measure_trampoline(void *arg)
{
	ADIS16497 *dev = reinterpret_cast<ADIS16497 *>(arg);

	/* make another measurement */
	dev->measure();
}

int
ADIS16497::measure()
{
	perf_begin(_sample_perf);
	perf_count(_sample_interval_perf);

	// Fetch the full set of measurements from the ADIS16497 in one pass (burst read).
	ADISReport adis_report {};
	adis_report.cmd = ((BURST_CMD | DIR_READ) << 8) & 0xff00;

	// ADIS16497 burst report should be 320 bits
	static_assert(sizeof(adis_report) == (320 / 8), "ADIS16497 report not 320 bits");

	const hrt_abstime t = hrt_absolute_time();

	write_reg16(PAGE_ID, 0x0);
	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	if (adis_report.BURST_ID != 0xA5A5) {
		PX4_ERR("BURST_ID: %#X", adis_report.BURST_ID);
	}

	// Check all Status/Error Flag Indicators (SYS_E_FLAG)
	if (adis_report.SYS_E_FLAG != 0) {
		PX4_ERR("SYS_E_FLAG: %#X", adis_report.SYS_E_FLAG);

		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	uint32_t checksum_msg = uint32_t(adis_report.CRC_UPR) << 16 | adis_report.CRC_LWR;

	uint32_t checksum_calc = get_imu_crc32((uint16_t *)&adis_report.SYS_E_FLAG, 15);

	if (checksum_msg != checksum_calc) {
		PX4_ERR("CHECKSUM: %#X vs calculated: %#X", checksum_msg, checksum_calc);
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	publish_accel(t, adis_report);
	publish_gyro(t, adis_report);

	perf_end(_sample_perf);
	return OK;
}

void
ADIS16497::publish_accel(const hrt_abstime &t, const ADISReport &report)
{
	float xraw_f = (int32_t(report.X_ACCEL_OUT) << 16 | report.X_ACCEL_LOW) / 65536.0f * _accel_range_scale;
	float yraw_f = (int32_t(report.Y_ACCEL_OUT) << 16 | report.Y_ACCEL_LOW) / 65536.0f * _accel_range_scale;
	float zraw_f = (int32_t(report.Z_ACCEL_OUT) << 16 | report.Z_ACCEL_LOW) / 65536.0f * _accel_range_scale;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	const float x_in_new = (xraw_f - _accel_scale.x_offset) * _accel_scale.x_scale;
	const float y_in_new = (yraw_f - _accel_scale.y_offset) * _accel_scale.y_scale;
	const float z_in_new = (zraw_f - _accel_scale.z_offset) * _accel_scale.z_scale;

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);

	const matrix::Vector3f val_filt = _accel_filter.apply(aval);

	sensor_accel_s arb{};
	matrix::Vector3f aval_integrated;

	if (_accel_int.put(t, aval, aval_integrated, arb.integral_dt)) {
		arb.timestamp = t;

		arb.device_id = _device_id.devid;
		arb.error_count = perf_event_count(_bad_transfers);

		// raw sensor readings
		arb.x_raw = report.X_ACCEL_OUT;
		arb.y_raw = report.Y_ACCEL_OUT;
		arb.z_raw = report.Z_ACCEL_OUT;
		arb.scaling = _accel_range_scale;

		arb.x = val_filt(0);
		arb.y = val_filt(1);
		arb.z = val_filt(2);

		arb.x_integral = aval_integrated(0);
		arb.y_integral = aval_integrated(1);
		arb.z_integral = aval_integrated(2);

		/* Temperature report: */
		// temperature 1 LSB = 0.1°C
		arb.temperature = report.TEMP_OUT * 0.1f - 25.0f;

		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}
}

void
ADIS16497::publish_gyro(const hrt_abstime &t, const ADISReport &report)
{
	// ADIS16497-2BMLZ scale factory
	float xraw_f = (int32_t(report.X_GYRO_OUT) << 16 | report.X_GYRO_LOW) / 65536.0f;
	float yraw_f = (int32_t(report.Y_GYRO_OUT) << 16 | report.Y_GYRO_LOW) / 65536.0f;
	float zraw_f = (int32_t(report.Z_GYRO_OUT) << 16 | report.Z_GYRO_LOW) / 65536.0f;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	const float x_gyro_in_new = (math::radians(xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	const float y_gyro_in_new = (math::radians(yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	const float z_gyro_in_new = (math::radians(zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);

	const matrix::Vector3f gval_filt = _gyro_filter.apply(gval);

	sensor_gyro_s grb{};
	matrix::Vector3f gval_integrated;

	if (_gyro_int.put(t, gval, gval_integrated, grb.integral_dt)) {
		grb.timestamp = t;

		grb.device_id = _gyro->_device_id.devid;
		grb.error_count = perf_event_count(_bad_transfers);

		/* Gyro report: */
		grb.scaling = math::radians(_gyro_range_scale);
		grb.x_raw = report.X_GYRO_OUT;
		grb.y_raw = report.Y_GYRO_OUT;
		grb.z_raw = report.Z_GYRO_OUT;

		grb.x = gval_filt(0);
		grb.y = gval_filt(1);
		grb.z = gval_filt(2);

		grb.x_integral = gval_integrated(0);
		grb.y_integral = gval_integrated(1);
		grb.z_integral = gval_integrated(2);

		/* Temperature report: */
		// temperature 1 LSB = 0.1°C
		grb.temperature = report.TEMP_OUT * 0.1f - 25.0f;

		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}
}

void
ADIS16497::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_sample_interval_perf);
	perf_print_counter(_bad_transfers);
}
