#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "mpu.h"

#define QUAT_W		0
#define QUAT_X		1
#define QUAT_Y		2
#define QUAT_Z		3

static const nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);

void get_ms(unsigned long *timestamp)
{
	uint32_t ticks = 0;
	app_timer_cnt_get(&ticks);

	timestamp[0] = 1.0f*ticks/32.768f;
}

void twi_init(void)
{
	nrf_gpio_cfg_input(MPU_INT_PIN, NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_output(MPU_FSYNC_PIN);
	nrf_gpio_pin_clear(MPU_FSYNC_PIN);

	nrf_gpio_cfg_output(MPU_nCS_PIN);
	nrf_gpio_pin_set(MPU_nCS_PIN);

	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config =
	{
		.scl						= 4,
		.sda						= 3,
		.frequency				= NRF_TWI_FREQ_400K,
		.interrupt_priority	= APP_IRQ_PRIORITY_HIGH
	};

	err_code = nrf_drv_twi_init(&twi_instance, &twi_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&twi_instance);
}

void twi_disable(void)
{
	nrf_drv_twi_disable(&twi_instance);
}

void printf_P(char *s)
{
	simple_uart_putstring((const uint8_t *)s);
}

void printf_PP(char *s, ...)
{
	simple_uart_putstring((const uint8_t *)s);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data)
{
	ret_code_t transfer_succeeded;

	transfer_succeeded = nrf_drv_twi_tx(&twi_instance, slave_addr, &reg_addr, sizeof(reg_addr), true);
	if(transfer_succeeded == NRF_SUCCESS)
	{
		transfer_succeeded = nrf_drv_twi_tx(&twi_instance, slave_addr, (uint8_t*)data, length, false);
	}

	return transfer_succeeded == NRF_SUCCESS?0:1;
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	ret_code_t transfer_succeeded;

	transfer_succeeded = nrf_drv_twi_tx(&twi_instance, slave_addr, &reg_addr, sizeof(reg_addr), true);
	if(transfer_succeeded == NRF_SUCCESS)
	{
		transfer_succeeded = nrf_drv_twi_rx(&twi_instance, slave_addr, (uint8_t*)data, length, false);
	}

	return transfer_succeeded == NRF_SUCCESS?0:1;
}


//int16_t a[3];	//[x, y, z]	accel vector
//int16_t g[3];	//[x, y, z]	gyro vector
//int32_t q[4];
int16_t sensors;
//unsigned long actts = 0;
uint8_t fifoCount;

//Altimeter calibration datas
uint16_t c1, c2, c3, c4, c5, c6;


double quaternionNormalize(int32_t* q, double* qf)
{
	double length = sqrt(q[QUAT_W]*q[QUAT_W] + q[QUAT_X]*q[QUAT_X] + q[QUAT_Y]*q[QUAT_Y] + q[QUAT_Z]*q[QUAT_Z]);
	
	qf[QUAT_W] = q[QUAT_W]/length;
	qf[QUAT_X] = q[QUAT_X]/length;
	qf[QUAT_Y] = q[QUAT_Y]/length;
	qf[QUAT_Z] = q[QUAT_Z]/length;
	
	return length;
}

void quaternionToEuler(double* q, double *v)
{
	v[1] = asinf(2.0f * (q[QUAT_W] * q[QUAT_Y] - q[QUAT_X] * q[QUAT_Z]));
	v[0] = atan2f(2.0f * (q[QUAT_Y] * q[QUAT_Z] + q[QUAT_W] * q[QUAT_X]), 1.0f - 2.0f * (q[QUAT_X] * q[QUAT_X] + q[QUAT_Y] * q[QUAT_Y]));
	v[2] = atan2f(2.0f * (q[QUAT_X] * q[QUAT_Y] + q[QUAT_W] * q[QUAT_Z]), 1.0f - 2.0f * (q[QUAT_Y] * q[QUAT_Y] + q[QUAT_Z] * q[QUAT_Z]));
	
	v[0] = v[0]*180.0/M_PI;
	v[1] = v[1]*180.0/M_PI;
	v[2] = v[2]*180.0/M_PI;
}

bool isAltimeterAvailable = false;

int mpu_open()
{
	if(mpu_init(NULL) != 0)
	{
		simple_uart_putstring((const uint8_t *)"MPU init failed!\n\r");
		return -1;
	}

	if(mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to set sensors!\n\r");
		return -1;
	}

	if(mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to initialize MPU fifo!\n\r");
		return -1;
	}

	if(mpu_set_sample_rate(MPU_SAMPLING_RATE))
	{
		simple_uart_putstring((const uint8_t *)"mpu_set_sample_rate() failed\n");
		return -1;
	}

	if(mpu_set_compass_sample_rate(MPU_SAMPLING_RATE))
	{
		simple_uart_putstring((const uint8_t *)"mpu_set_compass_sample_rate() failed\n");
		return -1;
	}

	if(dmp_load_motion_driver_firmware()!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to enable DMP!\n\r");
		return -1;
	}

	if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to enable DMP features!\n\r");
		return -1;
	}

	if(dmp_set_fifo_rate(MPU_SAMPLING_RATE)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to set dmp fifo rate!\n\r");
		return -1;
	}

	if(dmp_enable_gyro_cal(1)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to enable gyro calibration!\n\r");
		return -1;
	}

	if(mpu_set_dmp_state(1)!=0)
	{
		simple_uart_putstring((const uint8_t *)"Failed to enable DMP!\n\r");
		return -1;
	}
	

	long gyroBias[3];
	long accelBias[3];
	int result = mpu_run_self_test(gyroBias, accelBias);
	
	if (result == 0x7)
	{
		float sens;
		unsigned short accel_sens;
	
		mpu_get_gyro_sens(&sens);
		gyroBias[0] = (long)(gyroBias[0] * sens);
		gyroBias[1] = (long)(gyroBias[1] * sens);
		gyroBias[2] = (long)(gyroBias[2] * sens);
		dmp_set_gyro_bias(gyroBias);
		mpu_get_accel_sens(&accel_sens);
		accelBias[0] *= accel_sens;
		accelBias[1] *= accel_sens;
		accelBias[2] *= accel_sens;
		dmp_set_accel_bias(accelBias);

		char logstr[128];
		sprintf(logstr, "Biases: %ld %ld %ld %ld %ld %ld\n\r", gyroBias[0], gyroBias[1], gyroBias[2], accelBias[0], accelBias[1], accelBias[2]);
		simple_uart_putstring((const uint8_t *)logstr);
	}

	if(mpu_set_bypass(1)!=0)
		simple_uart_putstring((const uint8_t *)"Failed to enabled bypass mode!\n\r");
	
	//Reset
	uint8_t command=0x1e;
	bool success = nrf_drv_twi_tx(&twi_instance, 0x77, &command, sizeof(command), false);

	if(success != NRF_SUCCESS)
		simple_uart_putstring((const uint8_t *)"Reset failed!\n\r");

	nrf_delay_ms(3);

	//PROM read
	uint8_t buffer[2];
	success = i2c_read(0x77, 0xA2, sizeof(buffer), buffer);
	c1 = (buffer[0]<<8 | buffer[1]);
	success |= i2c_read(0x77, 0xA4, sizeof(buffer), buffer);
	c2 = (buffer[0]<<8 | buffer[1]);
	success |= i2c_read(0x77, 0xA6, sizeof(buffer), buffer);
	c3 = (buffer[0]<<8 | buffer[1]);
	success |= i2c_read(0x77, 0xA8, sizeof(buffer), buffer);
	c4 = (buffer[0]<<8 | buffer[1]);
	success |= i2c_read(0x77, 0xAA, sizeof(buffer), buffer);
	c5 = (buffer[0]<<8 | buffer[1]);
	success |= i2c_read(0x77, 0xAC, sizeof(buffer), buffer);
	c6 = (buffer[0]<<8 | buffer[1]);

	if(success != NRF_SUCCESS)
	{
		simple_uart_putstring((const uint8_t *)"PROM read rx failed!\n\r");
	}
	else
	{
		isAltimeterAvailable = true;
	
		uint32_t pressure = 0;
		unsigned long ts = 0;
		int altimeterMeasureState = 0;
		getAltimeterPressure(&pressure, &ts, &altimeterMeasureState);
		nrf_delay_us(8220);
		getAltimeterPressure(&pressure, &ts, &altimeterMeasureState);
		nrf_delay_us(8220);
		getAltimeterPressure(&pressure, &ts, &altimeterMeasureState);
	
		char logstr[128];
		sprintf(logstr, "Pressure: %ld\n\r", pressure);
		simple_uart_putstring((const uint8_t *)logstr);
	}
		
	if(mpu_set_bypass(0)!=0)
		simple_uart_putstring((const uint8_t *)"Failed to disabled bypass mode!\n\r");

/*
	char logstr[128];
	double v[3] = {0,0,0};
	double qf[4] = {0.0,0.0,0.0,0.0};
	uint32_t pressure = 0;
	int i = 0;
	
	//TODO: Infinite loop for testing only!
	do{
		nrf_delay_ms(1000/MPU_SAMPLING_RATE/3);
		
		if(i > 100)
		{
			getAltimeterPressure(&pressure);
			i=0;
		}

		do{
			int r = dmp_read_fifo(g, a, q, &actts, &sensors, &fifoCount);

			if(r == 0)
			{
				q[0] = q[0]/65536;
				q[1] = q[1]/65536;
				q[2] = q[2]/65536;
				q[3] = q[3]/65536;

				double length = quaternionNormalize(q, qf);

				if(length > 0.0)
				{
					quaternionToEuler(qf, v);

					sprintf(logstr, "%d\t%ld\t%ld\t%3.2f\t%3.2f\t%3.2f\t%d\t%d\t%d\n\r", fifoCount, actts, pressure, v[0], v[1], v[2], a[0], a[1], a[2]);
					simple_uart_putstring((const uint8_t *)logstr);
				}
			}
		}while(fifoCount > 0);
		
		i++;
	}
	while(true);
	
	simple_uart_putstring((const uint8_t *)"Done.\n\r");
*/
	return 0;
}

uint32_t d1 = 0;
uint32_t d2 = 0;

void getAltimeterPressure(uint32_t* pressure, unsigned long* ts, int* altimeterMeasureState)
{
	if(isAltimeterAvailable)
	{
		//delays: 0.5 / 1.1 / 2.1 / 4.1 / 8.22 ms

		if(mpu_set_bypass(1)!=0)
			simple_uart_putstring((const uint8_t *)"Failed to enabled bypass mode!\n\r");

		//Start conversation, D1(pressure) osr=4096 : 0x48
		//Start conversation, D1(pressure) osr=2048 : 0x46
		//Start conversation, D1(pressure) osr=1024 : 0x44
		//Start conversation, D1(pressure) osr= 512 : 0x42
		//Start conversation, D1(pressure) osr= 256 : 0x40
	
		uint8_t command;
		bool success;
		uint8_t buffer24[3] = {0,0,0};
	
		switch(*altimeterMeasureState)
		{
			case 0:
				command=0x48;
				success = nrf_drv_twi_tx(&twi_instance, 0x77, &command, sizeof(command), false);
				if(success != NRF_SUCCESS)
					simple_uart_putstring((const uint8_t *)"Conv start failed!\n\r");

				app_timer_cnt_get(ts);
				*altimeterMeasureState = 1;
				break;
			case 1:
//			nrf_delay_us(8220);
				success = i2c_read(0x77, 0x00, sizeof(buffer24), buffer24);
				d1 = (buffer24[0]<<16 | buffer24[1]<<8 | buffer24[2]);
				if(success != NRF_SUCCESS)
					simple_uart_putstring((const uint8_t *)"ADC read failed!\n\r");	
		//		sprintf(logstr, "D1: %ld\n\r", d1);
		//		simple_uart_putstring((const uint8_t *)logstr);

				//Start conversation, D2(pressure) osr=4096 : 0x58
				//Start conversation, D2(pressure) osr=2048 : 0x56
				//Start conversation, D2(pressure) osr=1024 : 0x54
				//Start conversation, D2(pressure) osr= 512 : 0x52
				//Start conversation, D2(pressure) osr= 256 : 0x50
				command=0x58;
				success = nrf_drv_twi_tx(&twi_instance, 0x77, &command, sizeof(command), false);
				if(success != NRF_SUCCESS)
					simple_uart_putstring((const uint8_t *)"Conv start failed!\n\r");	
				*altimeterMeasureState = 2;
				break;
			case 2:
//			nrf_delay_us(8220);
				success = i2c_read(0x77, 0x00, sizeof(buffer24), buffer24);
				d2 = (buffer24[0]<<16 | buffer24[1]<<8 | buffer24[2]);
				if(success != NRF_SUCCESS)
					simple_uart_putstring((const uint8_t *)"ADC read failed!\n\r");
		//		sprintf(logstr, "D2: %ld\n\r", d2);
		//		simple_uart_putstring((const uint8_t *)logstr);

				int32_t dt = d2 - c5*256;
		//		double temp = 20.00 + (1.0*dt*c6/838860800);

				int64_t off = 1.0*c2*65536+(1.0*c4*dt)/128;
				int64_t sens = 1.0*c1*32768+(1.0*c3*dt)/256;
				*pressure = 1.0*(1.0*d1*sens/2097152 - off)/32768;
				*altimeterMeasureState = 3;
				break;
		}
		
		if(mpu_set_bypass(0)!=0)
			simple_uart_putstring((const uint8_t *)"Failed to disabled bypass mode!\n\r");

/*
		sprintf(logstr, "dt: %ld\n\r", dt);
		simple_uart_putstring((const uint8_t *)logstr);

		sprintf(logstr, "temp: %3.2f\n\r", temp);
		simple_uart_putstring((const uint8_t *)logstr);

		sprintf(logstr, "off: %3.2f\n\r", (double)off);
		simple_uart_putstring((const uint8_t *)logstr);

		sprintf(logstr, "sens: %3.2f\n\r", (double)sens);
		simple_uart_putstring((const uint8_t *)logstr);

		sprintf(logstr, "p: %ld\n\r", p);
		simple_uart_putstring((const uint8_t *)logstr);
*/
	}
	else
		*altimeterMeasureState = 0;
}

int mpu_reset(void)
{
	return mpu_reset_fifo();
}

//int mpu_update(int16_t* g, int16_t* a, int16_t* q, unsigned long* actts, uint32_t* pressure, bool* altimeterMeasure)
int mpu_update(int16_t* g, int16_t* a, int16_t* q, unsigned long* actts)
{
//	char logstr[128];
	int32_t qt[4];
	
	int r = dmp_read_fifo(g, a, qt, actts, &sensors, &fifoCount);

	if(r == 0)
	{
		q[0] = qt[0]/65536;
		q[1] = qt[1]/65536;
		q[2] = qt[2]/65536;
		q[3] = qt[3]/65536;

		return fifoCount;
	}
	
	return -1;
}


