#ifndef MPU_H_
#define MPU_H_

#include <stdbool.h>
#include "nrf_delay.h"

#define YAW		0
#define PITCH	1
#define ROLL	2
#define DIM		3

#define MPU_IRQ	6

#define MPU_nCS_PIN		5
#define MPU_INT_PIN		6
#define MPU_FSYNC_PIN	7

#define min(a,b) ((a)<(b)?(a):(b))
#define delay_ms(a)    nrf_delay_ms(a)
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

void get_ms(unsigned long *timestamp);

#define MPU_DEBUG
#define MPU9250
#define AK8963_SECONDARY

#define MPU_SAMPLING_RATE							9	//Hz //TODO+NOTE

#if defined MPU_DEBUG
void printf_P(char *s);
void printf_PP(char *s, ...);
extern void simple_uart_puthex(uint8_t hex);
extern void simple_uart_putstring(const uint8_t *str);
#endif

void twi_init(void);
void twi_disable(void);

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

int mpu_open();
void getAltimeterPressure(uint32_t* pressure, unsigned long* ts, int* altimeterMeasureState);
int mpu_reset(void);
//int mpu_update(int16_t* g, int16_t* a, int16_t* q, unsigned long* actts, uint32_t* pressure, bool* altimeterMeasure);
int mpu_update(int16_t* g, int16_t* a, int16_t* q, unsigned long* actts);
int mpu_close();

#endif	// MPU_H_



