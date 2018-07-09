#ifndef BATTERY_H__
#define BATTERY_H__

#define BATTERY_GND_PIN			1
#define BATTERY_MEAS_PIN		2

extern int battery_in_percentage;

/**@brief Function for making the ADC start a battery level conversion.
*/
void battery_start(void);
void battery_measurement_result(uint8_t result);

#endif // BATTERY_H__

