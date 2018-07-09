#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_adc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "softdevice_handler.h"
#include "ble_bas.h"
#include "battery.h"
#include "app_util.h"

int battery_in_percentage = 0; 

/**@brief Function for handling the ADC result
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void ADC_IRQHandler(void)
{
	nrf_adc_conversion_event_clean();

	uint8_t adc_result = nrf_adc_result_get();

	battery_in_percentage = adc_result*100/163;

	if(battery_in_percentage > 100)
		battery_in_percentage = 100;

	NRF_GPIO->PIN_CNF[BATTERY_GND_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |  (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}

void battery_start(void)
{
	nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

	nrf_gpio_cfg_output(BATTERY_GND_PIN);
	nrf_gpio_pin_clear(BATTERY_GND_PIN);

	nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
	nrf_adc_config.resolution = NRF_ADC_CONFIG_RES_8BIT;

	// Initialize and configure ADC
	nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
	nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);	// P0.02 = AIN3
	nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
	NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
	NVIC_EnableIRQ(ADC_IRQn);
	nrf_adc_start();
}



