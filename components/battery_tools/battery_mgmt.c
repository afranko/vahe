/*
 * Battery Management
 */

#include "battery_mgmt.h"
#include "charger_mgmt.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

/* ----------------- */
/* Structs and types */
/* ----------------- */

typedef struct{
    float batVoltage;
    uint32_t rawVoltage;
    time_t *actTime;
    struct tm lastTimeStamp;
    uint8_t preScaleResistors[2];    // NOTE: in kOHMs
}battery_mgmt_t;

/* ------------------------------------- */
/* Hardware related variables and macros */
/* ------------------------------------- */

#define GPIO_LOW    0
#define GPIO_HIGH   1

#define DEFAULT_VREF    1100    //TODO beálltás

static const char* TAG = "BatteryMGMT";

static const adc_channel_t channel = BTR_ADC_CHANNEL_NO;
static const adc_atten_t atten = ADC_ATTEN_DB_11;   // TODO komment + kitalálni hogy mi a fasz
static const adc_unit_t unit = ADC_UNIT_1;

static esp_adc_cal_characteristics_t adc_chars;
static gpio_config_t gndConf;

static battery_mgmt_t batteryMgmt;

static void battery_measurement_full(void); //TODO LED vs. Üzenet?

void battery_measurement_full(void)
{
    uint32_t rawInputValue = 0;

    chg_adc_isr_off_callback();
    adc_power_on(); // TODO - energiahatékonyság??

    //TODO energiahatékonyásg miatt, a SAR ki-be kapcsolása - Charging Callback

    /* Set GND Pin to 0 (ground) */
    gndConf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&gndConf);
    gpio_set_level(ADC_GND_PIN, GPIO_LOW);  // Set pin to GND

    /* Reading raw values from ADC multiple times */
    for(int adcCounter = 0; adcCounter < BTR_SAMPLE_NUM; adcCounter++)
        rawInputValue += adc_get_raw((adc_channel_t)channel);

    /* Set GND Pin to HighZ (input) */
    gndConf.mode = GPIO_MODE_INPUT;
    gpio_config(&gndConf);  // Set pin to input mode (HighZ)

    /* Calculating battery voltage */
    rawInputValue /= BTR_SAMPLE_NUM;    //TODO CHECK
    batteryMgmt.rawVoltage = esp_adc_cal_raw_to_voltage(rawInputValue, adc_chars);
    batteryMgmt.batVoltage = (float) (batteryMgmt.batVoltage * batteryMgmt.preScaleResistors[1]) /
        (batteryMgmt.preScaleResistors[0] + batteryMgmt.preScaleResistors[1]);    // Prescale
    //batteryMgmt.batVoltage = (batteryMgmt.batVoltage / 4096U) * 3.9f;   // Scale
    //TODO ezek
    /* Thread-safe timestamp reading */
    batteryMgmt.lastTimeStamp = localtime_r((const time_t*)batteryMgmt.actTime);
    adc_power_off();    // TODO
    chg_adc_isr_on_callback(); //TODO 80ns-es időztés -> elvileg teljesül NOTE: 80ns -> 125 MHz
}

esp_err_t battery_mgmt_set_time(time_t *actualTime)  //TODO inline
{
    if(actualTime == NULL) {
        ESP_LOGE(TAG, "Time for battery management cannot be set due to time is a nullptr!\n"); // TODO Warningx
        return ESP_ERR_INVALID_ARG;
    }

    batteryMgmt.actTime = actualTime;
    return ESP_OK;
}

void init_battery_mgmt(time_t *actualTime)
{
    /* Init structure */
    esp_err_t esp_error_indicator = battery_mgmt_set_time(actualTime);
    ESP_ERROR_CHECK(esp_error_indicator);

    //ERROR handling TODO - FATAL?

    batteryMgmt.preScaleResistors[0] = BTR_ADC_PRESCALE_RESISTOR_UP;
    batteryMgmt.preScaleResistors[1] = BTR_ADC_PRESCALE_RESISTOR_DOWN;

    ERROR_ESP_CHECK(battery_mgmt_set_time);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    /* Check ADC Calibration */
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);    //TODO DEFAULT VREF beálltás
    ESP_LOGV(TAG, "Reference values:\neFuse Vref = 0\nTwo Point Value = 1\nDefault Vref = 2\n");
    ESP_LOGD(TAG, "Characterized using: %s\n", character_val_to_string(val_type));

    //TODO adc2_calib -> kalibrálást el kell végezni
    //TODO karakterisztika check?
    //TODO esp_adc_cal_raw_to_voltage check mi van?

    /* Init GND Pin */
    gndConf.intr_type = GPIO_PIN_INTR_DISABLE;
    gndConf.pin_bit_mask = BTR_ADC_GND_PIN; //TODO ezt a kurva nagy magic-et meg kell nézni
    gndConf.pull_down_en = 0;
    gndConf.pull_up_en = 0;
    gndConf.mode = GPIO_MODE_INPUT;
    gpio_config(&gndConf);
}
