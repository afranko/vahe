/*
 *
 */

/* -------- */
/* Includes */
/* -------- */

#include "charger_mgmt.h"

/* ------------------------------- */
/* Global variaables and functions */
/* ------------------------------- */

extern inline void chg_adc_isr_off_callback(void);

/* --------------------------------------------------- */
/* Static variables, constants, definitions and macros */
/* --------------------------------------------------- */

#define GPIO_LOW    0
#define GPIO_HIGH   1

#define CHR_INPUT_SEL   (1ULL << CHR_INPUT)
#define CHR_EN1_SEL (1ULL << CHR_EN1)
#define CHR_EN2_SEL (1ULL << CHR_EN2)

static const char* TAG = "ChargingMGMT";
static gpio_config_t io_conf;

/* ---------------------------- */
/* Static function declarations */
/* ---------------------------- */

/**
 *
 */
static void IRAM_ATTR chr_isr_handler(void *arg)
{
    ESP_LOGI(TAG, "Charger ISR is called!\n");
    if(gpio_get_level((gpio_num_t) arg))
    {
        /* 1.5A enabled */
        gpio_set_level(CHR_EN1_SEL, GPIO_LOW);
        gpio_set_level(CHR_EN2_SEL, GPIO_HIGH);
        ESP_LOGD(TAG, "CHG ISR:\nInput: LOW\nEN1: HIGH\nEN2: LOW\n");
    }
    else
    {
        /* 0.5A enabled - 1.5A disabled */
        gpio_set_level(CHR_EN1_SEL, GPIO_HIGH);
        gpio_set_level(CHR_EN2_SEL, GPIO_LOW);
        ESP_LOGD(TAG, "CHG ISR:\nInput: HIGH\nEN1: LOW\nEN2: HIGH\n");
    }
}

/* --------------------------- */
/* Implementation of functions */
/* --------------------------- */

//TODO hibakezelés (mi van ha nem sikerül elindtani a történetet??)

/**
 * @brief   Initializes the Charger Management System and configures corresponding GPIO pins
 * and Interrupt Service Routines (ISR).
 * @return
 *   - ESP_OK Success
 *   - Possible return values of gpio_install_isr_service() function
 */
esp_err_t chg_init(void)
{
    ESP_LOGV(TAG, "Charger MGMT init is called!\n");

    /* CHR_INPUT Pin settings */
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = CHR_INPUT_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    esp_error_indicator = gpio_config(&io_conf);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* CHR_EN1 Pin settings and preset */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = CHR_EN1_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    esp_error_indicator = gpio_config(&io_conf);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(CHR_EN1_SEL, GPIO_HIGH));

    /* CHR EN2 Pin setings and preset */
    io_conf.pin_bit_mask = CHR_EN2_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(CHR_EN2_SEL, GPIO_LOW));

    /* ISR settings */
    esp_error_indicator = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);   //TODO meg kell nézni hogy hány IT LEVEL kell
    if(esp_error_indicator != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR Service can't be initialized \n");
        return esp_error_indicator;
    }

    ESP_LOGI(TAG, "GPIO ISR Service has been initialized!\n");
    ESP_ERROR_CHECK(gpio_isr_handler_add(CHR_INPUT_SEL, chr_isr_handler, (void*)));
    return ESP_OK;
}

//TODO + NOTE
void chg_adc_isr_on_callback(void)
{
    ESP_LOGD(TAG, "Charger-ADC ISR ON callback is called!");
    ESP_ERROR_CHECK(gpio_isr_handler_add(CHR_INPUT_SEL, chr_isr_handler, (void*)));
}
