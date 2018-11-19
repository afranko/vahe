#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "decadriver/deca_device_api.h"

#include "ble_lbs.h"
#include "nrf_deca.h"
#include "dw_tag.h"
#include "dw_anchor.h"

#include "rfid.h"

#ifdef TAG_MODE
#include "wifi_con.h"
#endif

#include "commons.h"

#define CHG_DET	39
#define CHG_LO	26
#define CHG_HI	27

extern SemaphoreHandle_t xSemaphore;


const char intArray[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

TimerHandle_t xTimers[5];	// TIMERS
TimerHandle_t blinkTimer;

static TaskHandle_t xTaskToNotify = NULL;

static bool blinker = false;

unsigned int wifi_conn_cnt = 0; //divide the ts counter to check wifi connection

void vTimerCallback(TimerHandle_t pxTimer)
{
	switch((uint32_t) pvTimerGetTimerID(pxTimer))
	{
#ifdef TAG_MODE
	case 0:
		measurement_timeout_handler();
		break;
	case 2:
		ts_timeout_handler();

		if((wifi_conn_cnt % 10000) == 0)
		{
			if(connection_check())
				ets_printf("Wifi connected \n");
			else
				ets_printf("Wifi cant reconnect \n");
		}
		wifi_conn_cnt++;
		break;
#endif
	case 10:
		gpio_set_level(GPIO_NUM_2, blinker);
		blinker = !blinker;
		break;
	case 1:
		//if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			beacon_timeout_handler();
		//	xSemaphoreGive(xSemaphore);
		//}
		break;
	default:
		break;
	}
}

static void timers_init(void)
{
    printf("Timer Init OK.\n\r");

#ifdef TAG_MODE

    xTimers[0] = xTimerCreate("measurement_timer", MEASURE_TICKNUM/portTICK_PERIOD_MS, pdTRUE , (void*)0, vTimerCallback);
    xTimers[2] = xTimerCreate("ts_timer", TS_TICKNUM/portTICK_PERIOD_MS, pdTRUE , (void*)2, vTimerCallback);
    init_rx_timeout_timer();

#endif

    xTimers[1] = xTimerCreate("beacon_timer", BEACON_TICKNUM/portTICK_PERIOD_MS, pdTRUE , (void*)1, vTimerCallback);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;	//TODO+NOTE most akkor posedge negedge vagy high????
   	io_conf.mode = GPIO_MODE_OUTPUT;
   	io_conf.pin_bit_mask = ( 1ULL << 2);
   	io_conf.pull_down_en = 0;
   	io_conf.pull_up_en = 0;
   	gpio_config(&io_conf);

   	gpio_set_level(GPIO_NUM_2, 0);
    blinkTimer = xTimerCreate("blink_timer", 500/portTICK_PERIOD_MS, pdTRUE, (void*)10, vTimerCallback);
}

typedef struct
{
    uint8_t type;       // device type, 0x01 => Anchor, 0x02 => Tag
    uint8_t mac[4];     // UWB MAC address (ID)
    uint8_t version;
    uint8_t tx_power;   // The 2's complement of the calibrated Tx Power  (<RSSI @ 1m>)
} manuf_info_t;

#ifdef TAG_MODE
static bool m_measurement_timer_on = false;

static void measurement_update_handler(ble_lbs_t *p_lbs, uint8_t new_state)
{
    switch(new_state)
    {
    case 	START_RANGING:
        if(!m_measurement_timer_on)
        {
        	xTimerStart(xTimers[0], 0);	//TODO+NOTE
            m_measurement_timer_on = true;
        }
        break;
    case	STOP_RANGING:
        if(p_lbs->is_ranging_notification_enabled == false)
        {
            xTimerStop(xTimers[0], 0);		//TODO+NOTE
            m_measurement_timer_on = false;
        }
        break;
    case 	ANCHOR_LIST_CHANGED:
        break;
    default:
    	printf("MEASUREMENT UPDATE HANDLER -> DEFAULT STATE!\n");
    }
}

static void services_init(void)
{
    uint32_t       err_code;
    ble_lbs_init_t init;

    init.measurement_update_handler = measurement_update_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    ESP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint8_t mac[] = {0x01, 0x02, 0x03, 0x04};
    manuf_info_t manuf_info;
    manuf_info.type = 0x01;
    memcpy(&manuf_info.mac, &mac, sizeof(mac));
    manuf_info.version = 0x01;
    manuf_info.tx_power = 0xC5;
}
#endif

static bool dwt_isr_in_progress = false;

static void IRAM_ATTR gpiote_event_handler(void *arg)
{
    switch((uint32_t) arg)
    {
    case DW1000_IRQ:
    	;
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    	break;
    case CHG_DET:
    		if(gpio_get_level(CHG_DET) == 1) {
    			gpio_set_level(CHG_LO, 0);
    			gpio_set_level(CHG_HI, 1);
    			xTimerStop(blinkTimer, 0);
    			gpio_set_level(GPIO_NUM_2, 1);
    			ets_printf("CHG LIKE HELL\n");	// FAST CHARGER
    		}
    		else {
    			gpio_set_level(CHG_LO, 1);
    			gpio_set_level(CHG_HI, 0);
    			gpio_set_level(GPIO_NUM_2, 0);
    			xTimerStart(blinkTimer, 0);
    			ets_printf("CHG SLOW");
    		}
    	break;
    default:
    	break;
    }
}

static void gpiote_init(void)
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = ( 1ULL << DW1000_IRQ);
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
	ESP_ERROR_CHECK(gpio_isr_handler_add(DW1000_IRQ, gpiote_event_handler, (void*) DW1000_IRQ));

	/* Faszt Charging */
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = ( 1ULL << CHG_DET);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	ESP_ERROR_CHECK(gpio_isr_handler_add(CHG_DET, gpiote_event_handler, (void*) CHG_DET));

	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = ( 1ULL << CHG_LO);
	gpio_config(&io_conf);

	io_conf.pin_bit_mask = ( 1ULL << CHG_HI);
	gpio_config(&io_conf);
}

/******************************************************************************************************************************************************/

#ifdef TAG_MODE
static void start_tag()
{
	xTaskToNotify = xTaskGetCurrentTaskHandle();

    timers_init();
    advertising_init();
    services_init();

    xTimerStart(blinkTimer, 0);

    printf("Start!\n");

    deca_twr_initiator(response_receive_handler, rtls_beacon_receive_handler);
    gpiote_init();

    printf("Power on Baby!\n");

    xTimerStart( xTimers[2], 0 );

    m_lbs.is_ranging_notification_enabled = true;
    if(m_lbs.measurement_update_handler != NULL)
        m_lbs.measurement_update_handler(&m_lbs, START_RANGING);

    for (;;)
    {
		//if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			if(gpio_get_level(DW1000_IRQ) == 1 && !dwt_isr_in_progress)
			{
				dwt_isr_in_progress = true;
	//			nrf_gpio_pin_set(LED_1);
				dwt_isr();

				while(deca_twr_poll_msg()){}

	//			nrf_gpio_pin_clear(LED_1);
				dwt_isr_in_progress = false;
			}
			//xSemaphoreGive(xSemaphore);
			//}
		while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {};
    }
}
#endif

#ifdef ANCHOR_MODE
static void start_anchor()
{
	xTaskToNotify = xTaskGetCurrentTaskHandle();
    printf("Start!\n\r");

    timers_init();

    xTimerStart(blinkTimer, 0);

    deca_twr_responder();
    gpiote_init();
    dwt_rxenable(0);

    xTimerStart(xTimers[1], 0);
    printf("ANCHOR TIMER INIT!\n");

    for (;;)
    {
    	//if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			if(gpio_get_level(DW1000_IRQ) == 1 && !dwt_isr_in_progress)
			{
				dwt_isr_in_progress = true;
	//			nrf_gpio_pin_set(LED_1);
				dwt_isr();

				while(deca_twr_poll_msg()){}

	//			nrf_gpio_pin_clear(LED_1);
				dwt_isr_in_progress = false;
			}
			//xSemaphoreGive(xSemaphore);
			//}
        while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {};
    }
}
#endif

void app_main()
{
	printf("START IT!\n");
	vTaskDelay(7000 / portTICK_PERIOD_MS);
	initShield();
#ifdef TAG_MODE
	setUpNanoAndConnections();
	connection_init();
    start_tag();
#endif
#ifdef ANCHOR_MODE
	start_anchor();
#endif
}
