
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
//#include "driver/uart.h"
#include "freertos/timers.h"
#include "dw_tag.h"
#include "nrf_deca.h"


#define IRQ_LVL 1
#define SPI_MODE_0 0
uint32_t ulNotifiedValue = 0;
BaseType_t xHigherPriorityTaskWoken;
// Hardware constants for DECAWAVE 1000 pins

//#define DW1000_SS_PIN
//#define DW1000_SS_LED


void running_tag (void *pvParameter);


// descriptive structures and belonging variablesmeasurement_update_handler
esp_err_t           ret_spi_err;







// This section is has to be write into the initDW1000() function
// Initialize the SPI bus


// ================================


const char intArray[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

//static uint16_t                          	 	m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
//static ble_bas_t                              	bas;                                       /**< Structure used to identify the battery service. */
//static dm_application_instance_t                m_app_handle;                              /**< Application identifier allocated by device manager */



//static StaticTimer_t measure_timer;
//static StaticTimer_t ts_timer;
//static StaticTimer_t rxtimer;

int isDebugModeEnabled = 1;

//<Marci>
//define UART_NUM  UART_NUM_2
//#define UART_PIN_TX  1//BEFEJEZNI
//#define UART_PIN_RX  3//BEFEJEZNI
//#define UART_PIN_RST 18
//#define UART_PIN_CTS 19

#define TS_TICKNUM 100
#define MEASURE_TICKNUM 100
#define RX_TICKNUM 100
#define B_TICKNUM 100
#define DWM_1000_IRQ_PIN 18


// </Marci>
//#define USE_UART


void vTimerCallback( TimerHandle_t pxTimer )
 {
 const uint32_t ulMaxExpiryCountBeforeStopping = 10;
 uint32_t ulCount = 0;

    /* Optionally do something if the pxTimer parameter is NULL. */
    if(pxTimer == 0)
    {
	#if DEBUGMODE
    	printf("NULL vektort kapott a kib***ott vTimerCallback \n");
	#endif
    }

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( pxTimer );

    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    ulCount++;

    /* If the timer has expired 10 times then stop it from running. */
    if( ulCount >= ulMaxExpiryCountBeforeStopping )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        xTimerStop( pxTimer, 0 );
    }
    else
    {
       /* Store the incremented count back into the timer's ID field
       so it can be read back again the next time this software timer
       expires. */
       vTimerSetTimerID( pxTimer, ( void * ) ulCount );
    }
 }

static void timers_init(TimerHandle_t* timerhandle)//M
{   timerhandle[0] = xTimerCreate("T0",	MEASURE_TICKNUM/portTICK_PERIOD_MS ,	pdTRUE,	measurement_timeout_handler,	vTimerCallback);//&measure_timer);
	#if DEBUGMODE
		printf("Megvolt a measurement_handler timer inicializacioja 004_1 \n");
	#endif
	timerhandle[1] = xTimerCreate("T2", TS_TICKNUM/portTICK_PERIOD_MS , 		pdTRUE,	ts_timeout_handler,				vTimerCallback);//&ts_timer);
	#if DEBUGMODE
		printf("Megvolt a ts_timeout_handler timer inicializacioja 004_2 \n");
	#endif
	timerhandle[2] = xTimerCreate("T3", RX_TICKNUM/portTICK_PERIOD_MS , 		pdTRUE,	rxtimeout_timeout_handler,		vTimerCallback);//&rxtimer);
	#if DEBUGMODE
		printf("Megvolt az rx_timeout_handler timer inicializacioja 004_3 \n");
	#endif
	//timerhandle[3] = xTimerCreate("T3", B_TICKNUM/portTICK_PERIOD_MS , 		pdTRUE,	rxtimeout_timeout_handler,		vTimerCallback);//&rxtimer);

}


void xTimerStart_manual(TimerHandle_t timerhandle)
{
	xTimerStart(timerhandle,TS_TICKNUM);
#if DEBUGMODE
	printf("Starting rxtimeout_timeout_handler \n");
#endif
}

void xTimerStop_manual(TimerHandle_t timerhandle)
{
	xTimerStop(timerhandle,TS_TICKNUM);
#if DEBUGMODE
	printf("Starting measurement_timeout_handler \n");
#endif
}


/*
void xTimerStart_measurement(void)
{
	xTimerStart(measurement_timeout_handler,TS_TICKNUM);
	printf("Starting measurement_timeout_handler \n");
}

void xTimerStart_ts_timeout(void)
{
	xTimerStart(ts_timeout_handler,TS_TICKNUM);
	printf("Starting ts_timeout_handler \n");
}
void xTimerStart_rxtimeout(void)
{
	xTimerStart(rxtimeout_timeout_handler,TS_TICKNUM);
	printf("Starting rxtimeout_timeout_handler \n");
}

void xTimerStop_measurement(void)
{
	xTimerStop(measurement_timeout_handler,TS_TICKNUM);
	printf("Starting measurement_timeout_handler \n");
}

void xTimerStop_ts_timeout(void)
{
	xTimerStop(ts_timeout_handler,TS_TICKNUM);
	printf("Starting ts_timeout_handler \n");
}

void xTimerStop_rxtimeout(void)
{
	xTimerStop(rxtimeout_timeout_handler,TS_TICKNUM);
	printf("Starting rxtimeout_timeout_handler \n");
}*/

typedef struct
{
    uint8_t type;       // device type, 0x01 => Anchor, 0x02 => Tag //ezt kene atirni ugy, hogy csak TAG uzemmod legyen (most)
    uint8_t mac[4];     // UWB MAC address (ID)
    uint8_t version;
    uint8_t tx_power;   // The 2's complement of the calibrated Tx Power  (<RSSI @ 1m>)
} manuf_info_t;



//static bool m_measurement_timer_on = false;
//static bool m_mpu_timer_on = false; //nem kell

/*
static void measurement_update_handler( uint8_t new_state) //portolni
{
    uint32_t err_code;

    switch(new_state)
    {
    case 	START_RANGING:
        if(!m_measurement_timer_on)
        {
			      nrf_gpio_pin_set(LED_0);
            err_code = app_timer_start(m_measurement_timer_id, RANGING_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            m_measurement_timer_on = true;
        }
        break;

    case	STOP_RANGING:
        if(1)
        {
			       nrf_gpio_pin_clear(LED_0);
            err_code = app_timer_stop(m_measurement_timer_id);
            APP_ERROR_CHECK(err_code);
            m_measurement_timer_on = false;
        }
        break;

    case 	ANCHOR_LIST_CHANGED:
        break;
    }
}
*/




//UART init for ESP
/*
static void uart_init(void)//M
{


  const int uart_num = UART_NUM;
  uart_config_t uart_config =
  {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
      .rx_flow_ctrl_thresh = 122,
  };
  const int     uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
#ifdef DEBUGMODE

      ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
      ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_RST, UART_PIN_CTS)); *//* a pinek helyett lehetett volna UART_PIN_NO CHANGE-t hasznalni, es akkor a default marad */
     /* ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

#else
  {
      uart_param_config(uart_num, &uart_config);
      uart_set_pin(UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_RST, UART_PIN_CTS);
      uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
  }
#endif
}
*/

static bool dwt_isr_in_progress = false;

static void gpiote_event_handler(void* arg)
{

	 xHigherPriorityTaskWoken = 0;
        while((gpio_get_level(DWM_1000_IRQ_PIN)) == IRQ_LVL)
        {
            dwt_isr_in_progress = true;
            dwt_isr();

            while(deca_twr_poll_msg()){}
            dwt_isr_in_progress = false;
        }
        xHigherPriorityTaskWoken = pdFALSE;


        xTaskNotifyFromISR( running_tag,
                                        0,
                                        eNoAction,
										&xHigherPriorityTaskWoken);

}

static void gpiote_init(void)//USE
{
    int err_code;

    gpio_config_t gpio_config_dwm1000irq =
    {
    	    .pin_bit_mask 	= (1ULL << DWM_1000_IRQ_PIN),
    	    .mode 			= GPIO_MODE_OUTPUT,
    	    .pull_up_en 	= GPIO_PULLUP_DISABLE ,
    	    .pull_down_en 	= GPIO_PULLDOWN_ENABLE ,
    	    .intr_type 		= GPIO_INTR_DISABLE
    };

    err_code = gpio_config(&gpio_config_dwm1000irq);//lehet hogy ezeket RTC uzemmodban kell csinalni (low power, vagy deep sleep allapot)
	#if DEBUGMODE
    printf("GPIO config megvolt 006_1 \n");
	#endif
    if( (err_code != ESP_OK) & (isDebugModeEnabled) )
    {
		#if DEBUGMODE
    	printf("GPIO konfig nem sikerult 006_1_1 \n");
		#endif
    }

	gpio_config_dwm1000irq.mode 			= GPIO_MODE_INPUT;
	gpio_config_dwm1000irq.pull_up_en 		= GPIO_PULLUP_ENABLE;
	gpio_config_dwm1000irq.pull_down_en 	= GPIO_PULLDOWN_DISABLE;
	gpio_config_dwm1000irq.intr_type 		= GPIO_INTR_HIGH_LEVEL; //lehet hogy LOW_LEVEL

	err_code = gpio_config(&gpio_config_dwm1000irq);//lehet hogy ezeket RTC uzemmodban kell csinalni (low power, vagy deep sleep allapot)
	#if DEBUGMODE
	printf("GPIO config megvolt 006_2 \n");
	#endif
	if( (err_code != ESP_OK) & (isDebugModeEnabled) )
	{
		#if DEBUGMODE
		printf("GPIO konfig nem sikerult 006_2_1 \n");
		#endif
	}



	if( gpio_install_isr_service(ESP_INTR_FLAG_LOWMED) != ESP_OK )
	{
    	#if DEBUGMODE
    	printf("GPIO install nem sikerult 006_3 \n");
		#endif
	}
    else
    {
		#if DEBUGMODE
    	printf("GPIO install sikerult006_4 \n");
		#endif
    }

    err_code = gpio_isr_handler_add(DWM_1000_IRQ_PIN,gpiote_event_handler,NULL);
	#if DEBUGMODE
    printf("GPIO isr handle hozzaadasa megvolt 006_5 \n");
	#endif
    if( (err_code != ESP_OK) & (isDebugModeEnabled) )
    {
		#if DEBUGMODE
    	printf("GPIO konfig nem sikerult 006_6 \n");
		#endif
    }
}


/******************************************************************************************************************************************************/


static void start_tag(TimerHandle_t* timerhandle)
{

    //app_trace_init();
	spi_device_interface_config_t devcfg_config_local = {
		.clock_speed_hz = SPI_CLOCK_SPEED_10M,//Clock out at 10 MHz
		.mode           = SPI_MODE_0,         //SPI mode 0
		.spics_io_num   = PIN_NUM_CS,         //CS pin
		.queue_size     = SPI_QUEUE_SIZE,     //How many transaction able to queue at one time
		.flags = SPI_DEVICE_HALFDUPLEX
	};
	spi_bus_config_t    buscfg_config_local=
	{
		.miso_io_num    = PIN_NUM_MISO,
		.mosi_io_num    = PIN_NUM_MOSI,
		.sclk_io_num    = PIN_NUM_CLK,
		.quadwp_io_num  = SPI_PIN_NOT_USED,     // Write Protect signal
		.quadhd_io_num  = SPI_PIN_NOT_USED,     // HoID signal
		.max_transfer_sz= SPI_MAX_TRANSFER_SIZE


	};
		spi_device_handle_t dev_cfg_local;
		spi_buscfg 	= &buscfg_config_local;
		spi_devcfg 	= &devcfg_config_local;
		spi_dev 	= &dev_cfg_local;

    timers_init(timerhandle);
	#if DEBUGMODE
    printf("Start! 005_0\n");//ha ez a decawave-nek szol, akkor vissza kell irni simple_uart_putstring-re
	#endif
    int a = deca_twr_initiator(response_receive_handler, rtls_beacon_receive_handler);
	#if DEBUGMODE
    printf("Deca twr inicializacio meg volt, %d 006 \n",a);
	#endif
    gpiote_init();
	#if DEBUGMODE
    printf("Gpiote_init meg volt 007 \n");
	#endif
    xTimerStart_manual(timerhandle[1]);
	#if DEBUGMODE
    printf("timerhandle start meg volt 008 \n");
	#endif
    //uint32_t buff = dwt_read32bitreg(0x09);
    //printf("A beolvasott ertek a 0x01 cimrol : %x \n",buff);
    //buff++;
    //printf("A kiirando ertek a %x \n", buff);
    //dwt_write32bitreg(0x09,buff);
    //printf("A beolvasott ertek a 0x01 cimrol : %x \n",(uint32_t)dwt_read32bitreg(0x09));
    //buff--;
    //dwt_write32bitreg(0x09,buff);




/*
    for (int i = 0;;i++)
    {
    	//if(i%100 == 1)
    		//printf("For ciklusba belepett 008 \n");
    	//printf("%d",gpio_get_level(DWM_1000_IRQ_PIN));
        if((gpio_get_level(DWM_1000_IRQ_PIN) == 1 && !dwt_isr_in_progress) )//|| (i%100 == 0))//M portolni M
        {
            dwt_isr_in_progress = true;
#if DEBUGMODE
            if(dwt_isr_in_progress == true)
            	printf("A dwt_isr_in_progress erteke true 	009_0_1 \n");
            else
            	printf("A dwt_isr_in_progress erteke false 	009_0_2 \n");
#endif
            dwt_isr();
#if DEBUGMODE
            printf("dwt_isr lement 009_0_3 \n");
#endif
            while(deca_twr_poll_msg()){}
            dwt_isr_in_progress = false;
            configUSE_TASK_NOTIFICATIONS
            if(dwt_isr_in_progress == true)
            	printf("A dwt_isr_in_progress erteke true 	009_0_4 \n");
            else
            	printf("A dwt_isr_in_progress erteke false 	009_0_5 \n");

        }

    }
*/
}


void running_tag (void *pvParameter)
{

	uint32_t ulNotifiedValue = 0;
#if DEBUGMODE
		if("running tag task running 008 \n ");

#endif
	    for (int i = 0;;i++)
	    {
	    	//if(i%100 == 1)
	    		//printf("For ciklusba belepett 008 \n");
	    	//printf("%d",gpio_get_level(DWM_1000_IRQ_PIN));
	        if((gpio_get_level(DWM_1000_IRQ_PIN) == IRQ_LVL && !dwt_isr_in_progress) )//|| (i%100 == 0))//M portolni M
	        {
	            dwt_isr_in_progress = true;
	#if DEBUGMODE
	            if(dwt_isr_in_progress == true)
	            	printf("A dwt_isr_in_progress erteke true 	009_0_1 \n");
	            else
	            	printf("A dwt_isr_in_progress erteke false 	009_0_2 \n");
	#endif
	            dwt_isr();
	#if DEBUGMODE
	            printf("dwt_isr lement 009_0_3 \n");
	#endif
	            while(deca_twr_poll_msg()){}
	            dwt_isr_in_progress = false;
	            if(dwt_isr_in_progress == true)
	            	printf("A dwt_isr_in_progress erteke true 	009_0_4 \n");
	            else
	            	printf("A dwt_isr_in_progress erteke false 	009_0_5 \n");

	        }

	        //ide kerul a wait
	        xTaskNotifyWait( 				0x00,      /* Don't clear any notification bits on entry. */
                    						ULONG_MAX, /* Reset the notification value to 0 on exit. */
											&ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
											portMAX_DELAY );

	    }
}

void app_main()   //NOTE+TODO
{
	for(int i = 5; i > 0; i--)
	{
		#if DEBUGMODE
		printf("Inditasig meg %d masodperc... \n",i);
		#endif
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	#if DEBUGMODE
	printf("Indulas... \n \n \n");
	#endif
	vTaskDelay(1000 / portTICK_PERIOD_MS);


	vTaskDelay(50 / portTICK_PERIOD_MS);
	#if DEBUGMODE
	printf("Megvolt a taskdelay 001 \n");
	#endif
	gpio_set_direction(DWM_1000_IRQ_PIN, GPIO_MODE_OUTPUT);
	#if DEBUGMODE
	printf("Megvolt a setdirection 002 \n");
	#endif
	//uart_init();//portolni pipa


	#if DEBUGMODE
    printf("Device: DECA_BT TAG 003 \n");
	#endif
    TimerHandle_t tim[3];//at lesz adva a timerhandlers pointernek
	#if DEBUGMODE
    printf("Itt lett letrehozva a timer valtozo 003_1 \n");
	#endif
    timerhandlers = tim;
	#if DEBUGMODE
    printf("Itt lett atadva a timer valtozo 004 \n");
	#endif
    start_tag(timerhandlers);
    xTaskCreate(&running_tag, "runnning_tag", 2048, NULL, 5, NULL);


}
