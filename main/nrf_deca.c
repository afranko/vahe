
#include <math.h>
#include "nrf_deca.h"
#include "commons.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_heap_trace.h"


typedef uint64_t uint64;

#define TX_RX_BUF_LENGTH				128
#define POLL_TX_TO_RESP_RX_DLY_UUS	140

//Frame control:
#define FRAME_CONTROL_RTLS_BEACON	0x85

#ifdef TAG_MODE
extern uint8_t err_cnter;	//TODO + NOTE
#endif



/* SEMAFOR CITY */

SemaphoreHandle_t xSemaphore = NULL;	//MUTEX



void initShield(void) {
	xSemaphore = xSemaphoreCreateMutex();
	if(xSemaphore == NULL) {
		ets_printf("Szemafor malac!\n");
	}
}

/* END SEMAFORCITY */

/* DEBUGCITY*/

static uint64_t fakeMutexForWrite = 0;
static uint64_t fakeMutexForRead = 0;

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
dwt_config_t config = {
		4,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		DWT_PLEN_128,    /* Preamble length. */
		DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
		18,               /* TX preamble code. Used in TX only. */
		18,               /* RX preamble code. Used in RX only. */
		0,               /* Use non-standard SFD (Boolean) */
		DWT_BR_6M8,      /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

spi_bus_config_t buscfg={
		.miso_io_num=DW1000_MISO_PIN,
		.mosi_io_num=DW1000_MOSI_PIN,
		.sclk_io_num=DW1000_SCK_PIN,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=2047	//TODO+NOTE
		};

spi_device_interface_config_t devcfg={
	    .clock_speed_hz = 1*1000*1000,           //Clock out at 1 MHz
		.mode = 0,                                //SPI mode 0
	    .spics_io_num = -1,               //CS pi
		.queue_size = 15,          //TODO+NOTE
		//.flags = SPI_DEVICE_HALFDUPLEX,			//TODO+NOTE
		};


spi_device_handle_t spi;

#ifdef EVBOARD
	uint16_t address = 0xB0C1;
#else
	#ifdef CONFIG_DECA_ADDRESS
		uint16_t address = CONFIG_DECA_ADDRESS;
	#else
		uint16_t address = 0xCEF6;
	#endif
#endif

uint64_t dwm1000_get_system_time_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64_t dwm1000_get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint32_t spi_master_init(void)
{
	if(spi_bus_initialize(VSPI_HOST, &buscfg, 0) == ESP_OK) {
		return spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
	}
	else
		return ESP_FAIL;
}

uint16_t getTagAddress()
{
	address = address & 0x0FFF;
	return address;
}

void deca_sleep(unsigned int time_ms)
{
	vTaskDelay(time_ms / portTICK_PERIOD_MS);
}

void initDW1000(void)
{
	uint32_t err_code = spi_master_init();
	ESP_ERROR_CHECK(err_code);

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << DW1000_RST);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	io_conf.pin_bit_mask = (1ULL << DW1000_SS_PIN);
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
}

void reset_DW1000(void)
{
	gpio_set_level(DW1000_RST, 0);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	gpio_set_direction(DW1000_RST, GPIO_MODE_INPUT);
}

void spi_set_rate_low(void)
{
	spi_bus_remove_device(spi);

	devcfg.clock_speed_hz=3*1000*1000;

	spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
}

void spi_set_rate_high(void)
{
	spi_bus_remove_device(spi);

	devcfg.clock_speed_hz=20*1000*1000;

	spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
}


decaIrqStatus_t decamutexon(void)
{
	//portENTER_CRITICAL(&mux);
	//return mux.count;
	return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
	//portEXIT_CRITICAL(&mux);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)	//TODO m�g mindig magic MUX, printf???
{
	//uint8_t irqs = decamutexon();

	if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {

	if(fakeMutexForWrite++ > 0)			//TODO+NOTE
		ets_printf("NOTIF: %" PRIu64 "WRITE!\n", fakeMutexForWrite);

	//portDISABLE_INTERRUPTS();
	gpio_set_level(DW1000_SS_PIN, 0);

	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	memset(&trans[1], 0, sizeof(spi_transaction_t));

	trans[0].length = headerLength*8;
	trans[0].tx_buffer = headerBuffer;

	trans[1].length = bodylength*8;
	trans[1].tx_buffer = bodyBuffer;

	spi_transaction_t *ret_trans;

	esp_err_t error_c = spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIW PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	error_c = spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIW PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	//assert(ret_trans == &trans[0]);

	error_c = spi_device_queue_trans(spi, &trans[1], portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIW PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}


	spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);

	if(error_c != ESP_OK) {
		ets_printf("SPIW PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	//assert(ret_trans == &trans[1]);

	/*spi_device_transmit(spi, &trans[0]);
	spi_device_transmit(spi, &trans[1]);*/


	//decamutexoff(irqs);
	gpio_set_level(DW1000_SS_PIN, 1);

	//portENTER_CRITICAL(&mux);

	--fakeMutexForWrite;

	xSemaphoreGive(xSemaphore);
	}

	return 0;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	//portENTER_CRITICAL(&mux);

	if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {

	if(fakeMutexForRead++ > 0)
		ets_printf("NOTIF: %" PRIu64 "SPIREAD\n", fakeMutexForRead);

	gpio_set_level(DW1000_SS_PIN, 0);
	//uint8_t irqs = decamutexon();
	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	memset(&trans[1], 0, sizeof(spi_transaction_t));

	trans[0].length = headerLength*8;
	trans[1].length = readlength*8;
	trans[0].tx_buffer = headerBuffer;
	trans[1].rx_buffer = readBuffer;

	/*spi_device_transmit(spi, &trans[0]);
	spi_device_transmit(spi, &trans[1]);*/

	spi_transaction_t *ret_trans;
	esp_err_t error_c = spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIR PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	error_c = spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIR PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	//assert(ret_trans == &trans[0]);

	error_c = spi_device_queue_trans(spi, &trans[1], portMAX_DELAY);
	if(error_c != ESP_OK) {
		ets_printf("SPIR PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);

	if(error_c != ESP_OK) {
		ets_printf("SPIR PROBLEM: %s", esp_err_to_name(error_c));
		--fakeMutexForRead;
		return -1;
	}

	//assert(ret_trans == &trans[1]);

	//decamutexoff(irqs);
	gpio_set_level(DW1000_SS_PIN, 1);

	--fakeMutexForRead;

	//portEXIT_CRITICAL(&mux);
	xSemaphoreGive(xSemaphore);
	}

	return 0;
}

/**************************************************************************************************************************************************************************************/

response_receive_handler_t initiator_response_receive_handler = NULL;
rtls_beacon_receive_handler_t initiator_rtls_beacon_receive_handler = NULL;

void deca_twr_initiator_send_poll(uint16_t dst_address)
{
	mac_start_ranging(dst_address);
}

void deca_twr_initiator_listen_to_beacon()
{
	dwt_rxenable(0);
}

bool deca_twr_poll_msg()
{
	mac_packet_info_t* packet_info = mac_receive_poll();

	if(packet_info != NULL)
	{
		if((packet_info->packet[0] & 0xF0) == MAC_FRAME_TYPE_RANGING)
		{
			if((packet_info->packet[0] & 0x0F) == MAC_FRAME_ST_DIST)
			{
				rtls_dist_msg_t* dist_msg = (rtls_dist_msg_t*)packet_info->packet;
#ifdef TAG_MODE
				if(err_cnter > 0) {
				ets_printf("MALACOK AZ URBEN: VOLT ERTELME\n");
				}
				err_cnter = 0;	//TODO + NOTE!
#endif

				if(initiator_response_receive_handler != NULL)
					initiator_response_receive_handler(true, packet_info->src_addr, dist_msg->dist_cm,
					 dist_msg->rx_quality.rx_nlos_count); //NOTE+TODO
			}
		}
		else if((packet_info->packet[0] & 0xF0) == MAC_FRAME_TYPE_BEACON)
		{
            mac_beacon_package_format_t* beacon_msg = (mac_beacon_package_format_t*)packet_info->packet;

			if(initiator_rtls_beacon_receive_handler != NULL)
                initiator_rtls_beacon_receive_handler(packet_info->src_addr, beacon_msg->hop_addr, beacon_msg->hop_count);
		}

		mac_free_buffer(packet_info);

		return true;
	}
	else
	{
		return false;
	}
}

void deca_twr_rxtimeout()
{
	dwt_forcetrxoff();
}

void deca_twr_configure()
{
	dwt_configure(&config);
}

int deca_twr_initiator(response_receive_handler_t handler, rtls_beacon_receive_handler_t beacon_handler)
{
	address = address & 0x0FFF;

	initiator_response_receive_handler = handler;
	initiator_rtls_beacon_receive_handler = beacon_handler;

	initDW1000();
	reset_DW1000();

	spi_set_rate_low();
	int init_result = dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();
	dwt_configure(&config);

	dwt_setrxmode(DWT_RX_SNIFF, 2, 24);

	if(init_result == DWT_ERROR)
	{
		printf("Error on dwt_initialise!\n\r");
		return 1;
	}

    dwt_setleds(3);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(0);

//	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	mac_init(address);
	dwt_setcallbacks(mac_txcallback_impl, mac_rxcallback_impl);

	dwt_txconfig_t configTx;
	configTx.PGdly = 0x95;
	configTx.power = 0x1A140E2E;
	dwt_configuretxrf(&configTx);	// TODO+ NOTE 0x5F5F5F5F vagy 0x9A9A9A9A

	return 0;
}

/**************************************************************************************************************************************************************************************/

void deca_twr_responder_send_rtls_beacon()
{
	mac_transmit_beacon();
	mac_transmit_poll();
}

int deca_twr_responder(void)
{
	//address = address & 0x00FF;
	//address = address | 0xFE00;

	initDW1000();
	reset_DW1000();

	spi_set_rate_low();
	int init_result = dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&config);

	if(init_result == DWT_ERROR)
	{
		printf("Error on dwt_initialise!\n\r");
		return 1;
	}

    dwt_setleds(3);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(0);

//	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	mac_init(address);
	dwt_setcallbacks(mac_txcallback_impl, mac_rxcallback_impl);

//	uint8_t t = 0x95;
//	dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, &t);

	dwt_txconfig_t configTx;
	configTx.PGdly = 0x95;
	configTx.power = 0x1A140E2E;
	dwt_configuretxrf(&configTx);	//TODO + NOTE

/*
	dwt_configcontinuousframemode(21300);

	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(rtls_beacon_msg), rtls_beacon_msg, 0);
	dwt_writetxfctrl(sizeof(rtls_beacon_msg), 0);

	dwt_starttx(DWT_START_TX_IMMEDIATE );
*/

	return 0;
}
