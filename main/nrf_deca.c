

#include "nrf_deca.h"
#include "math.h"
#include "rtls.h"
#include "freertos/timers.h"
#include "deca_device_api.h"




typedef uint64_t uint64;

#define TX_RX_BUF_LENGTH				128
#define POLL_TX_TO_RESP_RX_DLY_UUS	140

//Frame control:
#define FRAME_CONTROL_RTLS_BEACON	0x85
#define PIN_NUM_MISO 22
#define PIN_NUM_MOSI 21
#define PIN_NUM_CLK 23
#define PIN_NUM_CS 19
#define SPI_PIN_NOT_USED -1
#define SPI_MAX_TRANSFER_SIZE 2048

#define SPI_CLOCK_SPEED_1M 1*1000*1000
#define SPI_CLOCK_SPEED_8M 8*1000*1000
#define SPI_CLOCK_SPEED_10M 10*1000*1000

#define SPI_MODE_0 0




#if (SPI0_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
#endif

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
		2,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		0x08,//DWT_PLEN_128,    /* Preamble length. */
		2,//DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		1,               /* Use non-standard SFD (Boolean) */
		0,//DWT_BR_6M8,      /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(1025+64-32)//(129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

//static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0};


#ifdef EVBOARD
	uint16_t address = 0xB0C1;
#else
	#ifdef DECA_ADDRESS
		uint16_t address = DECA_ADDRESS;
	#else
		uint16_t address = 0xBAC1;
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



uint16_t getTagAddress()
{
	address = address & 0x0FFF;
	return address;
}

void deca_sleep(unsigned int time_ms)
{
	vTaskDelay(time_ms/(0.1));//port_tick_ms
}

esp_err_t initDW1000()
{

    if( spi_bus_initialize(VSPI_HOST, spi_buscfg, SPI_DMA_CHANNEL) != ESP_OK)
	{
#if DEBUGMODE
    	printf("Error at iattach device spi bus!!!\n spi_set_rate_low() 005_2_1 \n");
#endif
	}
    //ESP_ERROR_CHECK(spi_err);
	if( spi_bus_add_device(VSPI_HOST, spi_devcfg, spi_dev) != ESP_OK)
	{
#if DEBUGMODE
		printf("Error at iattach device spi bus!!!\n spi_set_rate_low() 005_2_2 \n");
#endif
	}



    if(ESP_OK != gpio_set_direction(DW1000_RST, GPIO_MODE_INPUT))
    {
#if DEBUGMODE
    	printf("Nem sikerult a set direction (initDWM) 005_2_3 \n");
#endif
    }
    if( ESP_OK != gpio_pullup_dis(DW1000_RST))
    {
#if DEBUGMODE
        printf("Nem sikerult a DWM rst pullup disable!!! (initDWM) 005_2_4 \n");
#endif
    }

    if( ESP_OK != gpio_set_direction(DW1000_IRQ, GPIO_MODE_INPUT))
    {
#if DEBUGMODE
		printf("Nem sikerult a DWM set dir!!! (initDWM) 005_2_5 \n");
#endif
    }

    if( ESP_OK != gpio_pulldown_en(DW1000_IRQ))
	{
#if DEBUGMODE
		printf("Nem sikerult a DWM rst pulldown en!!! (initDWM) 005_2_6 \n");
#endif
	}


    //ESP_ERROR_CHECK(spi_err);
    return ESP_OK;
}
esp_err_t reset_DW1000(void)
{
    esp_err_t   rst_err;

    rst_err = gpio_set_direction(DW1000_RST, GPIO_MODE_OUTPUT);
    // There is no gpio clear function in the ESP library
    if( rst_err != ESP_OK)
    {
        printf("Error at set to OUTPUT the RST pin!!!\n reset_DW1000() func.\n\n");
        return rst_err;
    }
#if DEBUGMODE
    printf("Atjutott az elso setdirection-on \n");
#endif
    rst_err = gpio_set_level(DW1000_RST, 0);
#if DEBUGMODE
    printf("Atjutott az elso setlevel-en \n");
#endif

    vTaskDelay(2000 / portTICK_PERIOD_MS);  // 2 ms delay
#if DEBUGMODE
    printf("Atjutott az taskdelay-en \n");
#endif
    gpio_set_direction(DW1000_RST, GPIO_MODE_INPUT);
#if DEBUGMODE
    printf("Atjutott az masodik setdirection-on \n");
#endif
    rst_err = gpio_pullup_dis(DW1000_RST);
    if( rst_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at set the DW1000 RST pin!!!\n reset_DW1000() func.\n\n");
#endif
        return rst_err;
    }
#if DEBUGMODE
    printf("Atjutott a pullup disable-on \n");
#endif



    //ESP_ERROR_CHECK(rst_err);
    return rst_err;
}


esp_err_t spi_set_rate_low(void)
{

    esp_err_t           spi_err;

    spi_err = spi_bus_remove_device(*spi_dev);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at remove the device from the SPI bus!!!\n spi_set_rate_low() func.\n\n");
#endif
    }

    spi_bus_free(VSPI_HOST); // HSPI or VSPI ?? !!
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at setting free the spi bus!!!\n spi_set_rate_low() func.\n\n");
#endif
    }
    ESP_ERROR_CHECK(spi_err);

    spi_buscfg->miso_io_num    = PIN_NUM_MISO;
    spi_buscfg->mosi_io_num    = PIN_NUM_MOSI;
    spi_buscfg->sclk_io_num    = PIN_NUM_CLK;
    spi_buscfg->quadwp_io_num  = SPI_PIN_NOT_USED;     // Write Protect signal
    spi_buscfg->quadhd_io_num  = SPI_PIN_NOT_USED;     // HoID signal
    spi_buscfg->max_transfer_sz= SPI_MAX_TRANSFER_SIZE;

    spi_devcfg->clock_speed_hz = SPI_CLOCK_SPEED_1M;  //Clock out at 1 MHz
    spi_devcfg->mode           = SPI_MODE_0;         //SPI mode 0
    spi_devcfg->spics_io_num   = PIN_NUM_CS;         //CS pin
    spi_devcfg->queue_size     = SPI_QUEUE_SIZE;     //How many transaction able to queue at one time


    spi_err = spi_bus_initialize(VSPI_HOST,	spi_buscfg, SPI_DMA_CHANNEL);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at initialize spi bus!!!\n spi_set_rate_low() func.\n\n");
#endif
    }
    //Attach the device to the SPI bus
    spi_err = spi_bus_add_device(VSPI_HOST, spi_devcfg, spi_dev);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at iattach device spi bus!!!\n spi_set_rate_low() func.\n\n");
#endif
    }
    ESP_ERROR_CHECK(spi_err);
    return spi_err;
}

esp_err_t spi_set_rate_high(void)
{

    esp_err_t           spi_err;

    spi_err = spi_bus_remove_device(*spi_dev);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at remove the device from the SPI bus!!!\n spi_set_rate_high() func.\n\n");
#endif
    }

    spi_bus_free(VSPI_HOST); // HSPI or VSPI ?? !!
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at setting free the spi bus!!!\n spi_set_rate_high() func.\n\n");
#endif
    }
    ESP_ERROR_CHECK(spi_err);

    spi_buscfg->miso_io_num    = PIN_NUM_MISO;
    spi_buscfg->mosi_io_num    = PIN_NUM_MOSI;
    spi_buscfg->sclk_io_num    = PIN_NUM_CLK;
    spi_buscfg->quadwp_io_num  = SPI_PIN_NOT_USED;     // Write Protect signal
    spi_buscfg->quadhd_io_num  = SPI_PIN_NOT_USED;     // HoID signal
    spi_buscfg->max_transfer_sz= SPI_MAX_TRANSFER_SIZE;

    spi_devcfg->clock_speed_hz = SPI_CLOCK_SPEED_8M;  //Clock out at 8 MHz
    spi_devcfg->mode           = SPI_MODE_0;         //SPI mode 0
    spi_devcfg->spics_io_num   = PIN_NUM_CS;         //CS pin
    spi_devcfg->queue_size     = SPI_QUEUE_SIZE;     //How many transaction able to queue at one time


    spi_err = spi_bus_initialize(VSPI_HOST, spi_buscfg, SPI_DMA_CHANNEL);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at initialize spi bus!!!\n spi_set_rate_high() func.\n\n");
#endif
    }
    //Attach the device to the SPI bus
    spi_err = spi_bus_add_device(VSPI_HOST, spi_devcfg, spi_dev);
    if( spi_err != ESP_OK)
    {
#if DEBUGMODE
        printf("Error at iattach device spi bus!!!\n spi_set_rate_high() func.\n\n");
#endif
    }
    ESP_ERROR_CHECK(spi_err);
    return spi_err;
}

//decaIrqStatus_t decamutexon(void);
portMUX_TYPE decamutexon(void)
{
	portMUX_TYPE in;
	//uint8_t temp = 0;
	//sd_nvic_critical_region_enter(&temp);
	//taskENTER_CRITICAL(&in);
	//taskENTER_CRITICAL();
	taskDISABLE_INTERRUPTS();
	return in;

	//return temp;
}

//void decamutexoff(decaIrqStatus_t s)
void decamutexoff(portMUX_TYPE in)
{
	//sd_nvic_critical_region_exit(s);
	//taskEXIT_CRITICAL(&in);
	//taskEXIT_CRITICAL();
	taskENABLE_INTERRUPTS();
}



int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer)
{
	static spi_transaction_t trans;	//TODO QUEUE-ban küldjük, mert az nagyobb szopás
	uint8_t buff[bodylength + headerLength];
	for(int i = 0; i < headerLength + bodylength;i++)
		if(i < headerLength)
			buff[i] = headerBuffer[i];
		else
			buff[i] = bodyBuffer[i - headerLength];

	memset(&trans, 0, sizeof(spi_transaction_t));

	trans.length = (headerLength*8 + bodylength*8);
	trans.tx_buffer = buff;

	esp_err_t ret = spi_device_transmit(*spi_dev, &trans);
		if(ret != ESP_OK)
			return -1;

	printf("SPI WRITE\n");

	return 0;
}





int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer)
{

	static spi_transaction_t trans;


	memset(&trans, 0, sizeof(spi_transaction_t));


	trans.length = headerLength*8;
	trans.rxlength = readlength*8;
	trans.tx_buffer = headerBuffer;
	trans.rx_buffer = readBuffer;

	esp_err_t ret = spi_device_transmit(*spi_dev, &trans);
	if(ret != ESP_OK)
		return -1;



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
	#if DEBUGMODE
	printf("Handler1 atadasa 005_1 \n");
	#endif
	initiator_rtls_beacon_receive_handler = beacon_handler;
	#if DEBUGMODE
	printf("Handler2 atadasa 005_2 \n");
	#endif

	//Attach the device to the SPI bus

	if(ESP_OK != initDW1000())
	{
		#if DEBUGMODE
		printf("Megvolt az eszkoz init,nem sikerult 012 \n");
		#endif
	}
	uint8_t tmp[600];
	if(ESP_OK != dwt_spicswakeup(tmp, 600))
	{
#if DEBUGMODE
			printf("Megvolt a wakeup, nem sikerult 013 \n");
#endif
	}
	if(ESP_OK != reset_DW1000())
	{
#if DEBUGMODE
		printf("Megvolt a DW100 reset, nem sikerult 014 \n");
#endif
	}
	else
	{
#if DEBUGMODE
		printf("Megvolt a DW100 reset 014 \n");
#endif
	}
		vTaskDelay(2000 / portTICK_PERIOD_MS);


	if(ESP_OK != spi_set_rate_low())
	{
#if DEBUGMODE
			printf("Megvolt a setlow, nem sikerult 014 \n");
#endif
	}
	if( 0 != dwt_initialise(DWT_LOADUCODE))
	{
#if DEBUGMODE
		printf("Megvolt a dwt_initialise, nem sikerult 016 \n");
#endif
	}
	if(ESP_OK != spi_set_rate_high())
	{
#if DEBUGMODE
		printf("Megvolt a sethigh, nem sikerult 014 \n");
#endif
	}
	else
	{
#if DEBUGMODE
		printf("Megvolt a spi_set_rate_high 017 \n");
#endif
	}
	int init_result;
		init_result = dwt_configure(&config);
#if DEBUGMODE
	printf("Megvolt a dwt_configure, %d 018 \n",init_result);
#endif


	dwt_setrxmode(DWT_RX_SNIFF, 2, 24);
#if DEBUGMODE
	printf("Megvolt a dwt_setrxmode 019 \n");
#endif


	uint32_t devid = (uint32_t)dwt_readdevid();
#if DEBUGMODE
	printf("A devid : %X \n",devid);
#endif



	if(init_result == DWT_ERROR)
	{
#if DEBUGMODE
		printf("Error on dwt_initialise! \n\r");
#endif
		return 1;
	}

	printf("Configured, address, jon a villogas :  %x \n",address);

    dwt_setleds(3);
#if DEBUGMODE
    printf("Morzsa 1 \n");
#endif
	dwt_setrxantennadelay(0);
#if DEBUGMODE
	printf("Morzsa 2 \n");
#endif
	dwt_settxantennadelay(0);
#if DEBUGMODE
	printf("Morzsa 3 \n");
#endif

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
#if DEBUGMODE
	printf("Morzsa 4 \n");
#endif
	dwt_setrxtimeout(0);
#if DEBUGMODE
	printf("Morzsa 5 \n");
#endif

	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);
#if DEBUGMODE
	printf("Morzsa 6 \n");
#endif

	int sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
#if DEBUGMODE
	printf("A SYSCFG : %x \n",sys_cfg);
#endif
	//simple_uart_puthex((sys_cfg>>24)&0xFF);
	//simple_uart_puthex((sys_cfg>>16)&0xFF);
	//simple_uart_puthex((sys_cfg>>8)&0xFF);
	//simple_uart_puthex((sys_cfg)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");
	printf("Morzsa 7 \n");
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	//int sys_mask = dwt_read32bitreg(SYS_MASK_ID);
	//simple_uart_puthex((sys_mask>>24)&0xFF);
	//simple_uart_puthex((sys_mask>>16)&0xFF);
	//simple_uart_puthex((sys_mask>>8)&0xFF);
	//simple_uart_puthex((sys_mask)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");
	printf("Morzsa 8 \n");
	mac_init(address);
	printf("Morzsa 9 \n");
	dwt_setcallbacks(mac_txcallback_impl, mac_rxcallback_impl);
	printf("Morzsa 10 \n");
	dwt_txconfig_t configTx;
	configTx.PGdly = 0x95;
	configTx.power = 0x1A140E2E;
	dwt_configuretxrf(&configTx);
	printf("Eljutott az initiator vegere \n");

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
	address = address & 0x00FF;
	address = address | 0xFE00;

	initDW1000();
	reset_DW1000();

	spi_set_rate_low();
	int init_result = dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&config);

	//uint32 devid = dwt_readdevid();
	//simple_uart_puthex((devid>>24)&0xFF);
	//simple_uart_puthex((devid>>16)&0xFF);
	//simple_uart_puthex((devid>>8)&0xFF);
	//simple_uart_puthex((devid)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");

	if(init_result == DWT_ERROR)
	{
		printf("Error on dwt_initialise! \n\r");
		return 1;
	}

	printf("Configured \n");
	//simple_uart_putstring((const uint8_t *)"Configured: ");
	//simple_uart_puthex((address>>8)&0xFF);
	//simple_uart_puthex((address)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");

    dwt_setleds(3);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(0);

//	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);

	//int sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
	//simple_uart_puthex((sys_cfg>>24)&0xFF);
	//simple_uart_puthex((sys_cfg>>16)&0xFF);
	//simple_uart_puthex((sys_cfg>>8)&0xFF);
	//simple_uart_puthex((sys_cfg)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	//int sys_mask = dwt_read32bitreg(SYS_MASK_ID);
	//simple_uart_puthex((sys_mask>>24)&0xFF);
	//simple_uart_puthex((sys_mask>>16)&0xFF);
	//simple_uart_puthex((sys_mask>>8)&0xFF);
	//simple_uart_puthex((sys_mask)&0xFF);
	//simple_uart_putstring((const uint8_t *)"\n\r");

	mac_init(address);
	dwt_setcallbacks(mac_txcallback_impl, mac_rxcallback_impl);

//	uint8_t t = 0x95;
//	dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, &t);

	dwt_txconfig_t configTx;
	configTx.PGdly = 0x95;
	configTx.power = 0x1A140E2E;
	dwt_configuretxrf(&configTx);

/*
	dwt_configcontinuousframemode(21300);

	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(rtls_beacon_msg), rtls_beacon_msg, 0);
	dwt_writetxfctrl(sizeof(rtls_beacon_msg), 0);

	dwt_starttx(DWT_START_TX_IMMEDIATE );
*/
	return 0;
}






































