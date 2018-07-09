
#include <math.h>
#include "nrf_drv_gpiote.h"
#include "nrf_deca.h"


typedef uint64_t uint64;

#define TX_RX_BUF_LENGTH				128
#define POLL_TX_TO_RESP_RX_DLY_UUS	140

//Frame control:
#define FRAME_CONTROL_RTLS_BEACON	0x85

#if (SPI0_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
#endif

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
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

static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0};


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

static uint32_t spi_master_init(void)
{
	nrf_drv_spi_config_t config =
	{
			.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
			.irq_priority = APP_IRQ_PRIORITY_LOW,
			.orc          = 0xCC,
			.frequency    = NRF_DRV_SPI_FREQ_1M,
			.mode         = NRF_DRV_SPI_MODE_0,
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
	};
	config.sck_pin  = DW1000_SCK_PIN;
	config.mosi_pin = DW1000_MOSI_PIN;
	config.miso_pin = DW1000_MISO_PIN;

	//	spi_config.SPI_Pin_SS = DW1000_SS_LED;

	return nrf_drv_spi_init(&m_spi_master_0, &config, NULL);
}

uint16_t getTagAddress()
{
	address = address & 0x0FFF;
	return address;
}

void deca_sleep(unsigned int time_ms)
{
	nrf_delay_ms(time_ms);
}

void initDW1000(void)
{
	uint32_t err_code = spi_master_init();
	APP_ERROR_CHECK(err_code);

	nrf_gpio_cfg_output(DW1000_SS_PIN);
	nrf_gpio_pin_set(DW1000_SS_PIN);

	nrf_gpio_cfg_input(DW1000_RST, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_PULLDOWN);
}

void reset_DW1000(void)
{
	nrf_gpio_cfg_output(DW1000_RST);
	nrf_gpio_pin_clear(DW1000_RST);
	nrf_delay_ms(2);
	nrf_gpio_cfg_input(DW1000_RST, NRF_GPIO_PIN_NOPULL);
}

void spi_set_rate_low(void)
{
	nrf_drv_spi_uninit(&m_spi_master_0);

	nrf_drv_spi_config_t config =
	{
			.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
			.irq_priority = APP_IRQ_PRIORITY_LOW,
			.orc          = 0xCC,
			.frequency    = NRF_DRV_SPI_FREQ_1M,
			.mode         = NRF_DRV_SPI_MODE_0,
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
	};
	config.sck_pin  = DW1000_SCK_PIN;
	config.mosi_pin = DW1000_MOSI_PIN;
	config.miso_pin = DW1000_MISO_PIN;

	nrf_drv_spi_init(&m_spi_master_0, &config, NULL);
}

void spi_set_rate_high(void)
{
	nrf_drv_spi_uninit(&m_spi_master_0);

	nrf_drv_spi_config_t config =
	{
			.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
			.irq_priority = APP_IRQ_PRIORITY_LOW,
			.orc          = 0xCC,
			.frequency    = NRF_DRV_SPI_FREQ_8M,
			.mode         = NRF_DRV_SPI_MODE_0,
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
	};
	config.sck_pin  = DW1000_SCK_PIN;
	config.mosi_pin = DW1000_MOSI_PIN;
	config.miso_pin = DW1000_MISO_PIN;

	nrf_drv_spi_init(&m_spi_master_0, &config, NULL);
}

decaIrqStatus_t decamutexon(void)
{
	uint8_t temp = 0;
	sd_nvic_critical_region_enter(&temp);

	return temp;
}

void decamutexoff(decaIrqStatus_t s)
{
	sd_nvic_critical_region_exit(s);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	uint8_t irqs = decamutexon();
	nrf_gpio_pin_clear(DW1000_SS_PIN);

	uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master_0, (uint8 *)headerBuffer, headerLength, m_rx_data, 0);
	if(err_code != NRF_SUCCESS)
	{
		simple_uart_putstring((const uint8_t *)"Write error - header (SPI).\n\r");
	}

	err_code = nrf_drv_spi_transfer(&m_spi_master_0, (uint8 *)bodyBuffer, bodylength, m_rx_data, 0);
	if(err_code != NRF_SUCCESS)
	{
		simple_uart_putstring((const uint8_t *)"Write error - body (SPI).\n\r");
	}

	nrf_gpio_pin_set(DW1000_SS_PIN);
	decamutexoff(irqs);

	return 0;
}

int readfromspi(uint16 headerLength,  const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	uint8_t irqs = decamutexon();
	nrf_gpio_pin_clear(DW1000_SS_PIN);

	uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master_0, (uint8 *)headerBuffer, headerLength, m_rx_data, headerLength);	

	if(err_code != NRF_SUCCESS)
	{
		simple_uart_putstring((const uint8_t *)"Write error - read header (SPI).\n\r");
	}

	err_code = nrf_drv_spi_transfer(&m_spi_master_0, m_rx_data, readlength, readBuffer, readlength);
	if(err_code != NRF_SUCCESS)
	{
		simple_uart_putstring((const uint8_t *)"Read error (SPI).\n\r");
	}

	nrf_gpio_pin_set(DW1000_SS_PIN);
	decamutexoff(irqs);

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
	initiator_rtls_beacon_receive_handler = beacon_handler;

	initDW1000();

	uint8_t tmp[600];
	dwt_spicswakeup(tmp, 600);

	reset_DW1000();

	spi_set_rate_low();
	int init_result = dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&config);
	
	dwt_setrxmode(DWT_RX_SNIFF, 2, 24);

	uint32 devid = dwt_readdevid();    
	simple_uart_puthex((devid>>24)&0xFF);
	simple_uart_puthex((devid>>16)&0xFF);
	simple_uart_puthex((devid>>8)&0xFF);
	simple_uart_puthex((devid)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

	if(init_result == DWT_ERROR)
	{
		simple_uart_putstring((const uint8_t *)"Error on dwt_initialise!\n\r");
		return 1;
	}

	simple_uart_putstring((const uint8_t *)"Configured: ");
	simple_uart_puthex((address>>8)&0xFF);
	simple_uart_puthex((address)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

    dwt_setleds(3);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(0);

//	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);

	int sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
	simple_uart_puthex((sys_cfg>>24)&0xFF);
	simple_uart_puthex((sys_cfg>>16)&0xFF);
	simple_uart_puthex((sys_cfg>>8)&0xFF);
	simple_uart_puthex((sys_cfg)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	int sys_mask = dwt_read32bitreg(SYS_MASK_ID);
	simple_uart_puthex((sys_mask>>24)&0xFF);
	simple_uart_puthex((sys_mask>>16)&0xFF);
	simple_uart_puthex((sys_mask>>8)&0xFF);
	simple_uart_puthex((sys_mask)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

	mac_init(address);
	dwt_setcallbacks(mac_txcallback_impl, mac_rxcallback_impl);

	dwt_txconfig_t configTx;
	configTx.PGdly = 0x95;
	configTx.power = 0x1A140E2E;
	dwt_configuretxrf(&configTx);
	
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

	uint32 devid = dwt_readdevid();    
	simple_uart_puthex((devid>>24)&0xFF);
	simple_uart_puthex((devid>>16)&0xFF);
	simple_uart_puthex((devid>>8)&0xFF);
	simple_uart_puthex((devid)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

	if(init_result == DWT_ERROR)
	{
		simple_uart_putstring((const uint8_t *)"Error on dwt_initialise!\n\r");	
		return 1;
	}

	simple_uart_putstring((const uint8_t *)"Configured: ");
	simple_uart_puthex((address>>8)&0xFF);
	simple_uart_puthex((address)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

    dwt_setleds(3);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(0);

//	dwt_write32bitreg(SYS_CFG_ID, SYS_CFG_DIS_DRXB | SYS_CFG_RXWTOE);

	int sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
	simple_uart_puthex((sys_cfg>>24)&0xFF);
	simple_uart_puthex((sys_cfg>>16)&0xFF);
	simple_uart_puthex((sys_cfg>>8)&0xFF);
	simple_uart_puthex((sys_cfg)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RXOVRR, 1);

	int sys_mask = dwt_read32bitreg(SYS_MASK_ID);
	simple_uart_puthex((sys_mask>>24)&0xFF);
	simple_uart_puthex((sys_mask>>16)&0xFF);
	simple_uart_puthex((sys_mask>>8)&0xFF);
	simple_uart_puthex((sys_mask)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");

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




