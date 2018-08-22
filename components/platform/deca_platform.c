
//TODO kommentezni, kultúráltá tenni a fájlt + doxygen + hibajavtás + tesztelés

/* SPI device handler structure */
static spi_device_handle_t spi;

/* SPI bus config descriptor */
static spi_bus_config_t spi_bus_cfg = {
	.miso_io_num = DW1000_MISO_PIN,
	.mosi_io_num = DW1000_MOSI_PIN,
	.sclk_io_num = DW1000_SCK_PIN,
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,
};

/* SPI device config descriptor */
static spi_device_interface_config_t spi_dev_cfg = {
	.clock_speed_hz = 1*1000*1000,
	.mode = 0,
	.spics_io_num = -1,
	.queue_size = 1,
};



esp_err_t spi_init_deca(void)
{
	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
   	io_conf.mode = GPIO_MODE_OUTPUT;
   	io_conf.pin_bit_mask = ( 1ULL << DW1000_CS_PIN);
   	io_conf.pull_down_en = 0;
   	io_conf.pull_up_en = 1;
   	gpio_config(&io_conf);	//TODO HIBAKEZELÉS
   	gpio_set_level(DW1000_CS_PIN, GPIO_LOW);	//TODO HIBAKEZELÉS

	return ESP_OK;
}


int spi_set_rate_high()	//TODO vagy csak assert, mi legyen? nem tudom!
{
	if(spi_bus_remove_device(spi) != ESP_OK) {
		return DWT_ERROR;
	}

	devcfg.clock_speed_hz=20*1000*1000;		// 20MHz

	if(spi_bus_add_device(VSPI_HOST, &devcfg, &spi) != ESP_OK) {	//TODO ASSERT?
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

int spi_set_rate_low()
{
	return DWT_SUCCESS;
}

//TODO

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	//TODO MUTEX_ON and STUFFS
	gpio_set_level(DW1000_CS_PIN, GPIO_LOW);

	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	memset(&trans[1], 0, sizeof(spi_transaction_t));

	trans[0].length = headerLength*8;
	trans[0].tx_buffer = headerBuffer;

	trans[1].length = bodylength*8;
	trans[1].tx_buffer = bodyBuffer;

	if(spi_device_transmit(spi, &trans[0]) == ESP_OK) {
		if(spi_device_transmit(spi, &trans[1]) != ESP_OK) {	//TODO LOG
			return -1;
		}
	}
	else {
		return -1;
	}

	return 0;

	gpio_set_level(DW1000_CS_PIN, GPIO_HIGH);
	//TODO MUTEX_OFF and STUFFS


}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	//TODO MUTEX_ON and STUFFS
	gpio_set_level(DW1000_CS_PIN, GPIO_LOW);

	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	memset(&trans[1], 0, sizeof(spi_transaction_t));

	trans[0].length = headerLength*8;
	trans[1].length = readlength*8;
	trans[0].tx_buffer = headerBuffer;
	trans[1].rx_buffer = readBuffer;

	if(spi_device_transmit(spi, &trans[0]) == ESP_OK) {
		if(spi_device_transmit(spi, &trans[1]) != ESP_OK) {	//TODO LOG
			return -1;
		}
	}
	else {
		return -1;
	}

	return 0;

	gpio_set_level(DW1000_CS_PIN, GPIO_HIGH);
	//TODO MUTEX_OFF and STUFFS
}
