

#define PIN_NUM_MISO	22 					/*25*/
#define PIN_NUM_MOSI 	21					/*23*/
#define PIN_NUM_CLK  	23					/*19*/
#define PIN_NUM_CS   	19					/*22*/

//spi_device_handle_t spi;

spi_device_t* spi;


spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
		.max_transfer_sz = 2048	//TODO ezt mennyire?
};

spi_device_interface_config_t devcfg={
        .clock_speed_hz = SPI_MASTER_FREQ_8M,//1*1000*1000,//   //Clock out at 10 MHz TODO OK
        .mode 			= 0,                     //SPI mode 0
        .spics_io_num 	= PIN_NUM_CS,            //CS pin
        .queue_size 	= 7,                     //We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line TODO kell-e ezt kezelni?
		.flags			= SPI_DEVICE_HALFDUPLEX,
//			.dummy_bits		= 8,
//			.address_bits	= 8,
//			.command_bits	= 8,
};


int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    //TODO megcsinálni!
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));

	trans.length 	= headerLength*8;
	trans.rxlength 	= readlength*8;
	trans.tx_buffer = headerBuffer;
	trans.rx_buffer = readBuffer;

	esp_err_t ret = spi_device_transmit(spi, &trans);
	if(ret != ESP_OK) {
		return -1;
    }
    return 0;
}
