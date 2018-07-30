#ifndef DW1000_H_
#define DW1000_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

//#include "app_error.h"
//#include "deca_device_api.h"
#include "deca_types.h"

#include "deca_regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "freertos/timers.h"
#include "driver/spi_master.h"

#include "comm_mac.h"
#include "math.h"
#include "rtls.h"

//#undef		EVBOARD
//#define	EVBOARD

#define DEBUGMODE 1

#define DW1000_RST			5
#define DW1000_IRQ			18
#define DW1000_SCK_PIN		23
#define DW1000_MISO_PIN		22
#define DW1000_MOSI_PIN		21

#define PIN_NUM_MISO 22
#define PIN_NUM_MOSI 21
#define PIN_NUM_CLK 23
#define PIN_NUM_CS 19
#define SPI_PIN_NOT_USED -1
#define SPI_MAX_TRANSFER_SIZE 2048
#define SPI_DMA_CHANNEL 0
#define SPI_CLOCK_SPEED_1M 1*1000*1000
#define SPI_CLOCK_SPEED_8M 8*1000*1000
#define SPI_CLOCK_SPEED_10M 10*1000*1000
#define SPI_QUEUE_SIZE 7
spi_device_handle_t* spi_dev;
spi_bus_config_t* spi_buscfg;
spi_device_interface_config_t* spi_devcfg;
    // function headers
    esp_err_t initDW1000(void);
    esp_err_t reset_DW1000(void);


    esp_err_t spi_set_rate_low(void);
    esp_err_t spi_set_rate_high(void);

//	esp_err_t spi_set_rate_low(spi_device_interface_config_t *spi_devcfg, spi_device_handle_t *spi_dev,spi_bus_config_t    *spi_buscfg);
//	esp_err_t spi_set_rate_high(spi_device_interface_config_t *spi_devcfg, spi_device_handle_t *spi_dev,spi_bus_config_t    *spi_buscfg);
typedef void (*response_receive_handler_t) (bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ); //NOTE+TODO
typedef void (*rtls_beacon_receive_handler_t) (uint16_t src_address, uint16_t hop_address, uint8_t hop_count);

//#define simple_uart_put	app_uart_put
//void simple_uart_putstring(const uint8_t *str);
//void simple_uart_puthex(uint8_t hex);
uint16_t getTagAddress();
void deca_sleep(unsigned int time_ms);
portMUX_TYPE decamutexon(void);
void decamutexoff(portMUX_TYPE in);



int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer);
int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);

bool deca_twr_poll_msg();
void deca_twr_rxtimeout();
void deca_twr_initiator_send_poll(uint16_t dst_address);
void deca_twr_initiator_listen_to_beacon();

void deca_twr_configure();
int deca_twr_initiator(response_receive_handler_t handler, rtls_beacon_receive_handler_t beacon_handler);

void deca_twr_responder_send_rtls_beacon();
int deca_twr_responder(void);

#endif /* DW1000_H_ */


