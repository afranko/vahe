#ifndef DW1000_H_
#define DW1000_H_

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "math.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_spi.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_delay.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include "rtls.h"
#include "comm_mac.h"

//#undef		EVBOARD
//#define	EVBOARD

#ifdef	EVBOARD
	#define DW1000_RST			7
	#define DW1000_IRQ			5
	#define DW1000_SCK_PIN		0
	#define DW1000_MISO_PIN		2
	#define DW1000_MOSI_PIN		4
	#define DW1000_SS_PIN		6
	#define DW1000_SS_LED		18
#else
	#define DW1000_RST			8
	#define DW1000_IRQ			25
	#define DW1000_SCK_PIN		28
	#define DW1000_MISO_PIN		29
	#define DW1000_MOSI_PIN		30
	#define DW1000_SS_PIN		0
	#define DW1000_SS_LED		18
#endif

typedef void (*response_receive_handler_t) (bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ); //NOTE+TODO
typedef void (*rtls_beacon_receive_handler_t) (uint16_t src_address, uint16_t hop_address, uint8_t hop_count);

#define simple_uart_put	app_uart_put
void simple_uart_putstring(const uint8_t *str);
void simple_uart_puthex(uint8_t hex);

uint16_t getTagAddress();
void deca_sleep(unsigned int time_ms);
void initDW1000(void);
void reset_DW1000(void);
void spi_set_rate_low(void);
void spi_set_rate_high(void);

decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

bool deca_twr_poll_msg();
void deca_twr_rxtimeout();
void deca_twr_initiator_send_poll(uint16_t dst_address);
void deca_twr_initiator_listen_to_beacon();

void deca_twr_configure();
int deca_twr_initiator(response_receive_handler_t handler, rtls_beacon_receive_handler_t beacon_handler);

void deca_twr_responder_send_rtls_beacon();
int deca_twr_responder(void);

#endif /* DW1000_H_ */


