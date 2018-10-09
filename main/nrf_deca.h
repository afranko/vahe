#ifndef DW1000_H_
#define DW1000_H_

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "math.h"

#include "decadriver/deca_types.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#include "anchor/rtls.h"
#include "anchor/comm_mac.h"

#define DW1000_RST			5
#define DW1000_IRQ			18
#define DW1000_SCK_PIN		23
#define DW1000_MISO_PIN		22
#define DW1000_MOSI_PIN		21
#define DW1000_SS_PIN		19

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

/* Semafor City */
void initShield(void);

#endif /* DW1000_H_ */


