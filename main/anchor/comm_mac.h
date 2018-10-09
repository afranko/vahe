#ifndef __MAC_H_
#define __MAC_H_

#include <stdint.h>
#include "../decadriver/deca_device_api.h"
#include "../decadriver/deca_regs.h"
#include "queue.h"
#include "misc.h"

#define MAC_FRAME_TYPE_RANGING      0x00
#define MAC_FRAME_TYPE_BEACON       0x10

#define MAC_FRAME_ST_BEACON    0x00
#define MAC_FRAME_ST_POLL      0x01
#define MAC_FRAME_ST_RESP      0x02
#define MAC_FRAME_ST_FINAL     0x03
#define MAC_FRAME_ST_DIST      0x04

#define MAC_FRAME_BEACON_LENGTH     9
#define MAC_BEACON_PERIOD           200

QUEUE_EXTERN(mac_rx_queue);
QUEUE_EXTERN(mac_tx_queue);
BUFFER_POOL_EXTERN(mac_memory_pool);
extern uint8_t  _mac_seq_id;

typedef struct
{
    uint16_t    nhop_addr;
    uint8_t     nhop_count;
    uint32_t    nhop_tick;
} nwk_next_hop_t;

typedef struct
{
    uint64_t    rx_ts;
    uint16_t    src_addr;
    uint16_t    length;
    uint8_t     packet[0];
} mac_packet_info_t;


typedef struct {
    uint8_t     fctrl;
    uint8_t     seqid;
    uint16_t    src_addr;
    uint16_t    dst_addr;
    uint16_t    padding;
    uint8_t     payload[0];
} __packed mac_general_package_format_t;

typedef struct {
    uint8_t     fctrl;
    uint8_t     seqid;
    uint16_t    src_addr;
    uint16_t    hop_addr;
    uint8_t     hop_count;
} __packed mac_beacon_package_format_t;


void mac_init(uint16_t mac_addr);
void mac_txcallback_impl(const dwt_callback_data_t *data);
void mac_rxcallback_impl(const dwt_callback_data_t *data);

mac_packet_info_t*      mac_get_tx_buffer(uint8_t size);
void                    mac_free_buffer(void* buf);

int                     mac_transmit(mac_packet_info_t* msg);
void                    mac_transmit_beacon();
mac_packet_info_t*      mac_receive_poll();
void                    mac_transmit_poll();

void                    mac_start_ranging(uint16_t addr);

uint16_t                mac_get_addr();
void                    mac_set_next_hop_info(nwk_next_hop_t data);

uint8_t                 mac_send_results(uint8_t *data, uint8_t dLength, uint16_t dstntn);

void tfra_func(uint16_t addr);	//TODO + NOTE

static inline uint8_t mac_next_seq_id()
{
    return _mac_seq_id++;
}



#endif
