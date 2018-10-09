#include "comm_mac.h"
#include "rtls.h"
#include "platform.h"

static uint8_t              _rtls_buffer[RTLS_BUFFER_SIZE];
static uint16_t             _mac_addr;
uint8_t                     _mac_seq_id = 0;
volatile uint8_t            mac_rtls_tx_flag = 0;

static nwk_next_hop_t       nwk_next_hop_data;

extern uint64_t dwm1000_get_system_time_u64(void);
extern uint64_t dwm1000_get_rx_timestamp_u64(void);

extern rtls_struct_t last_saved;

// NOTICE: size must be 32-bit aligned!
BUFFER_POOL_CREATE(mac_memory_pool,COMM_MAC_BUFFER_LENGTH,COMM_MAC_BUFFER_ELEMENT_SIZE+
    sizeof(mac_packet_info_t)+128);
//NOTE+TODO: EXTENDED BUFFERSIZE
QUEUE_CREATE(mac_rx_queue,COMM_MAC_RX_QUEUE_LENGTH);
QUEUE_CREATE(mac_tx_queue,COMM_MAC_TX_QUEUE_LENGTH);

typedef enum
{
    RX_RES_THROW,
    RX_RES_OK_RXE,
    RX_RES_OK_RXD
} rx_result_t;

typedef enum
{
    TX_SENT,
    TX_NOTHING
} tx_result_t;

static rx_result_t  handle_rx_packet(mac_packet_info_t *mac_pkg);
static tx_result_t  mac_send_from_tx_queue();
static int          mac_send_ranging_message(rtls_struct_t* rtls);

void mac_init(uint16_t mac_addr)
{
    _mac_addr = mac_addr;
    buffer_pool_init(&mac_memory_pool);

    rtls_init(mac_addr);
    nwk_next_hop_data.nhop_addr = 0x0000;
    nwk_next_hop_data.nhop_count = 0xFF;
    nwk_next_hop_data.nhop_tick = 0;

    printf("MAC: init (addr: %04X)", _mac_addr);
}


void mac_txcallback_impl(const dwt_callback_data_t *data)
{
    if(mac_rtls_tx_flag)
    {
        mac_rtls_tx_flag = 0;
        dwt_rxenable(0);
    }
    else
    {
        if(mac_send_from_tx_queue() == TX_NOTHING)
        {
            dwt_rxenable(0);
        }
    }
}

void mac_rxcallback_impl(const dwt_callback_data_t *data)
{
    if(data->event == DWT_SIG_RX_OKAY)
    {
        mac_packet_info_t*   mac_pkg = buffer_alloc(&mac_memory_pool, data->datalength + sizeof(mac_packet_info_t));
        if(mac_pkg != NULL)
        {
            dwt_readrxdata(mac_pkg->packet, data->datalength - 2, 0);
            mac_pkg->length = data->datalength - 2;

            switch(handle_rx_packet(mac_pkg))
            {
            case RX_RES_THROW:
                buffer_free(&mac_memory_pool, mac_pkg);
                dwt_rxenable(0);
                break;
            case RX_RES_OK_RXE:
                queue_push(&mac_rx_queue, mac_pkg);
                dwt_rxenable(0);
                break;
            case RX_RES_OK_RXD:
                queue_push(&mac_rx_queue, mac_pkg);
                break;
            }
        }
        else
        {
            printf("Cannot allocate buffer for RX packet");
            dwt_rxenable(0);
        }
    }
    else
    {
        dwt_rxenable(0);
    }
}

static rx_result_t handle_rx_packet(mac_packet_info_t *mac_pkg)
{
    // TODO: check minimal length

    uint8_t type = mac_pkg->packet[0] & 0xF0;

    mac_pkg->src_addr = mac_pkg->packet[2];
    mac_pkg->src_addr |= mac_pkg->packet[3] << 8;

    uint16_t dst_addr = mac_pkg->packet[4];         // destination address or anchor address
    dst_addr |=  mac_pkg->packet[5] << 8;

    if(type == MAC_FRAME_TYPE_BEACON)
        return RX_RES_OK_RXE;

    if(dst_addr != _mac_addr) {
    	//ets_printf("MALACOK AZ URBEN! LAKCIM NELKUL! ADDR: %x\n", dst_addr);
        return RX_RES_THROW;
    }

    // shortcuts for ranging
    if(type == MAC_FRAME_TYPE_RANGING)
    {

        rtls_struct_t rtls;
        rtls.rx_ts = dwm1000_get_rx_timestamp_u64();
        rtls.msg = mac_pkg->packet;
        rtls.length = mac_pkg->length;
        rtls.out = _rtls_buffer;
        mac_pkg->rx_ts = rtls.rx_ts;

        rtls_res_t ret = rtls_handle_message(&rtls);
        if(ret == RTLS_OK)
        {
            if(mac_send_ranging_message(&rtls) != 0)
            {
                // Failed to send or not sent ranging message
                return RX_RES_OK_RXE;
            }
        }
        else
        {
            ets_printf("RX, RTLS packet process failed: %02X", ret);
            dwt_rxenable(0);
        }

        return RX_RES_OK_RXD;
    }
    else
    {
        ets_printf("Not ranging message");
    }

    return RX_RES_OK_RXE;
}

/**
 * @brief mac_send_ranging_message
 * @param rtls
 * @return 0 on successful sending, otherwise < 0
 */
static int mac_send_ranging_message(rtls_struct_t* rtls)
{
    if(rtls->out_length > 0)
    {
        mac_rtls_tx_flag = 1;

        dwt_writetxdata(rtls->out_length + 2, rtls->out, 0);
        dwt_writetxfctrl(rtls->out_length + 2, 0);

        if(rtls->tx_ts_32 != 0)
        {
            dwt_setdelayedtrxtime(rtls->tx_ts_32);
            int r = dwt_starttx(DWT_START_TX_DELAYED);
            if(r != DWT_SUCCESS)
            {
                printf("RTLS message TX failed: %d (%02X)\n", r, rtls->out[0]);
                return -1;
            }
        }
        else
        {
            dwt_starttx(DWT_START_TX_IMMEDIATE);
        }

        return 0;
    }

    return -2;
}


static tx_result_t  mac_send_from_tx_queue()
{
    mac_packet_info_t* msg = queue_pop(&mac_tx_queue);
    if(msg != NULL)
    {
        dwt_forcetrxoff();
        dwt_writetxdata(msg->length + 2, msg->packet, 0);
        dwt_writetxfctrl(msg->length + 2, 0);   // add CRC
        buffer_free(&mac_memory_pool, msg);

        dwt_starttx(DWT_START_TX_IMMEDIATE);

        return TX_SENT;
    }

    return TX_NOTHING;
}

int mac_transmit(mac_packet_info_t *msg)
{
    return queue_push(&mac_tx_queue, msg);
}

void mac_transmit_beacon()
{
    mac_packet_info_t* beacon_pkg = mac_get_tx_buffer(MAC_FRAME_BEACON_LENGTH);
    if(beacon_pkg != NULL)
    {
        beacon_pkg->length = sizeof(mac_beacon_package_format_t);

        mac_beacon_package_format_t* beacon = (mac_beacon_package_format_t*)beacon_pkg->packet;
        beacon->fctrl = MAC_FRAME_TYPE_BEACON;
        beacon->seqid = mac_next_seq_id();
        beacon->src_addr = _mac_addr;
        beacon->hop_addr = nwk_next_hop_data.nhop_addr;
        beacon->hop_count = nwk_next_hop_data.nhop_count;

        mac_transmit(beacon_pkg);
    }
}

mac_packet_info_t *mac_receive_poll()
{
    return (mac_packet_info_t*)queue_pop(&mac_rx_queue);
}

uint16_t mac_get_addr()
{
    return _mac_addr;
}

void mac_start_ranging(uint16_t addr)
{
    rtls_struct_t rtls;
    rtls.rx_ts = dwm1000_get_system_time_u64();
    rtls.out = _rtls_buffer;

    rtls_compose_poll_msg(addr,&rtls);

    dwt_forcetrxoff();
    if(mac_send_ranging_message(&rtls) != 0)
        dwt_rxenable(0);
}

void tfra_func(uint16_t addr)
{
	if(mac_send_ranging_message(&last_saved) != 0)
		dwt_rxenable(0);
}


mac_packet_info_t *mac_get_tx_buffer(uint8_t size)
{
    return buffer_alloc(&mac_memory_pool, size + sizeof(mac_packet_info_t));
}

void mac_transmit_poll()
{
    mac_send_from_tx_queue();
}

uint8_t mac_send_results(uint8_t *data, uint8_t dLength, uint16_t dstntn)
{
    mac_packet_info_t* info = mac_get_tx_buffer(sizeof(mac_general_package_format_t)+dLength);

    if(info == NULL)
        return 1;

    info->length = sizeof(mac_general_package_format_t)+dLength;

    mac_general_package_format_t* packet = (mac_general_package_format_t*)info->packet;

    packet->fctrl = 0x30;
    packet->seqid = 6;
    packet->src_addr = mac_get_addr();
    packet->dst_addr = dstntn;
    for(uint8_t i = 0; i < dLength; i++)
        packet->payload[i] = data[i];

    mac_transmit(info);
    mac_transmit_poll();
    //while(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS){}
    return 0;
}


void mac_set_next_hop_info(nwk_next_hop_t data)
{
    nwk_next_hop_data = data;
}


void mac_free_buffer(void *buf)
{
    buffer_free(&mac_memory_pool, buf);
}

