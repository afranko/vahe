#include "dw_tag.h"

#include <string.h>
#include <stdio.h>

#include "ble_lbs.h"

#include "nrf_deca.h"

#include "commons.h"
#include "send_tag_uwb.h"

#include "decadriver/deca_device_api.h"



static int listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;

uint8_t mpu[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int act_range = 0;
int act_anchor = 0;
int listen_to_beacon = 0;
unsigned long dw_timestamp = 0;

bool isMacSendEnabled   = true;   // NOTE: for mac_send
bool isMsgRangingInit   = false;  // NOTE: for mac_send
bool isMsgMpuInit   = false;  // NOTE: for mac_send

rangingMessage  rMessage;
mpuMessage      mMessage;
uint64_t        macTimeStamp    =   0;
//static bool     mpuTX           =   false;
static bool     rangTX          =   false;

uint8_t rSerializedMessage[200];
uint8_t mSerializedMessage[200];

char debugMessage[256] = "/0";

//static int altimeterMeasureCount = 0;
//static int altimeterMeasureState = 0;
//static unsigned long altimeterMeasureTs = 0;
//static uint8_t seqnum = 0;

void rTimerCallback(TimerHandle_t pxTimer)
{
	//ets_printf("TIMER ");
	//ets_printf("%d", (uint32_t) pvTimerGetTimerID(pxTimer));
	//ets_printf(" has stopped!\n");
	rxtimeout_timeout_handler();
}


void mpu_timeout_handler(void * p_context) {}

void ts_timeout_handler()
{
    macTimeStamp++;
}

void measurement_timeout_handler()
{
    if(m_lbs.is_ranging_notification_enabled)
    {
        //uint8_t tmp[600];				//TODO+NOTE
        //dwt_spicswakeup(tmp, 600);	//TODO+NOTE

        act_anchor = 0;
        act_range = 0;

        while(act_anchor < MAX_ANCHORS && m_lbs.anchorlist[act_anchor] == 0) { act_anchor++; }

        if(act_anchor == MAX_ANCHORS)
        {
//			simple_uart_putstring((const uint8_t *)"A");
            deca_twr_initiator_listen_to_beacon();
            listen_to_beacon = 1;
            xTimerStart(xTimers[4], 0);	//NOTE+TODO TICKS!
        }
        else
        {   
//			simple_uart_putstring((const uint8_t *)"B");
            deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
            xTimerStart(xTimers[3], 0);	//NOTE+TODO TICKS!
        }
    }
}

void rxtimeout_timeout_handler(void)
{
    deca_twr_rxtimeout();

    if(listen_to_beacon == 0)
    {
        response_receive_handler(false, m_lbs.anchorlist[act_anchor], 0, 0);
        //ets_printf("MALACOK AZ URBEN!\n");
    }
    else
    {
        listen_to_beacon = 0;

        //dwt_entersleep();
    }
}

void init_rx_timeout_timer()
{
	xTimers[3] = xTimerCreate("ranging_timer", RANGING_RX_TICKNUM/portTICK_PERIOD_MS, pdFALSE , (void*)3, rTimerCallback);
	xTimers[4] = xTimerCreate("beacon_timer", BEACON_RX_TICKNUM/portTICK_PERIOD_MS, pdFALSE , (void*)4, rTimerCallback);
}

void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ)
{
    //app_timer_stop(m_rxtimeout_timer_id);
	//TODO+NOTE TIMERSTOP????�

	xTimerStop(xTimers[3], 0);
	xTimerStop(xTimers[4], 0);

    if(!isMsgRangingInit)       // NOTE: for mac_send
    {
        init_ranging_msg(&rMessage, getTagAddress());
        isMsgRangingInit = true;
    }

	//simple_uart_putstring((const uint8_t *)"F");
    /*simple_uart_putstring((const uint8_t *)"Distance\n\r");
    simple_uart_puthex((distance>>8)&0xFF);
    simple_uart_puthex((distance)&0xFF);
    simple_uart_putstring((const uint8_t *)" \n\r");*/
    ets_printf("%X", (src_address>>8)&0xFF);
    ets_printf("%X", (src_address)&0xFF);
    ets_printf(" initiator_response_receive_handler called\n\r");
/*
    simple_uart_putstring((const uint8_t *)"macTimeStamp:\n\r");  
    simple_uart_puthex((macTimeStamp>>24) & 0xFF);
    simple_uart_puthex((macTimeStamp>>16) & 0xFF);
    simple_uart_puthex((macTimeStamp>>8) & 0xFF);
    simple_uart_puthex(macTimeStamp & 0xFF);
    simple_uart_putstring((const uint8_t *)"\n\r");

    simple_uart_putstring((const uint8_t *)"rxQ:\n\r"); 
    simple_uart_puthex(RxQ);
    simple_uart_putstring((const uint8_t *)"\n\r");*/

    if(!success)
    {
        m_lbs.anchor_notpresence[act_anchor]++;

        if(m_lbs.anchor_notpresence[act_anchor] > 3)
        {
            m_lbs.anchorlist[act_anchor] = 0;
            m_lbs.anchorcount--;

			ble_lbs_anchorlist_update(&m_lbs);
        }
    }
    else
        m_lbs.anchor_notpresence[act_anchor] = 0;

    uint32_t pressure = 0xACDCACDC;
    int32_t temperature = 0;

    /*uint8_t temp_range[10];

    temp_range[0] = dw_timestamp & 0xFF;
    temp_range[1] = (dw_timestamp>>8) & 0xFF;
    temp_range[2] = (dw_timestamp>>16) & 0xFF;
    temp_range[3] = (dw_timestamp>>24) & 0xFF;
    temp_range[4] = act_range & 0xFF;
    temp_range[5] = (distance) & 0xFF;
    temp_range[6] = (distance>>8) & 0xFF;
    temp_range[7] = (m_lbs.anchor_hop_address[act_anchor]) & 0xFF;
    temp_range[8] = (m_lbs.anchor_hop_address[act_anchor]>>8) & 0xFF;
    temp_range[9] = m_lbs.anchor_hop_count[act_anchor] & 0xFF;*/

    /*strcat(debugMessage, "A:");
    sprintf(debugMessage + strlen(debugMessage), "%x", src_address);    // A forceoltnál is forcoltabb castolás
    strcat(debugMessage, " D:");
    sprintf(debugMessage + strlen(debugMessage), "%d", distance);
    strcat(debugMessage, "; ");*/

    ets_printf("A: %x, D: %d \n", src_address, distance);

    /* MESSAGE RACKING */
    if(isMacSendEnabled && success)
        racking_ranging(&rMessage, src_address, distance, RxQ);

    act_range++;

    act_anchor++;
    while(act_anchor < MAX_ANCHORS && m_lbs.anchorlist[act_anchor] == 0) { act_anchor++; }

    /* NOTE: SEND TO ANCHOR */
    if(act_anchor == MAX_ANCHORS) //NOTE+TODO
    {
/*        unsigned char counterAlt= 0;
        while(altimeterMeasureState != 3 && counterAlt < 10)
        {
            getAltimeterPressure(&pressure, NULL, &altimeterMeasureState);
            nrf_delay_us(8220);
        }

        if(altimeterMeasureState != 3)
            pressure = 0xABCDACDC;
        altimeterMeasureState = 0;

*/
        //send_debug_message(debugMessage);   // Send Debug Message
        //debugMessage[0] = 0;    // Init String

        //if(mpu_get_temperature(&temperature, NULL) != 0)
        temperature = 0xABCDACDC;
        set_ranging_pt(&rMessage, (int32_t)pressure, temperature);

        if(!rangTX)
        {
            rangTX = true;
            pts_ranging_uwb(&rMessage, macTimeStamp, rSerializedMessage);
            rangTX = false;
        }
    }

    if(act_anchor < MAX_ANCHORS)
    {
//		simple_uart_putstring((const uint8_t *)"C");
        deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
        xTimerStart(xTimers[3], 0);
    }
    else if(listen_to_beacon_cnt == 0)
    {
        deca_twr_initiator_listen_to_beacon();
        listen_to_beacon = 1;
        listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;
        xTimerStart(xTimers[4], 0);	//TODO+NOTE 0-s timer
    }
    else
    {
        listen_to_beacon_cnt--;
        deca_twr_rxtimeout();
        //dwt_entersleep();
    }
}

void rtls_beacon_receive_handler(uint16_t src_address, uint16_t hop_address, uint8_t hop_count)
{
	//simple_uart_putstring((const uint8_t *)"E");
/*
	simple_uart_puthex((src_address>>8)&0xFF);
	simple_uart_puthex((src_address)&0xFF);
	simple_uart_putstring((const uint8_t *)"\n\r");
*/
    int alreadyIn = 0;
    for(int i=0; i < MAX_ANCHORS; i++)
    {
        if(m_lbs.anchorlist[i] == src_address)
        {
            m_lbs.anchor_hop_address[i] = hop_address;
            m_lbs.anchor_hop_count[i] = hop_count;
            m_lbs.anchor_notpresence[i] = 0;
            alreadyIn = 1;
            break;
        }
    }

    if(alreadyIn == 0)
    {
        for(int i=0; i < MAX_ANCHORS; i++)
        {
            if(m_lbs.anchorlist[i] == 0)
            {
                m_lbs.anchorlist[i] = src_address;
                m_lbs.anchor_hop_address[i] = hop_address;
                m_lbs.anchor_hop_count[i] = hop_count;
                m_lbs.anchor_notpresence[i] = 0;
                m_lbs.anchorcount++;
                break;
            }
        }

        ble_lbs_anchorlist_update(&m_lbs);
    }

//	simple_uart_puthex((m_lbs.anchorcount)&0xFF);
//	simple_uart_putstring((const uint8_t *)"\n\r");
}

/*
void send_debug_message(char message[])
{    
    uint16_t length = (uint8_t) strlen(message) + 2;     // MAX. 255 hosszú payload lehet csak! Ha nem tartjuk be csonkolt lesz az üzenet
    uint16_t lenCounter = (length / 256) + 1;
    uint16_t tag_id = getTagAddress();
    char output[255+8];

    do
    {
        output[0] = 0xAA;
        output[1] = length > 255 ? 255 : length;
        output[2] = (uint8_t)(tag_id >> 8) & 0xFF;
        output[3] = (uint8_t) tag_id & 0xFF;
        output[4] = (uint8_t)(macTimeStamp >> 24) & 0xFF;
        output[5] = (uint8_t)(macTimeStamp >> 16) & 0xFF;
        output[6] = (uint8_t)(macTimeStamp >> 6) & 0xFF;
        output[7] = (uint8_t) macTimeStamp & 0xFF;

        for(uint8_t i = 0; i < (length > 255 ? 255 : (length - 2)); i++)
            output[i+8] = message[i];

        if(length < 256)
        {
            output[length-1] = '\r';
            output[length] = '\n';
        }

        simple_uart_putstring((const uint8_t *)output);
        length -= 255;
    }
    while(--lenCounter > 0);
}
*/

void send_debug_message(char message[])
{
    uint16_t length = strlen(message);
    //uint16_t lenbyte = length + 2;
    char output[length+3];  // CR + LF + NULL
    //uint8_t outByte[9];
    //uint16_t tag_id = getTagAddress();

    /*outByte[0] = 0xAA;
    outByte[1] = (uint8_t)(lenbyte >> 8) & 0xFF;
    outByte[2] = (uint8_t) lenbyte & 0xFF;
    outByte[3] = (uint8_t)(tag_id >> 8) & 0xFF;
    outByte[4] = (uint8_t) tag_id & 0xFF;
    outByte[5] = (uint8_t)(macTimeStamp >> 24) & 0xFF;
    outByte[6] = (uint8_t)(macTimeStamp >> 16) & 0xFF;
    outByte[7] = (uint8_t)(macTimeStamp >> 6) & 0xFF;
    outByte[8] = (uint8_t) macTimeStamp & 0xFF;*/

    for(uint16_t i = 0; i < length; i++)
        output[i] = message[i];

    output[length] = '\r';
    output[length+1] = '\n';
    output[length+2] = '\0';

    ets_printf("%s",output);	//TODO+NOTE meg ra kell nezni

    //nrf_drv_uart_tx((uint8_t const * const)outByte, 9U);

    //for(uint16_t i = 0; i < length+3; ++i)
    //   simple_uart_putstring((const uint8_t*)&output[i]);

    //simple_uart_putstring((const uint8_t*) output);
    //nrf_drv_uart_tx((uint8_t const * const)output, length+2U);
}
