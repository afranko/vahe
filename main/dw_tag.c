#include "dw_tag.h"

//#include "ble_lbs.h"



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







void ts_timeout_handler(void * p_context)   //NOTE+TODO
{
    macTimeStamp++;
}

void measurement_timeout_handler(void * p_context)
{
    //UNUSED_PARAMETER(p_context);

    if(m_lbs.is_ranging_notification_enabled)
    {
        uint8_t tmp[600];
        dwt_spicswakeup(tmp, 600);

        act_anchor = 0;
        act_range = 0;

        while(act_anchor < MAX_ANCHORS && m_lbs.anchorlist[act_anchor] == 0) { act_anchor++; }

        if(act_anchor == MAX_ANCHORS)
        {
//			simple_uart_putstring((const uint8_t *)"A");
            deca_twr_initiator_listen_to_beacon();
            listen_to_beacon = 1;

            xTimerStart_manual(timerhandlers[0]);
        }
        else
        {   
			printf("%d \n",((int)'B'));
            deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
            xTimerStart_manual(timerhandlers[2]);
        }
    }
}



void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ)
{
    //app_timer_stop(m_rxtimeout_timer_id);
	xTimerStop_manual(timerhandlers[2]);

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
    //printf((src_address>>8)&0xFF);
    //simple_uart_puthex((src_address)&0xFF);
    //simple_uart_putstring((const uint8_t *)" initiator_response_receive_handler called\n\r");
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

			//ble_lbs_anchorlist_update(&m_lbs);
        }
    }
    else
        m_lbs.anchor_notpresence[act_anchor] = 0;

    uint32_t pressure = 0xACDCACDC;
    int32_t temperature = 0;

    /* NOTE+TODO: Functioning without ble connection */

    //if(m_lbs.conn_handle != BLE_CONN_HANDLE_INVALID)
    //    err_code = ble_lbs_ranging_update(&m_lbs, (uint8_t*) &temp_range, 10*sizeof(uint8_t));



    strcat(debugMessage, "A:");
    printf(debugMessage + strlen(debugMessage), "%x", src_address);    // A forceoltnál is forcoltabb castolás
    strcat(debugMessage, " D:");
    printf(debugMessage + strlen(debugMessage), "%d", distance);
    strcat(debugMessage, "; ");

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
        send_debug_message(debugMessage);   // Send Debug Message
        debugMessage[0] = 0;    // Init String


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
        xTimerStart_manual(timerhandlers[2]);
    }
    else if(listen_to_beacon_cnt == 0)
    {
        deca_twr_initiator_listen_to_beacon();
        listen_to_beacon = 1;
        listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;
        xTimerStart_manual(timerhandlers[2]);
    }
    else
    {
        listen_to_beacon_cnt--;
        deca_twr_rxtimeout();
        //dwt_entersleep();
    }
}
void rxtimeout_timeout_handler(void * p_context)
{
    //UNUSED_PARAMETER(p_context);

    deca_twr_rxtimeout();

    if(listen_to_beacon == 0)
    {

        response_receive_handler(false, m_lbs.anchorlist[act_anchor], 0, 0);

    }
    else
    {
        listen_to_beacon = 0;

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

        //ble_lbs_anchorlist_update(&m_lbs);
    }
}


void send_debug_message(char message[])
{
    uint16_t length = strlen(message);
    uint16_t lenbyte = length + 2;
    uint8_t outByte[9];
    uint16_t tag_id = getTagAddress();

    outByte[0] = 0xAA;
    outByte[1] = (uint8_t)(lenbyte >> 8) & 0xFF;
    outByte[2] = (uint8_t) lenbyte & 0xFF;
    outByte[3] = (uint8_t)(tag_id >> 8) & 0xFF;
    outByte[4] = (uint8_t) tag_id & 0xFF;
    outByte[5] = (uint8_t)(macTimeStamp >> 24) & 0xFF;
    outByte[6] = (uint8_t)(macTimeStamp >> 16) & 0xFF;
    outByte[7] = (uint8_t)(macTimeStamp >> 6) & 0xFF;
    outByte[8] = (uint8_t) macTimeStamp & 0xFF;



    printf("%u,%u,%u,%u,%u,%u,%u,%u,%u, \n", outByte[0],outByte[1],outByte[2],outByte[3],outByte[4],outByte[5],outByte[6],outByte[7],outByte[8]);


}
