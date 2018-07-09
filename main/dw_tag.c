#include "dw_tag.h"

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_conn_params.h"
#include "board.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "led.h"
#include "ble_lbs.h"
#include "battery.h"
#include "device_manager.h"
#include "app_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_deca.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "mpu.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "commons.h"
#include "send_tag_uwb.h"
#include "nrf_drv_uart.h"


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
static bool     mpuTX           =   false;
static bool     rangTX          =   false;

uint8_t rSerializedMessage[200];
uint8_t mSerializedMessage[200];

char debugMessage[256] = "/0";

//static int altimeterMeasureCount = 0;
//static int altimeterMeasureState = 0;
//static unsigned long altimeterMeasureTs = 0;
static uint8_t seqnum = 0;

APP_TIMER_DEF(m_rxtimeout_timer_id);

void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_start();
}

void mpu_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
}

void ts_timeout_handler(void * p_context)   //NOTE+TODO
{
    macTimeStamp++;
}

void measurement_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

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
            app_timer_start(m_rxtimeout_timer_id, BEACON_RX_TIMEOUT, NULL);
        }
        else
        {   
//			simple_uart_putstring((const uint8_t *)"B");
            deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
            app_timer_start(m_rxtimeout_timer_id, RANGING_RX_TIMEOUT, NULL);
        }
    }
}


#ifdef	CALIB

typedef struct {
    int		dist;
    int		cnt;
} histogram_t;

int calib_cnt = 1;
bool dist_arrived = false;
double m_oldM, m_newM, m_oldS, m_newS;
double avgDistance = 0;

#define HISTOGRAM_SIZE	30
histogram_t histogram[HISTOGRAM_SIZE];

void calib_response_receive_handler(bool success, uint16_t src_address, uint16_t distance)
{
    if(success)
    {
		simple_uart_puthex((distance>>8)&0xFF);
		simple_uart_puthex((distance)&0xFF);
		simple_uart_putstring((const uint8_t *)"\n\r");

        for(int i=0; i < HISTOGRAM_SIZE; i++)
        {
            if(histogram[i].dist == distance)
            {
                histogram[i].cnt++;
                break;
            }
            else if(histogram[i].dist == -1)
            {
                histogram[i].dist = distance;
                histogram[i].cnt++;
                break;
            }
        }

        avgDistance += distance;

        if(calib_cnt == 1)
        {
            m_oldM = m_newM = distance;
            m_oldS = 0.0;
        }
        else
        {
            m_newM = m_oldM + (distance - m_oldM) / calib_cnt;
            m_newS = m_oldS + (distance - m_oldM)*(distance - m_newM);

            m_oldM = m_newM;
            m_oldS = m_newS;
        }

        dist_arrived = true;
        calib_cnt++;
    }
    else
    {
        dist_arrived = true;
    }
}

#endif

void rxtimeout_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    deca_twr_rxtimeout();

    if(listen_to_beacon == 0)
    {
#ifdef	CALIB

        calib_response_receive_handler(false, CALIB_ANCHOR_ADDRESS, 0);

#else

        response_receive_handler(false, m_lbs.anchorlist[act_anchor], 0, 0);

#endif
    }
    else
    {
        listen_to_beacon = 0;

        //dwt_entersleep();
    }
}

void init_rx_timeout_timer()
{
    uint32_t err_code = app_timer_create(&m_rxtimeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, rxtimeout_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ)
{
    app_timer_stop(m_rxtimeout_timer_id);

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
    simple_uart_puthex((src_address>>8)&0xFF);
    simple_uart_puthex((src_address)&0xFF);
    simple_uart_putstring((const uint8_t *)" initiator_response_receive_handler called\n\r");
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

    uint8_t temp_range[10];

    temp_range[0] = dw_timestamp & 0xFF;
    temp_range[1] = (dw_timestamp>>8) & 0xFF;
    temp_range[2] = (dw_timestamp>>16) & 0xFF;
    temp_range[3] = (dw_timestamp>>24) & 0xFF;
    temp_range[4] = act_range & 0xFF;
    temp_range[5] = (distance) & 0xFF;
    temp_range[6] = (distance>>8) & 0xFF;
    temp_range[7] = (m_lbs.anchor_hop_address[act_anchor]) & 0xFF;
    temp_range[8] = (m_lbs.anchor_hop_address[act_anchor]>>8) & 0xFF;
    temp_range[9] = m_lbs.anchor_hop_count[act_anchor] & 0xFF;

    /* NOTE+TODO: Functioning wihtout ble connection */
    ret_code_t err_code = NRF_SUCCESS;

    if(m_lbs.conn_handle != BLE_CONN_HANDLE_INVALID)
        err_code = ble_lbs_ranging_update(&m_lbs, (uint8_t*) &temp_range, 10*sizeof(uint8_t));



    strcat(debugMessage, "A:");
    sprintf(debugMessage + strlen(debugMessage), "%x", src_address);    // A forceoltnál is forcoltabb castolás
    strcat(debugMessage, " D:");
    sprintf(debugMessage + strlen(debugMessage), "%d", distance);
    strcat(debugMessage, "; ");

    /* MESSAGE RACKING */
    if(isMacSendEnabled && success)
        racking_ranging(&rMessage, src_address, distance, RxQ);

    if((err_code != NRF_SUCCESS)&&(err_code != NRF_ERROR_INVALID_STATE)&&(err_code != BLE_ERROR_NO_TX_BUFFERS)&&(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }

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

        if(mpu_get_temperature(&temperature, NULL) != 0)
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
        app_timer_start(m_rxtimeout_timer_id, RANGING_RX_TIMEOUT, NULL);
    }
    else if(listen_to_beacon_cnt == 0)
    {
        deca_twr_initiator_listen_to_beacon();
        listen_to_beacon = 1;
        listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;
        app_timer_start(m_rxtimeout_timer_id, BEACON_RX_TIMEOUT, NULL);
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

void mpu_send_measurements()
{
    if(m_lbs.is_mpu_notification_enabled)
    {
        //bool altimeterMeasureCalled = false;
        //uint32_t pressure = 0;
        int fifoCount = 0;
        int16_t a[3];
        int16_t g[3];
        int16_t q[4];
        int16_t c[3];
        unsigned long actts;

        if(!isMsgMpuInit)
        {
            init_mpu_msg(&mMessage, getTagAddress());
            isMsgMpuInit = true;
        }

        do{
            fifoCount = mpu_update(g, a, q, &actts);

            if(fifoCount != -1)
            {
                /*
                mpu[0] = seqnum;
                mpu[1] = 0;
                mpu[2] = fifoCount&0xFF;
                mpu[3] = a[0]&0xFF;
                mpu[4] = (a[0]>>8)&0xFF;
                mpu[5] = a[1]&0xFF;
                mpu[6] = (a[1]>>8)&0xFF;
                mpu[7] = a[2]&0xFF;
                mpu[8] = (a[2]>>8)&0xFF;
                mpu[9] = g[0]&0xFF;
                mpu[10] = (g[0]>>8)&0xFF;
                mpu[11] = g[1]&0xFF;
                mpu[12] = (g[1]>>8)&0xFF;
                mpu[13] = g[2]&0xFF;
                mpu[14] = (g[2]>>8)&0xFF;*/

                if(mpu_get_compass_reg(c, NULL) != 0)
                {
                    for(unsigned char i = 0; i < 3; i++)
                        c[i] = 0;
                }

                if(isMacSendEnabled)
                    racking_mpu(&mMessage, a, g, c);

                uint32_t err_code = NRF_SUCCESS; //NOTE+TODO

                if(m_lbs.conn_handle != BLE_CONN_HANDLE_INVALID)
                    err_code = ble_lbs_mpu_update(&m_lbs, (uint8_t*) &mpu, 15*sizeof(uint8_t));

                if((err_code != NRF_SUCCESS)&&(err_code != NRF_ERROR_INVALID_STATE)&&(err_code != BLE_ERROR_NO_TX_BUFFERS)&&(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
                    APP_ERROR_HANDLER(err_code);

                if((mMessage.length >= 6U) && !mpuTX) //NOTE+TODO MPU_SAMPLING_RATE - működik így? nem UNSIGNED!
                {
                    mpuTX = true;
                    pts_mpu_uwb(&mMessage, macTimeStamp, mSerializedMessage);
                    mpuTX = false;
                }

                /*
                mpu[0] = seqnum;
                mpu[1] = 1;
                mpu[2] = q[0]&0xFF;
                mpu[3] = (q[0]>>8)&0xFF;
                mpu[4] = q[1]&0xFF;
                mpu[5] = (q[1]>>8)&0xFF;
                mpu[6] = q[2]&0xFF;
                mpu[7] = (q[2]>>8)&0xFF;
                mpu[8] = q[3]&0xFF;
                mpu[9] = (q[3]>>8)&0xFF;
                */

                int msg_size = 10;
                /*
                if(altimeterMeasureCount > 133 && !altimeterMeasureCalled)
                {
                    getAltimeterPressure(&pressure, &altimeterMeasureTs, &altimeterMeasureState);
                    altimeterMeasureCalled = true;

                    if(altimeterMeasureState == 0 || altimeterMeasureState == 3)
                        altimeterMeasureCount = 0;
                } */
                /*
                if(altimeterMeasureState == 3)
                {
                    mpu[10] = altimeterMeasureTs&0xFF;
                    mpu[11] = (altimeterMeasureTs>>8)&0xFF;
                    mpu[12] = (altimeterMeasureTs>>16)&0xFF;
                    mpu[13] = (altimeterMeasureTs>>24)&0xFF;
                    mpu[14] = pressure&0xFF;
                    mpu[15] = (pressure>>8)&0xFF;
                    mpu[16] = (pressure>>16)&0xFF;
                    mpu[17] = (pressure>>24)&0xFF;

                    msg_size = 18;
                    altimeterMeasureState = 0;
                }*/
                
                if(m_lbs.conn_handle != BLE_CONN_HANDLE_INVALID)
                    err_code = ble_lbs_mpu_update(&m_lbs, (uint8_t*) &mpu, msg_size*sizeof(uint8_t));

                if((err_code != NRF_SUCCESS)&&(err_code != NRF_ERROR_INVALID_STATE)&&(err_code != BLE_ERROR_NO_TX_BUFFERS)&&(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
                    APP_ERROR_HANDLER(err_code);

                seqnum++;
            }
        }while(fifoCount > 0);

        /*
        if(fifoCount != -1)
        {
            altimeterMeasureCount++;
        }*/
    }
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
    uint16_t lenbyte = length + 2;
    char output[length+3];  // CR + LF + NULL
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

    for(uint16_t i = 0; i < length; i++)
        output[i] = message[i];

    output[length] = '\r';
    output[length+1] = '\n';
    output[length+2] = '\0';

    nrf_drv_uart_tx((uint8_t const * const)outByte, 9U);

    //for(uint16_t i = 0; i < length+3; ++i)
    //   simple_uart_putstring((const uint8_t*)&output[i]);

    simple_uart_putstring((const uint8_t*) output);
    nrf_drv_uart_tx((uint8_t const * const)output, length+2U);
}
