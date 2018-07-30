#include "send_tag_uwb.h"
#include "nrf_deca.h"

static uint16_t destAnchor = 0x0000;

void init_ranging_msg(rangingMessage *msg, uint16_t tag_id)
{
	msg->msgType = 0x00;
	msg->tagID = tag_id;
	msg->length = 0;
}

void init_mpu_msg(mpuMessage *msg, uint16_t tag_id)
{
	msg->msgType = 0xFF;
	msg->tagID = tag_id;
	msg->length = 0;
}

void racking_ranging(rangingMessage *msg, uint16_t anchor, uint16_t dist, uint8_t rxq)
{

	msg->rpacks[msg->length].anchorID = anchor;
	msg->rpacks[msg->length].distance = dist;
	msg->rpacks[msg->length].RxQ = rxq;
	msg->length++;
}

void racking_mpu(mpuMessage *msg, int16_t accP[], int16_t gyroP[], int16_t compassP[])
{
	for(uint8_t i = 0; i < 3; i++)
	{
		msg->mpuPacks[msg->length].acc[i] = accP[i];
		msg->mpuPacks[msg->length].gyro[i] = gyroP[i];
		msg->mpuPacks[msg->length].compass[i] = compassP[i];
	}
	msg->length++;
}

void set_ranging_pt(rangingMessage *msg, int32_t press, int32_t temp)
{
	msg->pressure = press;
	msg->temperature = temp;
}

/* Prepare to send functions (PTS) */

void pts_ranging_uwb(rangingMessage *msg, uint64_t tStamp, uint8_t serializedMsg[])
{
	msg->timeStamp = tStamp;
	serialize_ranging(msg, serializedMsg);

	//#error "0xDEAD a prefix! WiFi modulban át kell állítani"

	destAnchor = nearestAnchor(msg);
	if(msg->length <= 10U)
		mac_send_results(serializedMsg, 16U + msg->length * 5U, destAnchor);

	msg->length = 0;
}

void pts_mpu_uwb(mpuMessage *msg, uint64_t tStamp, uint8_t serializedMsg[])
{
	msg->timeStamp = tStamp;
	serialize_mpu(msg, serializedMsg);

	if(msg->length <= 6U)
		mac_send_results(serializedMsg, 7U + msg->length * 18U, destAnchor);

	msg->length = 0;
}

void serialize_ranging(rangingMessage *msg, uint8_t serializedMsg[])
{
	uint8_t i = 0, j;

	//serializedMsg[i++] = 0xDE;
	//serializedMsg[i++] = 0xAD;
	serializedMsg[i++] = msg->msgType;
	serializedMsg[i++] = (uint8_t) msg->length;
	serializedMsg[i++] = (uint8_t) (msg->tagID >> 8);
	serializedMsg[i++] = (uint8_t) msg->tagID;
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 24);
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 16);
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 8);
	serializedMsg[i++] = (uint8_t) msg->timeStamp;

	serializedMsg[i++] = (uint8_t) (msg->pressure >> 24);
	serializedMsg[i++] = (uint8_t) (msg->pressure >> 16);
	serializedMsg[i++] = (uint8_t) (msg->pressure >> 8);
	serializedMsg[i++] = (uint8_t) msg->pressure;
	serializedMsg[i++] = (uint8_t) (msg->temperature >> 24);
	serializedMsg[i++] = (uint8_t) (msg->temperature >> 16);
	serializedMsg[i++] = (uint8_t) (msg->temperature >> 8);
	serializedMsg[i++] = (uint8_t) msg->temperature;
	
	for(j = 0; j < msg->length; j++)
	{
		serializedMsg[i++] = (uint8_t) (msg->rpacks[j].anchorID >> 8);
		serializedMsg[i++] = (uint8_t) (msg->rpacks[j].anchorID);
		serializedMsg[i++] = (uint8_t) (msg->rpacks[j].distance >> 8);
		serializedMsg[i++] = (uint8_t) (msg->rpacks[j].distance);
		serializedMsg[i++] = (uint8_t) (msg->rpacks[j].RxQ);
	}
}

void serialize_mpu(mpuMessage *msg, uint8_t serializedMsg[])
{
	uint8_t i = 0, j = 0, k = 0;

	//serializedMsg[i++] = 0xDE;
	//serializedMsg[i++] = 0xAD;
	serializedMsg[i++] = msg->msgType;
	serializedMsg[i++] = (uint8_t) (msg->tagID >> 8);
	serializedMsg[i++] = (uint8_t) msg->tagID;
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 24);
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 16);
	serializedMsg[i++] = (uint8_t) (msg->timeStamp >> 8);
	serializedMsg[i++] = (uint8_t) msg->timeStamp;

	for(j = 0; j < msg->length; j++)
	{
		for (k = 0; k < 3; k++)
		{
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].acc[k] >> 8);
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].acc[k]);
		}

		
		for (k = 0; k < 3; k++)
		{
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].gyro[k] >> 8);
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].gyro[k]);
		}
		

		for (k = 0; k < 3; k++)
		{
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].compass[k] >> 8);
			serializedMsg[i++] = (uint8_t) (msg->mpuPacks[j].compass[k]);
		}
	}
}

uint16_t nearestAnchor(rangingMessage *msg)
{
	uint16_t anchor = msg->rpacks[0].anchorID;
	uint16_t distAnchor = msg->rpacks[0].distance;

	for(uint8_t i = 1; i < msg->length; i++)
		anchor = distAnchor < msg->rpacks[i].distance ? anchor : msg->rpacks[i].anchorID;

	return anchor;
}
