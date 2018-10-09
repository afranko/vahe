
#ifndef SEND_TAG_UWB_H_
#define SEND_TAG_UWB_H_
#include "commons.h"

#ifdef TAG_MODE

#include <stdbool.h>
#include <stdint.h>

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "wifi_con.h"

typedef struct mac_send_ranging_package_t
{
	uint16_t 		anchorID;
	uint16_t 		distance;
	uint8_t 		RxQ;
}packetRanging;

typedef struct mac_send_ranging_message_t
{
	int8_t 			msgType;
	uint16_t 		tagID;
	uint64_t 		timeStamp;
	int8_t			length;
	packetRanging 	rpacks[10];
	int32_t			pressure;
	int32_t			temperature;
}rangingMessage;

typedef struct mac_send_mpu_measurement_package_t
{
	int16_t 		acc[3];
	int16_t			gyro[3];
	int16_t			compass[3];
}mpuPackage;

typedef struct mac_send_mpu_message_t
{
	int8_t 			msgType;
	uint16_t 		tagID;
	uint64_t 		timeStamp;
	uint8_t			length;
	mpuPackage		mpuPacks[12];	
}mpuMessage;

void init_ranging_msg(rangingMessage *msg, uint16_t tag_id);
void init_mpu_msg(mpuMessage *msg, uint16_t tag_id);
void racking_ranging(rangingMessage *msg, uint16_t anchor, uint16_t dist, uint8_t rxq);
void racking_mpu(mpuMessage *msg, int16_t accP[], int16_t gyroP[], int16_t compassP[]);
void pts_ranging_uwb(rangingMessage *msg, uint64_t tStamp, uint8_t serializedMsg[]);
void pts_mpu_uwb(mpuMessage *msg, uint64_t tStamp, uint8_t serializedMsg[]);
void serialize_ranging(rangingMessage *msg, uint8_t serializedMsg[]);
void serialize_mpu(mpuMessage *msg, uint8_t serializedMsg[]);
void set_ranging_pt(rangingMessage *msg, int32_t press, int32_t temp);
uint16_t nearestAnchor(rangingMessage *msg);
#endif
#endif	/* SEND_TAG_UWB_H_ */
