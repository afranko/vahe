#include "send_tag_uwb.h"

#ifdef TAG_MODE

#include "anchor/comm_mac.h"
#include "nrf_deca.h"
#include "freertos/FreeRTOS.h"
#include "rfid.h"

#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_http_client.h"

#include "cJSON.h"

uint32_t globalArea = 0; //GLOBAL FSM ID

static uint16_t destAnchor = 0x0000;

extern int tcpSocket;

esp_http_client_config_t ips_uwb_server_config;


void init_http_send(void)
{
	ips_uwb_server_config.url = "http://10.0.0.111:8080/ips-core/device/tracking/report_anchor_detections";

	ips_uwb_server_config.buffer_size = DEFAULT_HTTP_BUF_SIZE;
	ips_uwb_server_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
	ips_uwb_server_config.method = HTTP_METHOD_POST;
	ips_uwb_server_config.auth_type = HTTP_AUTH_TYPE_NONE;
}

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

	//TODO WIFI -> SEND
	//Wifi.write(SERVERIP, serializedMsg);


	/*destAnchor = nearestAnchor(msg);
	if(msg->length <= 10U)
		mac_send_results(serializedMsg, 16U + msg->length * 5U, destAnchor);*/

	uint8_t prefix[3] = {0xc5, 0xab, 0xa0};

	/* HTTP */
	esp_http_client_handle_t ips_uwb_server_handler;
	ips_uwb_server_handler = esp_http_client_init(&ips_uwb_server_config);
	printf("struct hand\n");

	cJSON *root, *det[10], *array; //max 10 ranging data

	root = cJSON_CreateObject();
	cJSON_AddStringToObject(root,"tagId", msg->tagID);
	cJSON_AddNumberToObject(root,"timestamp", msg->timeStamp);
	cJSON_AddNumberToObject(root,"pressure", msg->pressure);
	cJSON_AddNumberToObject(root,"temperature", msg->temperature);
	array = cJSON_AddArrayToObject(root, "detections");
	for(int k = 0; k < msg->length; k++)
	{
		cJSON_AddItemToArray(array, det[k] =cJSON_CreateObject());
		cJSON_AddStringToObject(det[k],"deviceId",msg->rpacks[k].anchorID);
		cJSON_AddNumberToObject(det[k],"distance", msg->rpacks[k].distance);
	}

	printf("JSON ready\n\n");
	printf("%s\n\n", cJSON_Print(root));

	const char* msg_http = cJSON_PrintUnformatted(root);
//	char resp[512];
	esp_http_client_set_header(ips_uwb_server_handler, "Content-Type", "application/json");

	char *buffer = malloc(DEFAULT_HTTP_BUF_SIZE + 1);
	if (buffer == NULL) {
		printf("Cannot malloc http receive buffer\n");
		return;
	}

	esp_err_t err;
	if ((err = esp_http_client_open(ips_uwb_server_handler, strlen(msg_http))) != ESP_OK) {
		printf("Failed to open HTTP connection: %s\n", esp_err_to_name(err));
		free(buffer);
		return;
	}

	esp_http_client_write(ips_uwb_server_handler, msg_http, strlen(msg_http));
	int content_length =  esp_http_client_fetch_headers(ips_uwb_server_handler);
	int total_read_len = 0, read_len;

	/*
	 * forced read
	 */
	esp_http_client_read(ips_uwb_server_handler, buffer, DEFAULT_HTTP_BUF_SIZE);
	printf("HTTP Stream reader Status = %d, content_length = %d\n",
					esp_http_client_get_status_code(ips_uwb_server_handler),
					esp_http_client_get_content_length(ips_uwb_server_handler));
	printf("%s\n", buffer);


	cJSON *uwb_server_json_response;
	uwb_server_json_response = cJSON_Parse(buffer);
	printf("uwb_server_json_response\n%s\n", cJSON_Print(uwb_server_json_response));
	printf("uwb_server_json_response_status\n%s\n", cJSON_Print(cJSON_GetObjectItem(uwb_server_json_response, "statusCode")));

	//TODO ezeket a pointereket úgy elhagyjuk mint a picsas

	// TODO
	char *areaNum = cJSON_Print(cJSON_GetObjectItem(uwb_server_json_response, "rfidSphereId"));
	uint32_t currentArea = atoi(areaNum);
	free(areaNum);

	char *areaName = cJSON_Print(cJSON_GetObjectItem(uwb_server_json_response, "areaName")); //TODO

	esp_http_client_close(ips_uwb_server_handler);
	esp_http_client_cleanup(ips_uwb_server_handler);
	free(buffer);

	/* RFID READ */
	if(globalArea != currentArea)
	{
		uint8_t tombMeret = 100;
		char rfidString[100 * 3 + 1] = "\0";
		char rfidElement[4] = "\0";
		EPCData orderedArray[ tombMeret  ];
		readEPCsForce(1000, &tombMeret, orderedArray);
		ets_printf("%u : tombmeret \n", tombMeret);
		for (int i = 0; i < tombMeret; i++)
		{
			ets_printf("%d : [ ");
			for (int j = 0; j < EPC_LENGTH; j++)
			{
				ets_printf("%u ", orderedArray[i].bytes[j]);
				strcat(rfidString, itoa(orderedArray[i].bytes[j], rfidElement, 10));
			}
			ets_printf("] \n");
		}
		ets_printf("_ \n");

		/*
		 * rfid
		 */
		esp_http_client_config_t ips_rfid_server_config;
		ips_rfid_server_config.url = "http://152.66.246.237:8454/eventhandler/publish";
		ips_rfid_server_config.buffer_size = DEFAULT_HTTP_BUF_SIZE;
		ips_rfid_server_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
		ips_rfid_server_config.method = HTTP_METHOD_POST;
		ips_rfid_server_config.auth_type = HTTP_AUTH_TYPE_NONE;
		printf("ips_rfid_server_config kesz\n");

		esp_http_client_handle_t ips_rfid_server_handler;
		ips_rfid_server_handler = esp_http_client_init(&ips_rfid_server_config);
		printf("struct hand\n");

		cJSON *ips_rfid_server_allmsg, *event, *eventMetadata, *source; //max 10 ranging data

		ips_rfid_server_allmsg = cJSON_CreateObject();

		//ips_rfid_server_source
		source= cJSON_CreateObject();
		cJSON_AddStringToObject(source,"systemName","SmartProduct1");
		cJSON_AddStringToObject(source,"address", "127.0.0.1");
		cJSON_AddNumberToObject(source,"port", 8080);

		//Todo
		//if(enter or leave)

		//ips_rfid_server_event
		event = cJSON_CreateObject();
		if(globalArea == 0)
		{
			cJSON_AddStringToObject(event,"type","area_entered"); //"area_left"
			cJSON_AddStringToObject(event,"payload", areaName);
		}
		else
		{
			cJSON_AddStringToObject(event,"type","area_left"); //"area_left"
			cJSON_AddStringToObject(event,"payload", areaName);
		}
		eventMetadata = cJSON_CreateObject();
		cJSON_AddItemToObject(event, "eventMetadata", eventMetadata);
		cJSON_AddStringToObject(eventMetadata,"extra","some extra string value");

		cJSON_AddStringToObject(eventMetadata,"rfid", rfidString);

		cJSON_AddItemToObject(ips_rfid_server_allmsg, "source", source);
		cJSON_AddItemToObject(ips_rfid_server_allmsg, "event", event);

		//TODO FREEEEEEE

		printf("JSON ready\n\n");

		char* ips_rfid_server_msg = cJSON_PrintUnformatted(ips_rfid_server_allmsg);
		printf("ips_rfid_server_msg\n");

		esp_http_client_set_post_field(ips_rfid_server_handler, ips_rfid_server_msg, strlen(ips_rfid_server_msg));
		esp_http_client_set_header(ips_rfid_server_handler, "Content-Type", "application/json");
		esp_http_client_perform(ips_rfid_server_handler);
		printf("code %d\n", esp_http_client_get_status_code(ips_rfid_server_handler));

		free(areaName);
		free(ips_rfid_server_msg); //TODO ez a free ami elvileg kell
	}
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
#endif
