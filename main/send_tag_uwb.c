#include "send_tag_uwb.h"

#ifdef TAG_MODE

#include "anchor/comm_mac.h"
#include "nrf_deca.h"
#include "freertos/FreeRTOS.h"

//#ifdef RFID_MODE
	#include "rfid.h"
//#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_http_client.h"

#include "cJSON.h"

//#ifdef RFID_MODE
static int32_t globalArea = -1; //GLOBAL FSM ID
static char rfidString[15 * 3 * 12 + 1] = "\0"; //Vissza lett véve 15-re

static char* currentAreaName = NULL;
static char* currentAreaExtra = NULL;
//#endif

static uint16_t destAnchor = 0x0000;
extern int tcpSocket;

char anchor_strings[17];

//const char* TAG = "SENTAG";

static esp_http_client_config_t ips_uwb_server_config, ips_rfid_server_config;
static esp_http_client_handle_t ips_uwb_server_handler, ips_rfid_server_handler;

/*esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            ets_printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                ets_printf("%.*s", evt->data_len, (char*)evt->data);
            }
			else
			{
				ets_printf("CHUCNKOLT\n");
				ets_printf("%.*s", evt->data_len, (char*)evt->data);
			}

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}*/

void init_http_send(void)
{
	//UWB
	ips_uwb_server_config.url = "http://192.168.0.200:8080/ips-core/device/tracking/report_anchor_detections"; //RFID
	//ips_uwb_server_config.url = "http://192.168.1.96:8080/ips-core/device/tracking/report_anchor_detections";	// FINN

	ips_uwb_server_config.buffer_size = DEFAULT_HTTP_BUF_SIZE;
	ips_uwb_server_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
	ips_uwb_server_config.method = HTTP_METHOD_POST;
	ips_uwb_server_config.auth_type = HTTP_AUTH_TYPE_NONE;

	ips_uwb_server_handler = esp_http_client_init(&ips_uwb_server_config);

	//RFID
	ips_rfid_server_config.url = "http://192.168.0.201:8454/eventhandler/publish";

	ips_rfid_server_config.buffer_size = DEFAULT_HTTP_BUF_SIZE;
	ips_rfid_server_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
	ips_rfid_server_config.method = HTTP_METHOD_POST;
	ips_rfid_server_config.auth_type = HTTP_AUTH_TYPE_NONE;

	ips_rfid_server_handler = esp_http_client_init(&ips_rfid_server_config);
}

void cleanup_http_send(void)
{
	esp_http_client_cleanup(ips_uwb_server_handler);
	esp_http_client_cleanup(ips_rfid_server_handler);
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

	//uint8_t prefix[3] = {0xc5, 0xab, 0xa0};

	/* HTTP */
	cJSON *ips_uwb_server_allmsg, *det[10], *array; //max 10 ranging data

	/*EPCData oArray[ 15 ];
	uint8_t tot = 0;
	readEPCsForce(1000, &tot, oArray);
	if(tot == 0)
		ets_printf("\n\n\nNem volt RFID a kozelben vagy szar az olvasas \n\n\n\n");
	else
	{
		for (int iu = 0; iu < tot; iu++)
		{
			ets_printf("\n\n\n\n\n%d : [ ");
			for (int ju = 0; ju < EPC_LENGTH; ju++)
			{
				ets_printf("%u ", oArray[iu].bytes[ju]);
				ets_printf("_");
			}
			ets_printf("] \n\n\n\n\n\n");
		}
	}
	//ets_printf("_ \n");*/

	ips_uwb_server_allmsg = cJSON_CreateObject();
	itoa(msg->tagID, anchor_strings, 16);
	//cJSON_AddNumberToObject(ips_uwb_server_allmsg,"tagid", msg->tagID);

	for(unsigned char iter = 0; iter < 17; ++iter)
	{
		if( anchor_strings[iter] >= 97 )
		{
			anchor_strings[iter] -= 32;
		}
	}

	cJSON_AddStringToObject(ips_uwb_server_allmsg,"tagId", anchor_strings);
	cJSON_AddNumberToObject(ips_uwb_server_allmsg,"timestamp", msg->timeStamp);
	cJSON_AddNumberToObject(ips_uwb_server_allmsg,"pressure", msg->pressure);
	cJSON_AddNumberToObject(ips_uwb_server_allmsg,"temperature", msg->temperature);
	array = cJSON_AddArrayToObject(ips_uwb_server_allmsg, "detections");
	for(int k = 0; k < msg->length; k++)
	{
		cJSON_AddItemToArray(array, det[k] =cJSON_CreateObject());
		itoa(msg->rpacks[k].anchorID, anchor_strings, 16);
		for(unsigned char iter = 0; iter < 17; ++iter)
		{
			if( anchor_strings[iter] >= 97 )
			{
				anchor_strings[iter] -= 32;
			}
		}
		cJSON_AddStringToObject(det[k],"deviceId", anchor_strings);
		//cJSON_AddNumberToObject(det[k],"anchor", msg->rpacks[k].anchorID);
		cJSON_AddNumberToObject(det[k],"distance", msg->rpacks[k].distance);
	}

	ets_printf("JSON ready\n\n");
	//ets_printf("%s\n\n", cJSON_Print(ips_uwb_server_allmsg));

	/*const*/ char* ips_uwb_server_msg = cJSON_PrintUnformatted(ips_uwb_server_allmsg);
	//strcpy(ips_uwb_server_msg, "{}");

	/*for(int i = 0; i < 10; i++)
		cJSON_Delete(det[i]); //json felszab
	cJSON_Delete(array); //json felszab*/
	cJSON_Delete(ips_uwb_server_allmsg); //json felszab

//	char resp[512];
	esp_http_client_set_header(ips_uwb_server_handler, "Content-Type", "application/json");
	esp_http_client_set_post_field(ips_uwb_server_handler, ips_uwb_server_msg, strlen(ips_uwb_server_msg));

	char *buffer = malloc(DEFAULT_HTTP_BUF_SIZE + 1);
	if (buffer == NULL) {
		ets_printf("Cannot malloc http receive buffer\n");
		goto CLEAR_MESSAGE;
	}

	esp_err_t err;
	if ((err = esp_http_client_open(ips_uwb_server_handler, strlen(ips_uwb_server_msg))) != ESP_OK) {
		//ets_printf("Failed to open HTTP connection: %s\n", esp_err_to_name(err));
		free(buffer);
		goto CLEAR_MESSAGE;
	}

	esp_http_client_write(ips_uwb_server_handler, (const char*)ips_uwb_server_msg, strlen(ips_uwb_server_msg));
	//int content_length =  esp_http_client_fetch_headers(ips_uwb_server_handler);
	//int total_read_len = 0, read_len;
	esp_http_client_fetch_headers(ips_uwb_server_handler);

	//read
	esp_http_client_read(ips_uwb_server_handler, buffer, DEFAULT_HTTP_BUF_SIZE);
	ets_printf("HTTP Stream reader Status = %d, content_length = %d\n",
					esp_http_client_get_status_code(ips_uwb_server_handler),
					esp_http_client_get_content_length(ips_uwb_server_handler));
	//printf("%s\n", buffer);

	esp_http_client_close(ips_uwb_server_handler);

	free(ips_uwb_server_msg);

#ifdef RFID_MODE
	cJSON *ips_uwb_server_json_response = cJSON_Parse(buffer);
	//printf("uwb_server_json_response\n%s\n", cJSON_Print(ips_uwb_server_json_response));
	//printf("uwb_server_json_response_status\n%s\n", cJSON_Print(cJSON_GetObjectItem(ips_uwb_server_json_response, "statusCode")));

	int32_t currentArea;


	//Meg kell nézni, hogy null-t dob-e ha nincs benne
	cJSON *areaNumObject = cJSON_GetObjectItem(ips_uwb_server_json_response, "rfidSphereId");
	cJSON *areaNameObject;
	cJSON *areaExtraObject;
	if(areaNumObject == NULL)
	{
		ets_printf("Most vagyunk uresben\n");
		currentArea = -1;
	}
	else
	{
		currentArea = areaNumObject->valueint;
	}
	//cJSON_Delete(ips_uwb_server_json_response);
#endif
	free(buffer);

	//RFID READ
#ifdef RFID_MODE
	if(globalArea != currentArea)
	{

		areaNameObject = cJSON_GetObjectItem(ips_uwb_server_json_response, "name");
		areaExtraObject = cJSON_GetObjectItem(ips_uwb_server_json_response, "extra");

		//TODO ez most 100*12 byte, az jó nagy, ebből visszaveszek a biztonság kedvéért
		uint8_t tombMeret = 15;
		//rfidString[100 * 3 + 1] = "\0";
		char rfidElement[4] = "\0";
		EPCData orderedArray[ tombMeret  ];
		/*for(size_t iterator = 0; iterator < 3; ++iterator)
		{
			for (unsigned int jter = 0; jter < EPC_LENGTH; jter++)
			{
				orderedArray[iterator].bytes[jter] = (uint8_t) iterator * jter;
			}
		}*/
		//tombMeret = 3;	//TODO ez amúgy nem kell
		readEPCsForce(1000, &tombMeret, orderedArray);
		//ets_printf("%u : tombmeret \n", tombMeret);
		for (int i = 0; i < tombMeret; i++)
		{
			//ets_printf("%d : [ ");
			for (int j = 0; j < EPC_LENGTH; j++)
			{
				//ets_printf("%u ", orderedArray[i].bytes[j]);
				itoa(orderedArray[i].bytes[j], rfidElement, 10);
				strcat(rfidString, rfidElement);
				strcat(rfidString, "_");
			}
			//ets_printf("] \n");
		}
		//ets_printf("_ \n");

		//belépés előtt mindig leaveelnük egy areaból, de lehet, hogy most üres területen vagyunk
		if((globalArea != -1))	//Nem üresből megyünk bárhova
		{

			cJSON *ips_rfid_server_allmsg, *event, *eventMetadata, *source; //max 10 ranging data

			ips_rfid_server_allmsg = cJSON_CreateObject();

			//ips_rfid_server_source
			source= cJSON_CreateObject();
			cJSON_AddStringToObject(source,"systemName","SmartProduct1");
			cJSON_AddStringToObject(source,"address", "127.0.0.1");
			cJSON_AddNumberToObject(source,"port", 8080);

			//ips_rfid_server_event
			event = cJSON_CreateObject();

			eventMetadata = cJSON_CreateObject();
			cJSON_AddItemToObject(event, "eventMetadata", eventMetadata);
			cJSON_AddStringToObject(eventMetadata,"rfid", rfidString);
			cJSON_AddItemToObject(ips_rfid_server_allmsg, "source", source);



			cJSON_AddStringToObject(event,"type","area_left"); //"area_left"
			cJSON_AddStringToObject(event,"payload", currentAreaName);
			cJSON_AddStringToObject(eventMetadata,"extra", currentAreaExtra);
			cJSON_AddItemToObject(ips_rfid_server_allmsg, "event", event);

			ets_printf("RFID JSON ready - lefter\n\n");

			//ets_printf("\n\n%s\n\n", cJSON_Print(ips_rfid_server_allmsg));


			char* ips_rfid_server_msg = cJSON_PrintUnformatted(ips_rfid_server_allmsg);

			//ets_printf("\n\n%s\n\n", ips_rfid_server_msg);

			esp_http_client_set_header(ips_rfid_server_handler, "Content-Type", "application/json");
			esp_http_client_set_post_field(ips_rfid_server_handler, ips_rfid_server_msg, strlen(ips_rfid_server_msg));
			esp_http_client_perform(ips_rfid_server_handler);
			ets_printf("code %d\n", esp_http_client_get_status_code(ips_rfid_server_handler));
			free(ips_rfid_server_msg);

			//cJSON_DeleteItemFromObject(event, "type");
			//cJSON_DeleteItemFromObject(event, "payload");
			//cJSON_DeleteItemFromObject(eventMetadata, "extra");
			cJSON_Delete(ips_rfid_server_allmsg);
		}

		//bárhonnan megyünk nem üresbe
		if(currentArea != -1)
		{
			cJSON *ips_rfid_server_allmsg, *event, *eventMetadata, *source; //max 10 ranging data

			ips_rfid_server_allmsg = cJSON_CreateObject();

			//ips_rfid_server_source
			source= cJSON_CreateObject();
			cJSON_AddStringToObject(source,"systemName","SmartProduct1");
			cJSON_AddStringToObject(source,"address", "127.0.0.1");
			cJSON_AddNumberToObject(source,"port", 8080);

			//ips_rfid_server_event
			event = cJSON_CreateObject();

			eventMetadata = cJSON_CreateObject();
			cJSON_AddItemToObject(event, "eventMetadata", eventMetadata);
			cJSON_AddStringToObject(eventMetadata,"rfid", rfidString);
			cJSON_AddItemToObject(ips_rfid_server_allmsg, "source", source);


			cJSON_AddStringToObject(event,"type","area_entered");
			cJSON_AddStringToObject(event,"payload", areaNameObject->valuestring);
			cJSON_AddStringToObject(eventMetadata,"extra", areaExtraObject->valuestring);
			cJSON_AddItemToObject(ips_rfid_server_allmsg, "event", event);

			ets_printf("RFID JSON ready -- enterr\n\n");

			char* ips_rfid_server_msg = cJSON_PrintUnformatted(ips_rfid_server_allmsg);

			//ets_printf("\n\n%s\n\n", ips_rfid_server_msg);


			//ets_printf("\n\n%s\n\n", cJSON_Print(ips_rfid_server_allmsg));

			esp_http_client_set_header(ips_rfid_server_handler, "Content-Type", "application/json");
			esp_http_client_set_post_field(ips_rfid_server_handler, ips_rfid_server_msg, strlen(ips_rfid_server_msg));
			esp_http_client_perform(ips_rfid_server_handler);
			ets_printf("code %d\n", esp_http_client_get_status_code(ips_rfid_server_handler));

			free(ips_rfid_server_msg);
			cJSON_Delete(ips_rfid_server_allmsg);
		}

		globalArea = currentArea; //Meg kell jegyeznünk

		//cJSON_Delete(source); //json felszab
		//cJSON_Delete(event); //json felszab
		//cJSON_Delete(eventMetadata); //json felszab
		 //json felszab

		// Megjegyezzük az aktuális leírókat
		if(globalArea != -1)
		{
			free(currentAreaName);
			free(currentAreaExtra);
			currentAreaName = (char*) malloc(strlen(areaNameObject->valuestring) + 1);
			currentAreaExtra = (char*) malloc(strlen(areaExtraObject->valuestring) + 1);
			strcpy(currentAreaName, areaNameObject->valuestring);
			strcpy(currentAreaExtra, areaExtraObject->valuestring);
		}

		rfidString[0] = '\0';	// kinullázuk az rfid stringet
	}
	cJSON_Delete(ips_uwb_server_json_response);
#endif
	CLEAR_MESSAGE:
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
