#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "cJSON.h"




//http
#include "esp_http_client.h"

#define SSID "AITIA_EMELET"
#define PSWD "sssgggaaa"

// Event group
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

// Wifi event handler
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {

    case SYSTEM_EVENT_STA_START:
    	printf("SYSTEM_EVENT_STA_START\n");
        esp_wifi_connect();
        break;

	case SYSTEM_EVENT_STA_GOT_IP:
		printf("SYSTEM_EVENT_STA_GOT_IP\n");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
		printf("SYSTEM_EVENT_STA_DISCONNECTED\n");
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;

	default:
		printf("case: %d\n", event->event_id);
        break;
    }

	return ESP_OK;
}

// Main task
void main_task(void* pvParameter)
{
	// wait for connection
	printf("waiting connected bit\n");

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	printf("connected!\n");
	printf("\n");

	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway     %s\n", ip4addr_ntoa(&ip_info.gw));
	printf("\n");


	/*
	 * uwb
	 */
	esp_http_client_config_t ips_uwb_server_config;
	ips_uwb_server_config.url = "http://10.0.0.111:8080/ips-core/device/tracking/report_anchor_detections";

	ips_uwb_server_config.buffer_size = DEFAULT_HTTP_BUF_SIZE;
	ips_uwb_server_config.transport_type = HTTP_TRANSPORT_OVER_TCP;
	ips_uwb_server_config.method = HTTP_METHOD_POST;
	ips_uwb_server_config.auth_type = HTTP_AUTH_TYPE_NONE;
	printf("ips_uwb_server_config kesz\n");

	esp_http_client_handle_t ips_uwb_server_handler;
	ips_uwb_server_handler = esp_http_client_init(&ips_uwb_server_config);
	printf("struct hand\n");

	cJSON *root, *det1, *det2, *det3, *array;
	root=cJSON_CreateObject();
	cJSON_AddStringToObject(root,"tagId","4");
	cJSON_AddNumberToObject(root,"timestamp", 1256953732);
	cJSON_AddNumberToObject(root,"pressure", 12);
	cJSON_AddNumberToObject(root,"temperature", 40);
	array = cJSON_AddArrayToObject(root, "detections");
	cJSON_AddItemToArray(array, det1 =cJSON_CreateObject());
	cJSON_AddStringToObject(det1,"deviceId","DD1");
	cJSON_AddNumberToObject(det1,"distance", 50);
	cJSON_AddItemToArray(array, det2 =cJSON_CreateObject());
	cJSON_AddStringToObject(det2,"deviceId","DD2");
	cJSON_AddNumberToObject(det2,"distance", 32);
	cJSON_AddItemToArray(array, det3 =cJSON_CreateObject());
	cJSON_AddStringToObject(det3,"deviceId","DD3");
	cJSON_AddNumberToObject(det3,"distance", 50);
	printf("JSON\n\n");

	const char* msg = cJSON_PrintUnformatted(root);
//	char resp[512];

	printf("ips_uwb_server_msg\n%s\n", cJSON_Print(root));
	esp_http_client_set_header(ips_uwb_server_handler, "Content-Type", "application/json");

	/*
	 * perform
	 */

	char *buffer = malloc(DEFAULT_HTTP_BUF_SIZE + 1);
	if (buffer == NULL) {
		printf("Cannot malloc http receive buffer\n");
		return;
	}

	esp_err_t err;
	if ((err = esp_http_client_open(ips_uwb_server_handler, strlen(msg))) != ESP_OK) {
		printf("Failed to open HTTP connection: %s\n", esp_err_to_name(err));
		free(buffer);
		return;
	}
	//esp_http_client_set_post_field(ips_uwb_server_handler, msg, strlen(msg+1));

	printf("%s\n", msg);
	esp_http_client_write(ips_uwb_server_handler, msg, strlen(msg));
	int content_length =  esp_http_client_fetch_headers(ips_uwb_server_handler);
	int total_read_len = 0, read_len;
//	if (total_read_len < content_length && content_length <= DEFAULT_HTTP_BUF_SIZE) {
//		read_len = esp_http_client_read(ips_uwb_server_handler, buffer, content_length);
//		if (read_len <= 0) {
//			printf("Error read data\n");
//		}
//		buffer[read_len] = 0;
//		printf( "read_len = %d\n", read_len);
//	}

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


	esp_http_client_close(ips_uwb_server_handler);
	esp_http_client_cleanup(ips_uwb_server_handler);
	free(buffer);


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

	//ips_rfid_server_event
	event = cJSON_CreateObject();
	cJSON_AddStringToObject(event,"type","area_entered"); //"area_left"
	cJSON_AddStringToObject(event,"payload", "area_string_name");
	eventMetadata = cJSON_CreateObject();
	cJSON_AddItemToObject(event, "eventMetadata", eventMetadata);
	cJSON_AddStringToObject(eventMetadata,"extra","some extra string value");
	cJSON_AddStringToObject(eventMetadata,"rfid","barmi amit marci adott");

	cJSON_AddItemToObject(ips_rfid_server_allmsg, "source", source);
	cJSON_AddItemToObject(ips_rfid_server_allmsg, "event", event);


	printf("JSON ready\n\n");

	const char* ips_rfid_server_msg = cJSON_PrintUnformatted(ips_rfid_server_allmsg);
	printf("ips_rfid_server_msg\n");

	esp_http_client_set_post_field(ips_rfid_server_handler, ips_rfid_server_msg, strlen(ips_rfid_server_msg));
	esp_http_client_set_header(ips_rfid_server_handler, "Content-Type", "application/json");
	esp_http_client_perform(ips_rfid_server_handler);
	printf("code %d\n", esp_http_client_get_status_code(ips_rfid_server_handler));



//	esp_http_client_set_post_field(ips_rfid_server_handler, ips_rfid_server_msg_event, strlen(ips_rfid_server_msg_event));
//	esp_http_client_set_header(ips_rfid_server_handler, "Content-Type", "application/json");
//	esp_http_client_perform(ips_rfid_server_handler);
//
//	printf("code %d\n", esp_http_client_get_status_code(ips_rfid_server_handler));


	while(1)
	{
		//printf("d");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

}


// Main application
void app_main()
{
	 //disable the default wifi logging

	//grg
	printf("fut\n");


//	esp_log_level_set(wifi, ESP_LOG_NONE);

//	grg
	 printf("loglevel\n");

//	 create the event group to handle wifi events
	wifi_event_group = xEventGroupCreate();

//	 initialize the tcp stack
	tcpip_adapter_init();

//	grg
	printf("tcpinit\n");

//	 initialize the wifi event handler
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

//	initialize NVS
	esp_err_t ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK( ret );
//	 initialize the wifi stack in STAtion mode with config in RAM
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));



//	 configure the wifi connection and start the interface
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PSWD,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
	printf("Connecting to %s...\n" , SSID);



//	 start the main task
    xTaskCreate(&main_task, "main_task", 4096, NULL, 5, NULL);
}
