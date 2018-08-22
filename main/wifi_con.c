#include "wifi_con.h"

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

//WIFI
void wifi_connect()
{
    wifi_config_t cfg = {
        .sta = {
            .ssid = SSID,
            .password = PASSPHARSE,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
    ESP_ERROR_CHECK( esp_wifi_connect() );

}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE); // disable wifi driver logging
    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

bool wifi_init()
{
	bool ready = false;
	wifi_ap_record_t ap_info;

	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_event_group = xEventGroupCreate();
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );
	initialise_wifi();

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, ( TickType_t ) 0x000000ffUL );
    ready = true;

	if(esp_wifi_sta_get_ap_info(&ap_info) != 0)
	{
		printf("Init: no connection!\n");
		ready = false;
	}
	else
	{
		printf("Init: connection ready!\n");
		ready = true;
	}

	return ready;
}

uint8_t wifi_check()
{
	uint8_t status = 1;
	wifi_ap_record_t ap_info;

	if(esp_wifi_sta_get_ap_info(&ap_info) != 0)
	{
		printf("Check: no connection!\n");
		ESP_ERROR_CHECK(esp_wifi_disconnect());
		printf("Check: disconnected!\n");
		ESP_ERROR_CHECK( esp_wifi_connect());
		xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, ( TickType_t ) 0x000000ffUL );//portMAX_DELAY
		if(esp_wifi_sta_get_ap_info(&ap_info) == 0)
		{
			printf("Check: reconnected!\n");
			status = 2;
		}
		else
		{
			printf("Check: reconnection was not successful!\n");
			status = 0;
		}
	}
	else
	{
		printf("Check: connected!\n");
		status = 1;
	}
	return status;
}

/*
 * returns true, if socket is ready
 */
bool socket_init()
{
	bool ready = 0;
	struct sockaddr_in tcpServerAddr;

	bzero(&tcpServerAddr, sizeof tcpServerAddr);
	tcpServerAddr.sin_addr.s_addr = htonl((((((10 << 8) | 0) << 8) | 0) << 8) | 92);//inet_addr(TCPServerIP);
	tcpServerAddr.sin_family = AF_INET;
	tcpServerAddr.sin_port = htons(PORT);

	tcpSocket = socket(AF_INET, SOCK_STREAM, 0); //socket

	//socketnyitas
	int socketready = 0;
	while(!socketready)
	{
		printf("Socketinit elso while ciklusa \n");
		xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true, (TickType_t)0x000000FFUL);
		// print the local IP address
		tcpip_adapter_ip_info_t ip_info;
		ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
		printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
		printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
		printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
		printf("\n");

		//socket
		if(tcpSocket < 0)
		{
			printf("Sock: Failed to allocate socket.\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		else
		{
			printf("Sock: allocated socket\n");
			socketready = 1;
			//printf("Sock: socket ready\n");
		}
	}

	//kapcsolodas
	int connectionready = 0;
	while(!connectionready)
	{
		printf("Socketinit masodik while ciklusa \n");
		if(connect(tcpSocket, (struct sockaddr *)&tcpServerAddr , sizeof(tcpServerAddr)) != 0) //(struct sockaddr *)&tcpServerAddr
		{
			printf("Sock: socket connect failed errno=%d \n", errno);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		else
		{
			printf("Sock: connected \n");
			connectionready = 1;
		}
	}
	ready = 1;
	return ready;
}

bool connection_init()
{
	bool ready = false;
	if(wifi_init())
	{
		printf("Con: socket allocation!\n");
		socket_init();

	}
	else
	{
		printf("Con: no connection!\n");
		ready = false;
	}

	return ready;
}

bool connection_check()
{
	bool ready = false;
	uint8_t code = wifi_check();

	if(code == 2)
	{
		printf("Con check: Socket relocation!\n");
		close(tcpSocket);
		if(socket_init())
		{
			ready = true;
		}
		else
			printf("Con check: socket fail!\n");
	}
	if(code == 1)
		ready = true;
	return ready;
}


//TEST MAIN FUNCTION
/*
void app_main()
{
	printf("main\n");
    //wifi_init();

	connection_init();

	while(1)
	{
		int i = connection_check();
		printf("i: %d\n", i);

		if(i == 1 || i ==2)
		{
			printf("sock %d\n", tcpSocket);
			uint8_t num = 0xfa;
			if(!(write(tcpSocket , (uint8_t *) &num , sizeof(num))<0))
				printf("kuldes %02x\n", num);
			else
				printf("errno = %d\n", errno);
		}
	}
}
*/
