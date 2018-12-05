/*
 * wifi_con.h
 */

#ifndef MAIN_WIFI_CON_H_
#define MAIN_WIFI_CON_H_

#include <stdio.h>
#include <stdbool.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"

#include "commons.h"

#define SSID "KC_janiini" //TODO check if its OK
#define PASSPHARSE "kyllikki"

//#define SSID "Kobanya_Ujhegy_Free_WiFi"
//#define PASSPHARSE "szapunci2"


#define TCPServerIP "192.168.1.200" //nincs hasznalva
#define PORT 6767

int tcpSocket;
bool socket_init();//M
/*
 * wifi connection handler
 * RETURNS true, if connection is ready
 */
bool wifi_init();


/*
 * wifi + tcp socket connection initialiser
 * RETURNS boolean true, if connection is ready
 */
bool connection_init();

/*
 * this function checks
 * RETURNS
 *  - 0: no connection
 *  - 1: connected
 *  - 2: reconnected
 */
uint8_t wifi_check();

/*
 * wifi + tcp socket connection handler function
 * RETURNS boolean true, if connection is ready
 */
bool connection_check();


#endif /* MAIN_WIFI_CON_H_ */
