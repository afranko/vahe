#ifndef COMMONS_H_
#define COMMONS_H_

//#define CALIB
#define CALIB_ANCHOR_ADDRESS		    0x8003

#define	SERIAL_RX_PIN			    RX_PIN_NUMBER
#define	SERIAL_TX_PIN			    TX_PIN_NUMBER

#define BLE_NUS_MAX_DATA_LEN		    100
#define UART_TX_BUF_SIZE		    1						/**< UART TX buffer size. */
#define UART_RX_BUF_SIZE		    1						/**< UART RX buffer size. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT	    0						/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define HR_INC_BUTTON_PIN_NO		    BUTTON_0					/**< Button used to increment heart rate. */
#define HR_DEC_BUTTON_PIN_NO		    BUTTON_1					/**< Button used to decrement heart rate. */
#define BOND_DELETE_ALL_BUTTON_ID	    HR_DEC_BUTTON_PIN_NO			/**< Button used for deleting all bonded centrals during startup. */

#define DEVICE_NAME			    "DecaLoc"					/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME		    "BME ETIK"					/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL		    MSEC_TO_UNITS(100, UNIT_0_625_MS)		/**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS	    0						/**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER		    0						/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE		    10						/**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL	    APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)	/**< Battery level measurement interval (ticks). */
#define RANGING_MEAS_INTERVAL		    APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)	/**< Heart rate measurement interval (ticks). */
#define MPU_MEAS_INTERVAL		    APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)	/**< Heart rate measurement interval (ticks). */

#define BEACON_INTERVAL			    APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

#define TS_COUNT_INTERVAL		    APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)	/* NOTE+TODO TimeStamp */

#define RANGING_RX_TIMEOUT		    APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define BEACON_RX_TIMEOUT		    APP_TIMER_TICKS(110, APP_TIMER_PRESCALER)
#define BEACON_LISTEN_CNT		    2

#define MIN_HEART_RATE			    60						/**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE			    300						/**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_CHANGE		    2						/**< Value by which the heart rate is incremented/decremented during button press. */

#define APP_GPIOTE_MAX_USERS		    2						/**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY		    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)	/**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define MIN_CONN_INTERVAL		    MSEC_TO_UNITS(100, UNIT_1_25_MS)		/**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL		    MSEC_TO_UNITS(100, UNIT_1_25_MS)		/**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY			    0						/**< Slave latency. */
#define CONN_SUP_TIMEOUT		    MSEC_TO_UNITS(4000, UNIT_10_MS)		/**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY	    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)	/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY	    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)	/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT	    3						/**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT		    30						/**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND			    1						/**< Perform bonding. */
#define SEC_PARAM_MITM			    0						/**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES	    BLE_GAP_IO_CAPS_NONE			/**< No I/O capabilities. */
#define SEC_PARAM_OOB			    0						/**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE		    7						/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE		    16						/**< Maximum encryption key size. */

#define DEAD_BEEF			    0xDEADBEEF					/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#endif /* COMMONS_H_ */

