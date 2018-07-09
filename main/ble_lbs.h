#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define LBS_UUID_BASE {0x23, 0xD1, 0xBD, 0x9A, 0x5E, 0x78, 0x40, 0x16, 0xDE, 0xF9, 0x16, 0x17, 0x00, 0x00, 0x00, 0x00}
#define LBS_UUID_SERVICE 			0x1640
#define LBS_UUID_RANGING_CHAR 		0x1641
#define LBS_UUID_ANCHORLIST_CHAR 	0x1642
#define LBS_UUID_MPU_CHAR 			0x1643

#define MAX_LBS_NAME				32

#define MAX_ANCHORS		10

// Forward declaration of the ble_lbs_t type. 
typedef struct ble_lbs_s ble_lbs_t;

enum measurement_state_t{
	START_RANGING,
	STOP_RANGING,
	START_MPU,
	STOP_MPU,
	ANCHOR_LIST_CHANGED
};

typedef void (*measurement_update_handler_t) (ble_lbs_t * p_lbs, uint8_t new_state);

typedef struct
{
	measurement_update_handler_t measurement_update_handler;	/**< Event handler to be called when LED characteristic is written. */
} ble_lbs_init_t;

/**@brief DECA Service structure. This contains various status information for the service. */
typedef struct ble_lbs_s
{
	uint16_t                    service_handle;
	ble_gatts_char_handles_t    ranging_char_handles;
	ble_gatts_char_handles_t    anchorlist_char_handles;
	ble_gatts_char_handles_t    mpu_char_handles;
	uint8_t                     uuid_type;
	uint16_t                    conn_handle;

	uint8_t							anchorcount;
    uint16_t						anchorlist[MAX_ANCHORS];
	uint8_t							anchor_notpresence[MAX_ANCHORS];
    uint16_t						anchor_hop_address[MAX_ANCHORS];
    uint8_t							anchor_hop_count[MAX_ANCHORS];

	bool						is_ranging_notification_enabled;
	bool						is_mpu_notification_enabled;
	uint8_t                     mpu_data[6];

	measurement_update_handler_t measurement_update_handler;
} ble_lbs_t;

extern ble_lbs_t    m_lbs;

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out]  p_lbs       LED Button Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_lbs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED Button Service.
 *
 *
 * @param[in]   p_lbs      LED Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_lbs_on_ble_evt(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt);

int ble_lbs_get_buffer_count();
int ble_lbs_get_buffer_drop_count();
uint32_t ble_lbs_anchorlist_update(ble_lbs_t * p_lbs);
uint32_t ble_lbs_ranging_update(ble_lbs_t * p_lbs, uint8_t* ranging_data, int size);
uint32_t ble_lbs_mpu_update(ble_lbs_t * p_lbs, uint8_t* mpu_data, uint16_t len);

#endif // BLE_LBS_H__


