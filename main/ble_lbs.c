#include "ble_lbs.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

/*
static uint8_t ranging_char_name[MAX_LBS_NAME]= "Ranging";
static uint8_t anchorlist_char_name[MAX_LBS_NAME]= "Anchor List";
static uint8_t mpu_char_name[MAX_LBS_NAME]= "MPU data";
 */

ble_lbs_t    m_lbs;

#define MAX_BUFFER_ENTRIES	20

typedef struct
{
	uint16_t							data_len;	/**< Total length of data */
	ble_gatts_char_handles_t*	handle;		/**< Identifies peer and service instance */
	uint8_t							data[20];	/**< Scanned key pattern */
} buffer_entry_t;


typedef struct
{
	buffer_entry_t	buffer[MAX_BUFFER_ENTRIES];	/**< Maximum number of entries that can enqueued in the list */
	uint8_t			rp;									/**< Index to the read location */
	uint8_t			wp;									/**< Index to write location */
	int				count;								/**< Number of elements in the list */
	int				drop_count;
} buffer_list_t;


static buffer_list_t buffer_list;


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_lbs       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
	buffer_list.rp = 0;
	buffer_list.wp = 0;
	buffer_list.count = 0;
	buffer_list.drop_count = 0;

	p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_lbs       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
	
	//p_lbs->is_ranging_notification_enabled = false; //NOTE+TODO
	if(p_lbs->measurement_update_handler != NULL)
		p_lbs->measurement_update_handler(p_lbs, STOP_RANGING);
		
	//p_lbs->is_mpu_notification_enabled = false; //NOTE+TODO
	if(p_lbs->measurement_update_handler != NULL)
		p_lbs->measurement_update_handler(p_lbs, STOP_MPU);
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_lbs       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_lbs->anchorlist_char_handles.value_handle)
	{
		p_lbs->anchorcount = p_evt_write->len/2;
		if(p_lbs->measurement_update_handler != NULL)
			p_lbs->measurement_update_handler(p_lbs, ANCHOR_LIST_CHANGED);
	}

	if ((p_evt_write->handle == p_lbs->ranging_char_handles.cccd_handle) && (p_evt_write->len == 2))
	{
		if (ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_lbs->is_ranging_notification_enabled = true;
			if(p_lbs->measurement_update_handler != NULL)
				p_lbs->measurement_update_handler(p_lbs, START_RANGING);
		}
		else
		{
			p_lbs->is_ranging_notification_enabled = false;
			if(p_lbs->measurement_update_handler != NULL)
				p_lbs->measurement_update_handler(p_lbs, STOP_RANGING);
		}
	}
	if ((p_evt_write->handle == p_lbs->mpu_char_handles.cccd_handle) && (p_evt_write->len == 2))
	{
		if (ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_lbs->is_mpu_notification_enabled = true;
			if(p_lbs->measurement_update_handler != NULL)
				p_lbs->measurement_update_handler(p_lbs, START_MPU);
		}
		else
		{
			p_lbs->is_mpu_notification_enabled = false;
			if(p_lbs->measurement_update_handler != NULL)
				p_lbs->measurement_update_handler(p_lbs, STOP_MPU);
		}
	}
}


void ble_lbs_on_ble_evt(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_lbs, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_lbs, p_ble_evt);
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(p_lbs, p_ble_evt);
		break;

	default:
		// No implementation needed.
		break;
	}
}


/**@brief Function for adding the LED characteristic.
 *
 */
static uint32_t ranging_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;//, userdesc_md;
	/*
	memset(&userdesc_md, 0, sizeof(userdesc_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&userdesc_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&userdesc_md.write_perm);
	userdesc_md.vloc       = BLE_GATTS_VLOC_STACK;
	userdesc_md.rd_auth    = 0;
	userdesc_md.wr_auth    = 0;
	userdesc_md.vlen       = 0;
	 */
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read   = 1;
	char_md.char_props.write  = 0;
	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;//(unsigned char *)ranging_char_name;
	//	char_md.char_user_desc_max_size = MAX_LBS_NAME;
	//	char_md.char_user_desc_size = strlen((char *)ranging_char_name);
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL; //&userdesc_md;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = p_lbs->uuid_type;
	ble_uuid.uuid = LBS_UUID_RANGING_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid       = &ble_uuid;
	attr_char_value.p_attr_md    = &attr_md;
	attr_char_value.init_len     = 0;
	attr_char_value.init_offs    = 0;
	attr_char_value.max_len      = 20;
	attr_char_value.p_value      = NULL;

	return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
			&attr_char_value,
			&p_lbs->ranging_char_handles);
}

/**@brief Function for adding the Button characteristic.
 *
 */
static uint32_t anchorlist_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;//, userdesc_md;
	/*
	memset(&userdesc_md, 0, sizeof(userdesc_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&userdesc_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&userdesc_md.write_perm);
	userdesc_md.vloc       = BLE_GATTS_VLOC_STACK;
	userdesc_md.rd_auth    = 0;
	userdesc_md.wr_auth    = 0;
	userdesc_md.vlen       = 0;
	 */
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read   = 1;
	char_md.char_props.write  = 1;
	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;//(unsigned char *)anchorlist_char_name;
	//	char_md.char_user_desc_max_size = MAX_LBS_NAME;
	//	char_md.char_user_desc_size = strlen((char *)anchorlist_char_name);
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;//&userdesc_md;
	char_md.p_cccd_md         = NULL;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = p_lbs->uuid_type;
	ble_uuid.uuid = LBS_UUID_ANCHORLIST_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_USER;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid       = &ble_uuid;
	attr_char_value.p_attr_md    = &attr_md;
	attr_char_value.init_len     = 0;
	attr_char_value.init_offs    = 0;
	attr_char_value.max_len      = 20;
	attr_char_value.p_value      = (uint8_t*) &p_lbs->anchorlist[0];

	return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
			&attr_char_value,
			&p_lbs->anchorlist_char_handles);
}

static uint32_t mpu_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;//, userdesc_md;
	/*
	memset(&userdesc_md, 0, sizeof(userdesc_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&userdesc_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&userdesc_md.write_perm);
	userdesc_md.vloc       = BLE_GATTS_VLOC_STACK;
	userdesc_md.rd_auth    = 0;
	userdesc_md.wr_auth    = 0;
	userdesc_md.vlen       = 0;
	 */
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read   = 1;
	char_md.char_props.write  = 0;
	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;//(unsigned char *)mpu_char_name;
	//	char_md.char_user_desc_max_size = MAX_LBS_NAME;
	//	char_md.char_user_desc_size = strlen((char *)mpu_char_name);
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;//&userdesc_md;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	ble_uuid.type = p_lbs->uuid_type;
	ble_uuid.uuid = LBS_UUID_MPU_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid       = &ble_uuid;
	attr_char_value.p_attr_md    = &attr_md;
	attr_char_value.init_len     = 0;
	attr_char_value.init_offs    = 0;
	attr_char_value.max_len      = 20;
	attr_char_value.p_value      = NULL;

	return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
			&attr_char_value,
			&p_lbs->mpu_char_handles);
}


uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;

	for(int i=0; i < MAX_ANCHORS; i++)
	{
		p_lbs->anchorlist[i] = 0;
		p_lbs->anchor_notpresence[i] = 0;
        p_lbs->anchor_hop_address[i] = 0;
        p_lbs->anchor_hop_count[i] = 0;
	}
	p_lbs->anchorcount = 0;

	p_lbs->is_ranging_notification_enabled = true; //NOTE+TODO
	p_lbs->is_mpu_notification_enabled = true; //NOTE+TODO

	// Initialize service structure
	p_lbs->conn_handle       = BLE_CONN_HANDLE_INVALID;
	p_lbs->measurement_update_handler = p_lbs_init->measurement_update_handler;

	// Add service
	ble_uuid128_t base_uuid = {LBS_UUID_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->uuid_type);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	ble_uuid.type = p_lbs->uuid_type;
	ble_uuid.uuid = LBS_UUID_SERVICE;

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = ranging_char_add(p_lbs, p_lbs_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = anchorlist_char_add(p_lbs, p_lbs_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = mpu_char_add(p_lbs, p_lbs_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	ble_gatts_value_t anchorlist_val;
	anchorlist_val.len=0;
	anchorlist_val.offset=0;
	anchorlist_val.p_value=NULL;
	sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, p_lbs->anchorlist_char_handles.value_handle, &anchorlist_val);

	return NRF_SUCCESS;
}

int ble_lbs_get_buffer_count()
{
	return buffer_list.count;
}

int ble_lbs_get_buffer_drop_count()
{
	return buffer_list.drop_count;
}

void ble_lbs_send_entries(ble_lbs_t * p_lbs)
{
	uint32_t err_code;
	ble_gatts_hvx_params_t params;

	while(buffer_list.count > 0)
	{
		memset(&params, 0, sizeof(params));
		params.type = BLE_GATT_HVX_NOTIFICATION;
		params.handle = buffer_list.buffer[buffer_list.rp].handle->value_handle;
		params.p_data = buffer_list.buffer[buffer_list.rp].data;
		params.p_len = &(buffer_list.buffer[buffer_list.rp].data_len);

		err_code = sd_ble_gatts_hvx(p_lbs->conn_handle, &params);

		if(err_code == NRF_SUCCESS)
		{
			buffer_list.rp++;
			buffer_list.count--;

			if(buffer_list.rp == MAX_BUFFER_ENTRIES)
				buffer_list.rp = 0;
		}
		else
			break;
	}
}

uint32_t ble_lbs_anchorlist_update(ble_lbs_t * p_lbs)
{
	if(buffer_list.count < MAX_BUFFER_ENTRIES)
	{
/*
		uint16_t temp_list[10];
	
		int j = 0;
		for(int i=0; i < MAX_ANCHORS; i++)
		{
			if(p_lbs->anchorlist[i] != 0)
			{
				temp_list[j] = p_lbs->anchorlist[i];
				j++;
			}
		}
*/		
		buffer_list.buffer[buffer_list.wp].handle = &(p_lbs->anchorlist_char_handles);
		buffer_list.buffer[buffer_list.wp].data_len = (p_lbs->anchorcount)*sizeof(uint16_t);
		memcpy(buffer_list.buffer[buffer_list.wp].data, p_lbs->anchorlist, buffer_list.buffer[buffer_list.wp].data_len);
	
/*
		int j = 0;
		for(int i=0; i < MAX_ANCHORS; i++)
		{
			if(p_lbs->anchorlist[i] != 0)
			{
				memcpy(buffer_list.buffer[buffer_list.wp].data+2*j, &(p_lbs->anchorlist[i]), sizeof(uint16_t));

				j++;
			}
		}
*/	
		

		buffer_list.count++;
		buffer_list.wp++;

		if(buffer_list.wp == MAX_BUFFER_ENTRIES)
			buffer_list.wp = 0;
	}
	else
		buffer_list.drop_count++;

	ble_lbs_send_entries(p_lbs);

	return NRF_SUCCESS;
}

uint32_t ble_lbs_ranging_update(ble_lbs_t * p_lbs, uint8_t* ranging_data, int size)
{
	if(buffer_list.count < MAX_BUFFER_ENTRIES)
	{
		buffer_list.buffer[buffer_list.wp].handle = &(p_lbs->ranging_char_handles);
		buffer_list.buffer[buffer_list.wp].data_len = size;
		memcpy(buffer_list.buffer[buffer_list.wp].data, ranging_data, buffer_list.buffer[buffer_list.wp].data_len);

		buffer_list.count++;
		buffer_list.wp++;

		if(buffer_list.wp == MAX_BUFFER_ENTRIES)
			buffer_list.wp = 0;
	}
	else
		buffer_list.drop_count++;

	ble_lbs_send_entries(p_lbs);

	return NRF_SUCCESS;
}

uint32_t ble_lbs_mpu_update(ble_lbs_t * p_lbs, uint8_t* mpu_data, uint16_t len)
{
	if(buffer_list.count < MAX_BUFFER_ENTRIES)
	{
		buffer_list.buffer[buffer_list.wp].handle = &(p_lbs->mpu_char_handles);
		buffer_list.buffer[buffer_list.wp].data_len = len;
		memcpy(buffer_list.buffer[buffer_list.wp].data, mpu_data, buffer_list.buffer[buffer_list.wp].data_len);

		buffer_list.count++;
		buffer_list.wp++;

		if(buffer_list.wp == MAX_BUFFER_ENTRIES)
			buffer_list.wp = 0;
	}
	else
		buffer_list.drop_count++;

	ble_lbs_send_entries(p_lbs);
	
	return NRF_SUCCESS;
}


