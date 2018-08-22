#include "ble_lbs.h"
#include <string.h>

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

uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{

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
	p_lbs->measurement_update_handler = p_lbs_init->measurement_update_handler;

	return 0;
}

int ble_lbs_get_buffer_count()
{
	return buffer_list.count;
}

int ble_lbs_get_buffer_drop_count()
{
	return buffer_list.drop_count;
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

	return 0;
}

uint32_t ble_lbs_ranging_update(ble_lbs_t * p_lbs, uint8_t* ranging_data, int size)
{
	if(buffer_list.count < MAX_BUFFER_ENTRIES)
	{
		buffer_list.buffer[buffer_list.wp].data_len = size;
		memcpy(buffer_list.buffer[buffer_list.wp].data, ranging_data, buffer_list.buffer[buffer_list.wp].data_len);

		buffer_list.count++;
		buffer_list.wp++;

		if(buffer_list.wp == MAX_BUFFER_ENTRIES)
			buffer_list.wp = 0;
	}
	else
		buffer_list.drop_count++;

	return 0;
}

uint32_t ble_lbs_mpu_update(ble_lbs_t * p_lbs, uint8_t* mpu_data, uint16_t len)
{
	if(buffer_list.count < MAX_BUFFER_ENTRIES)
	{
		buffer_list.buffer[buffer_list.wp].data_len = len;
		memcpy(buffer_list.buffer[buffer_list.wp].data, mpu_data, buffer_list.buffer[buffer_list.wp].data_len);

		buffer_list.count++;
		buffer_list.wp++;

		if(buffer_list.wp == MAX_BUFFER_ENTRIES)
			buffer_list.wp = 0;
	}
	else
		buffer_list.drop_count++;
	
	return 0;
}


