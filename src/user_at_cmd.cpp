/**
 * @file user_at_cmd.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief User AT commands
 * @version 0.1
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main.h"

#ifdef NRF52_SERIES
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

/** Filename to save master node address */
static const char master_node_file_name[] = "MAST";

/** File to save battery check status */
File master_node_file(InternalFS);
#endif
#ifdef ESP32
#include <Preferences.h>
/** ESP32 preferences */
Preferences esp32_prefs;
#endif

/*****************************************
 * Set Master Node Mesh Address
 *****************************************/

/**
 * @brief Set Master Node Mesh Address
 *
 * @param str Address as Hex String
 * @return int AT_SUCCESS if ok, AT_ERRNO_PARA_FAIL if invalid value
 */
int at_set_master_node(char *str)
{
	if (strlen(str) != 8)
	{
		return AT_ERRNO_PARA_NUM;
	}
	uint32_t new_addr = (uint32_t)strtoul(str, NULL, 16);

	MYLOG("USR_AT", "Received new master node address %08lX", new_addr);
	if ((new_addr == 0) || (new_addr == g_this_device_addr))
	{
		return AT_ERRNO_PARA_NUM;
	}

	bool need_save = g_master_node_addr != new_addr ? true : false;

	g_master_node_addr = new_addr;

	// Save new master node address if changed
	if (need_save)
	{
		save_master_node();
	}
	return AT_SUCCESS;
}

/**
 * @brief Get Master Node Mesh Address
 *
 * @return int AT_SUCCESS
 */
int at_query_master_node(void)
{
	snprintf(g_at_query_buf, ATQUERY_SIZE, "%08lX", g_master_node_addr);
	return AT_SUCCESS;
}

/**
 * @brief Get this Node Mesh Address
 *
 * @return int AT_SUCCESS
 */
int at_query_node(void)
{
	snprintf(g_at_query_buf, ATQUERY_SIZE, "%08lX", g_this_device_addr);
	return AT_SUCCESS;
}

/**
 * @brief Get Mesh map
 *
 * @return int
 */
int at_query_mesh(void)
{
	/** Node ID of the selected receiver node */
	uint32_t node_id[48];
	/** First hop ID of the selected receiver node */
	uint32_t first_hop[48];
	/** Number of hops to the selected receiver node */
	uint8_t num_hops[48];
	/** Number of nodes in the map */
	uint8_t num_elements;

	if (xSemaphoreTake(access_node_list_sem, (TickType_t)1000) == pdTRUE)
	{
		/** Number of nodes in the map */
		num_elements = nodes_in_map();

		for (int idx = 0; idx < num_elements; idx++)
		{
			get_node(idx, node_id[idx], first_hop[idx], num_hops[idx]);
		}
		// Release access to nodes list
		xSemaphoreGive(access_node_list_sem);

		// Display the nodes
		int len = snprintf(&g_at_query_buf[len], ATQUERY_SIZE - len, "\n#01: %08lX ^^", g_this_device_addr);
		for (int idx = 0; idx < num_elements; idx++)
		{
			if (first_hop[idx] == 0)
			{
				len += snprintf(&g_at_query_buf[len], ATQUERY_SIZE - len, "\n#%02d: %08lX <->", idx + 2, node_id[idx]);
			}
			else
			{
				len += snprintf(&g_at_query_buf[len], ATQUERY_SIZE - len, "\n#%02d: %08lX ->%08lX #h: %d", idx + 2, node_id[idx], first_hop[idx], num_hops[idx]);
			}
		}
	}
	else
	{
		return AT_ERRNO_EXEC_FAIL;
	}

	return AT_SUCCESS;
}

/**
 * @brief List of all available commands with short help and pointer to functions
 *
 */
atcmd_t g_user_at_cmd_list_mesh[] = {
	/*|    CMD    |     AT+CMD?      |    AT+CMD=?    |  AT+CMD=value |  AT+CMD  | Permissions |*/
	// Module commands
	{"+MASTER", "Set/get Master Node address", at_query_master_node, at_set_master_node, NULL, "RW"},
	{"+NODE", "Get this Node address", at_query_node, NULL, NULL, "R"},
	{"+MESH", "Get the mesh map", at_query_mesh, NULL, NULL, "R"},
};

/**
 * @brief Read saved Master Node address
 *
 */
void read_master_node(void)
{
#ifdef NRF52_SERIES
	if (InternalFS.exists(master_node_file_name))
	{
		longlong_byte_u buffer;

		master_node_file.open(master_node_file_name, FILE_O_READ);
		master_node_file.read((void *)buffer.b_values, 4);
		master_node_file.close();
		g_master_node_addr = buffer.l_value;
		MYLOG("USR_AT", "File found, master node address is %08lX", g_master_node_addr);
	}
	else
	{
		g_master_node_addr = 0x00000000;
		MYLOG("USR_AT", "File not found, set master node address to 0x00000000");
	}
#endif
#ifdef ESP32
	esp32_prefs.begin("mn", false);
	g_master_node_addr = esp32_prefs.getULong("mn", 0x00000000);
	esp32_prefs.end();
#endif
}

/**
 * @brief Save the Master Node address
 *
 */
void save_master_node(void)
{
#ifdef NRF52_SERIES
	if (InternalFS.exists(master_node_file_name))
	{
		InternalFS.remove(master_node_file_name);
	}

	longlong_byte_u buffer;
	buffer.l_value = g_master_node_addr;

	master_node_file.open(master_node_file_name, FILE_O_WRITE);
	master_node_file.write((const char *)buffer.b_values, 4);
	master_node_file.close();
	MYLOG("USR_AT", "Saved master node address %08lX", g_master_node_addr);
#endif
#ifdef ESP32
	esp32_prefs.begin("mn", false);
	esp32_prefs.putULong("mn", g_master_node_addr);
	esp32_prefs.end();
#endif
}

/** Number of user defined AT commands */
uint8_t g_user_at_cmd_num = 0;

/** Pointer to the combined user AT command structure */
atcmd_t *g_user_at_cmd_list;

/**
 * @brief Initialize the user defined AT command list
 *
 */
void init_user_at(void)
{
	// uint16_t index_next_cmds = 0;
	// uint16_t required_structure_size = sizeof(g_user_at_cmd_list_mesh);
	// MYLOG("USR_AT", "Structure size %d Mesh", required_structure_size);

	// Reserve memory for the structure
	// g_user_at_cmd_list = (atcmd_t *)malloc(required_structure_size);
	g_user_at_cmd_list = g_user_at_cmd_list_mesh;

	// Add AT commands to structure
	MYLOG("USR_AT", "Adding Mesh AT commands");
	g_user_at_cmd_num += sizeof(g_user_at_cmd_list_mesh) / sizeof(atcmd_t);
	// memcpy((void *)&g_user_at_cmd_list[index_next_cmds], (void *)g_user_at_cmd_list_mesh, sizeof(g_user_at_cmd_list_mesh));
	// index_next_cmds += sizeof(g_user_at_cmd_list_mesh) / sizeof(atcmd_t);
	// MYLOG("USR_AT", "Index after adding Mesh ccommands %d", index_next_cmds);

	has_custom_at = true;
}
