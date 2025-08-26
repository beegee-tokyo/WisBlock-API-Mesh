/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Main application for Mesh Network PoC
 * @version 0.1
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main.h"

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "RAK-MESH";

/** Buffer for BLE/Mesh data */
char data_buffer[512] = {0};

/** Master Node address */
uint32_t g_master_node_addr = 0x00000000;

/** Counter, only for testing */
uint64_t msg_cnt = 0;

/** Union to convert uint32_t into uint8_t array */
longlong_byte_u convert_value;

/** Flag if OLED was found */
bool has_rak1921 = false;

/** Buffer for OLED output */
char line_str[256];

/**
   @brief Application specific setup functions

*/
void setup_app(void)
{
	pinMode(LED_BLUE, OUTPUT);

	// Initialize Serial for debug output
	Serial.begin(115200);

	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
		}
		else
		{
			break;
		}
	}

	/************************************************************/
	/** This code works only in LoRa P2P mode.                  */
	/** Forcing here the usage of LoRa P2P, independant of      */
	/** saved settings.                                         */
	/************************************************************/
	// Read LoRaWAN settings from flash
	api_read_credentials();
	// Force LoRa P2P
	g_lorawan_settings.lorawan_enable = false;
	//////////////////////////////////////////////
	// Below is optional forcing all devices to the same P2P settings
	//////////////////////////////////////////////
	// // Force frequency setting to 916.0 MHz
	// g_lorawan_settings.p2p_frequency = 916000000;
	// // Force bandwidth setting to 125 MHz
	// g_lorawan_settings.p2p_bandwidth = 0; // 125 Mhz
	// // Force spreading factor setting to SF7
	// g_lorawan_settings.p2p_sf = 7;
	// // Force coding rate setting to CR 4/5
	// g_lorawan_settings.p2p_cr = 1;
	// // Force preamble length setting to 8 symbols
	// g_lorawan_settings.p2p_preamble_len = 8;
	// // Force symbol timeout setting to 0 symbols
	// g_lorawan_settings.p2p_symbol_timeout = 0;
	// Save LoRaWAN settings
	api_set_credentials();

	// Set Application version
	snprintf(g_custom_fw_ver, 63, "WisBlock Mesh V%d.%d.%d", SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);

#ifdef NRF52_SERIES
	// Enable BLE
	g_enable_ble = true;
#endif
}

/**
   @brief Application specific initializations

   @return true Initialization success
   @return false Initialization failure
*/
bool init_app(void)
{
	MYLOG("APP", "init_app");

	init_user_at();
	read_master_node();

	has_rak1921 = init_rak1921();

	float batt = read_batt();
	for (int rd_lp = 0; rd_lp < 10; rd_lp++)
	{
		batt += read_batt();
		batt = batt / 2;
	}
	if (has_rak1921)
	{
		sprintf(line_str, "RUI3 Mesh - B %.2fV", batt / 1000);
		rak1921_write_header(line_str);
	}
	else
	{
		MYLOG("APP", "No OLED found");
	}

	Serial.println("*****************************************");
	Serial.println("WisBlock API Mesh");
	Serial.println("*****************************************");
	Serial.println("** All mesh nodes MUST be set to the same");
	Serial.println("** LoRa frequency                        ");
	Serial.println("** Bandwidth                             ");
	Serial.println("** Spreading factor                      ");
	Serial.println("** Coding rate                           ");
	Serial.println("** Preamble length                       ");
	Serial.println("** Symbol timeout                        ");
	Serial.println("*****************************************");

	g_mesh_events.data_avail_cb = on_mesh_data;
	g_mesh_events.map_changed_cb = map_changed_cb;

	// Enable RX mode
	g_lora_p2p_rx_mode = RX_MODE_RX; // RX_MODE_RX_DUTY;
	Radio.Rx(0);

	// Initialize the LoRa Mesh
	// * events
	init_mesh(&g_mesh_events);

	return true;
}

/**
   @brief Application specific event handler
		  Requires as minimum the handling of STATUS event
		  Here you handle as well your application specific events
*/
void app_event_handler(void)
{
	// Timer triggered event
	if ((g_task_event_type & STATUS) == STATUS)
	{
		g_task_event_type &= N_STATUS;
		MYLOG("APP", "Timer wakeup");

		print_mesh_map_oled();

		// Create a dummy data packet with 3x 64bit counter value
		msg_cnt++;

		convert_value.l_value = msg_cnt;
		memcpy(&data_buffer[0], convert_value.b_values, 8);
		memcpy(&data_buffer[8], convert_value.b_values, 8);
		memcpy(&data_buffer[16], convert_value.b_values, 8);

		// Select a random node from the map
		uint8_t selected_node_idx = 0;
		// Select broadcast as default
		bool use_broadcast = true;
		// Target node address;
		uint32_t node_addr = g_master_node_addr;

		if ((g_master_node_addr != 0) && (g_master_node_addr != 0xFFFFFFFF))
		{
			node_addr = g_master_node_addr;
			g_nodes_list_s route;
			// Check if we have a route to the master
			if (get_route(g_master_node_addr, &route))
			{
				MYLOG("APP", "Send to master node %08lX", g_master_node_addr);
				use_broadcast = false;
			}
			else
			{
				// No route, send as broadcast
				MYLOG("APP", "No route to master node %08lX", g_master_node_addr);
				use_broadcast = true;
			}
		}
		else
		{
			// Get the number of nodes in the map
			uint8_t node_index = nodes_in_map() + 1;
			MYLOG("APP", "%d nodes in the map", node_index);

			// Check how many nodes are in the map
			if (node_index > 2)
			{
				// Multiple nodes, select a random one
				selected_node_idx = (uint8_t)random(1, (long)node_index - 1);
				use_broadcast = false;
				MYLOG("APP", "Using node %d from the map", selected_node_idx);
			}
			else if (node_index == 2)
			{
				// Only 2 nodes in the map, send to the other one
				selected_node_idx = 1;
				use_broadcast = false;
				MYLOG("APP", "Using node 1 from the map");
			}
			else
			{
				// No other node, lets send a broadcast
				selected_node_idx = 0;
				use_broadcast = false;
				MYLOG("APP", "Empty map, using broadcast");
			}
			node_addr = get_node_addr(selected_node_idx);
			MYLOG("APP", "Got receiver address %08lX", node_addr);
		}

		// Enqueue the data package for sending
		uint8_t data_size = 24;
		if (!send_to_mesh(use_broadcast, node_addr, (uint8_t *)data_buffer, data_size))
		{
			MYLOG("APP", "Could not enqueue packet for sending");
		}
	}

	// Mesh Map changed
	if ((g_task_event_type & MESH_MAP_CHANGED) == MESH_MAP_CHANGED)
	{
		g_task_event_type &= N_MESH_MAP_CHANGED;
		Serial.println("Mesh Map has changed!");
		// Print the node map
		print_mesh_map();
	}
}

#ifdef NRF52_SERIES
/**
   @brief Handle BLE UART data

*/
void ble_data_handler(void)
{
	if (g_enable_ble)
	{
		// BLE UART data handling
		if ((g_task_event_type & BLE_DATA) == BLE_DATA)
		{
			MYLOG("AT", "RECEIVED BLE");
			/** BLE UART data arrived */
			g_task_event_type &= N_BLE_DATA;

			while (g_ble_uart.available() > 0)
			{
				at_serial_input(uint8_t(g_ble_uart.read()));
				delay(5);
			}
			at_serial_input(uint8_t('\n'));
		}
	}
}
#endif

/**
 * @brief Handle LoRa events
 *
 */
void lora_data_handler(void)
{
	// LoRa Join finished handling
	if ((g_task_event_type & LORA_JOIN_FIN) == LORA_JOIN_FIN)
	{
		g_task_event_type &= N_LORA_JOIN_FIN;

		// This PoC works only in LoRa P2P mode.
		// LoRaWAN join should never happen
		MYLOG("APP", "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		MYLOG("APP", "LoRaWAN Join callback. This code is only working with LoRa P2P!!!!!!");
		MYLOG("APP", "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}

	// LoRa TX finished handling
	if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
	{
		g_task_event_type &= N_LORA_TX_FIN;
		MYLOG("APP", "LoRa TX cycle %s", g_rx_fin_result ? "finished ACK" : "failed NAK");

		if ((g_lorawan_settings.confirmed_msg_enabled) && (g_lorawan_settings.lorawan_enable))
		{
			AT_PRINTF("+EVT:SEND CONFIRMED %s\n", g_rx_fin_result ? "SUCCESS" : "FAIL");
		}
		else
		{
			AT_PRINTF("+EVT:SEND OK\n");
		}

		// Call Mesh TX callback to check if further actions are required
		mesh_check_tx();

		print_mesh_map_oled();
	}

	// LoRa data handling
	if ((g_task_event_type & LORA_DATA) == LORA_DATA)
	{
		g_task_event_type &= N_LORA_DATA;
		MYLOG("APP", "Received package over LoRa");

		// Call Mesh RX callback to analyze the received package
		mesh_check_rx(g_rx_lora_data, g_rx_data_len, g_last_rssi, g_last_snr);

		print_mesh_map_oled();
	}
}

/**
 * @brief Callback after a LoRa Mesh data package was received
 *
 * @param fromID		Sender Node address
 * @param rxPayload		Pointer to the received data
 * @param rxSize		Length of the received package
 * @param rxRssi		Signal strength while the package was received
 * @param rxSnr			Signal to noise ratio while the package was received
 */
void on_mesh_data(uint32_t fromID, uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr)
{
	Serial.println("-------------------------------------");
	Serial.printf("Got data from node %08lX\n", fromID);

	longlong_byte_u new_counter;

	// Demo output of the received data packet
	memcpy(new_counter.b_values, &rxPayload[0], 8);
	Serial.printf("Got counter %Ld\n", new_counter.l_value);
	memcpy(new_counter.b_values, &rxPayload[8], 8);
	Serial.printf("Sensor Value 1 %Ld\n", new_counter.l_value);
	memcpy(new_counter.b_values, &rxPayload[16], 8);
	Serial.printf("Sensor Value 2 %Ld\n", new_counter.l_value);

	Serial.println("-------------------------------------");
}

/**
 * @brief Callback after the nodes list changed
 *
 */
void map_changed_cb(void)
{
	// No actions required, but can be used to react to a change in the Mesh Map
	// Do not use long code here, just set a flag and handle the event in the app_event_handler()
	api_wake_loop(MESH_MAP_CHANGED);
}