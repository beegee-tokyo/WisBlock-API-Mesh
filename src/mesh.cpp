/**
 * @file mesh.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Mesh network functions
 * @version 0.1
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */
/*********************************************/
/* Could move to WisBlock-API-V2             */
/*********************************************/

#include "main.h"

/** Structure with Mesh event callbacks */
mesh_events_s g_mesh_events;

#define BROKEN_NET

/** Task to handle mesh */
TaskHandle_t mesh_task_handle = NULL;

/** Queue to handle cloud send requests */
volatile xQueueHandle mesh_msg_queue;

/** Semaphore for Mesh task wakeup */
SemaphoreHandle_t mesh_task_wakeup_signal;

/** Timer for network map sync */
SoftwareTimer map_sync_timer;

/** Event flag for Mesh Task */
volatile uint16_t mesh_event;

/** The Mesh node ID, created from ID of the nRF52 */
uint32_t g_this_device_addr;
/** The Mesh broadcast ID, created from node ID */
uint32_t g_broadcast_id;

/** Map message buffer */
map_msg_s map_sync_msg;

/** Structure for outgoing data */
data_msg_s out_data_buffer;

/** Max number of messages in the queue */
#define SEND_QUEUE_SIZE 4
/** Send buffer for SEND_QUEUE_SIZE messages */
data_msg_s send_msg_buffer[SEND_QUEUE_SIZE];
/** Message size buffer for SEND_QUEUE_SIZE messages */
uint8_t send_msg_size[SEND_QUEUE_SIZE];
/** Queue to handle send requests */
volatile xQueueHandle send_queue;
#ifdef ESP32
/** Mux used to enter critical code part (clear up queue content) */
portMUX_TYPE access_msg_queue_mux = portMUX_INITIALIZER_UNLOCKED;
#endif
/** Mux used to enter critical code part (access to node list) */
SemaphoreHandle_t access_node_list_sem;

/** LoRa TX package */
uint8_t tx_pckg[256];
/** Size of data package */
uint16_t tx_buff_len = 0;
/** LoRa RX buffer */
uint8_t rx_buffer[256];

/** Sync time for routing at start, every 30 seconds */
#define INIT_SYNCTIME 30000
/** Sync time for routing after mesh has settled, every 10 minutes*/
#define DEFAULT_SYNCTIME 600000
/** Sync time */
time_t sync_time = INIT_SYNCTIME;
/** Counter to switch from initial sync time to default sync time*/
uint8_t switch_sync_time_cnt = 10;

/** Enum for network state */
typedef enum
{
	MESH_IDLE = 0, //!< The radio is idle
	MESH_RX,	   //!< The radio is in reception state
	MESH_TX,	   //!< The radio is in transmission state
	MESH_NOTIF	   //!< The radio is doing mesh notification
} mesh_radio_state_t;

/** Lora statemachine status */
mesh_radio_state_t lora_state = MESH_IDLE;

/** Mesh callback variable */
mesh_events_s *_mesh_events;

/** Number of nodes in the map */
int g_num_of_nodes = 0;

/** Flag if the nodes map has changed */
boolean nodes_changed = false;

/**
 * @brief Callback for Mesh Map Update Timer
 *
 * @param unused
 */
void map_sync_handler(TimerHandle_t unused)
{
	mesh_event |= SYNC_MAP;
	xSemaphoreGive(mesh_task_wakeup_signal);
}

/**
 * @brief Initialize the Mesh network
 *
 * @param events Structure of event callbacks
 */
void init_mesh(mesh_events_s *events)
{
	_mesh_events = events;

#ifdef ESP32
	g_num_of_nodes = 48;
#else
	g_num_of_nodes = 30;
#endif

	// Prepare empty nodes map
	g_nodes_map = (g_nodes_list_s *)malloc(g_num_of_nodes * sizeof(g_nodes_list_s));

	if (g_nodes_map == NULL)
	{
		MYLOG("MESH", "Could not allocate memory for nodes map");
	}
	else
	{
		MYLOG("MESH", "Memory for nodes map is allocated");
	}
	memset(g_nodes_map, 0, g_num_of_nodes * sizeof(g_nodes_list_s));

	// Create queue
	send_queue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(uint8_t));
	if (send_queue == NULL)
	{
		MYLOG("MESH", "Could not create send queue!");
	}
	else
	{
		MYLOG("MESH", "Send queue created!");
	}
	// Create blocking semaphore for nodes list access
	access_node_list_sem = xSemaphoreCreateBinary();
	xSemaphoreGive(access_node_list_sem);

	// Create blocking semaphore for Mesh task
	mesh_task_wakeup_signal = xSemaphoreCreateBinary();
	xSemaphoreGive(mesh_task_wakeup_signal);
	xSemaphoreTake(mesh_task_wakeup_signal, 1);

	// Create node ID
	uint8_t deviceMac[8];

	BoardGetUniqueId(deviceMac);

	g_this_device_addr += (uint32_t)deviceMac[2] << 24;
	g_this_device_addr += (uint32_t)deviceMac[3] << 16;
	g_this_device_addr += (uint32_t)deviceMac[4] << 8;
	g_this_device_addr += (uint32_t)deviceMac[5];

	MYLOG("MESH", "Mesh NodeId = %08lX", g_this_device_addr);

	// Create broadcast ID
	g_broadcast_id = g_this_device_addr & 0xFFFFFF00;
	MYLOG("MESH", "Broadcast ID is %08lX", g_broadcast_id);

	// Create message queue for LoRa
	mesh_msg_queue = xQueueCreate(10, sizeof(uint8_t));
	if (mesh_msg_queue == NULL)
	{
		MYLOG("MESH", "Could not create LoRa message queue!");
	}
	else
	{
		MYLOG("MESH", "LoRa message queue created!");
	}

	if (!xTaskCreate(mesh_task, "MeshSync", 3096, NULL, 1, &mesh_task_handle))
	{
		MYLOG("MESH", "Starting Mesh Sync Task failed");
	}
	else
	{
		MYLOG("MESH", "Starting Mesh Sync Task success");
	}

	map_sync_timer.begin(sync_time, map_sync_handler, NULL, false);
	map_sync_timer.start();

	// Wake up task to handle request for nodes map
	mesh_event |= SYNC_MAP;
	xSemaphoreGive(mesh_task_wakeup_signal);
}

/**
 * @brief Task to handle the mesh
 *
 * @param pvParameters Unused task parameters
 */
void mesh_task(void *pvParameters)
{
	// Queue variable to be sent to the task
	uint8_t queue_index;

	lora_state = MESH_IDLE;

	while (1)
	{
		if (xSemaphoreTake(mesh_task_wakeup_signal, portMAX_DELAY) == pdTRUE)
		{
			// MYLOG("MESH", "Mesh task wakeup");
			if ((mesh_event & NODE_CHANGE) == NODE_CHANGE)
			{
				mesh_event &= N_NODE_CHANGE;

				// MYLOG("MESH", "Mesh task Node change");
				nodes_changed = false;
				if ((_mesh_events != NULL) && (_mesh_events->map_changed_cb != NULL))
				{
					_mesh_events->map_changed_cb();
				}
			}

			if ((mesh_event & SYNC_MAP) == SYNC_MAP)
			{
				// MYLOG("MESH", "Mesh task Sync Map");
				mesh_event &= N_SYNC_MAP;

				// Time to sync the Mesh
				if (xSemaphoreTake(access_node_list_sem, (TickType_t)1000) == pdTRUE)
				{
					// MYLOG("MESH", "Checking mesh map");
					if (!clean_map())
					{
						sync_time = INIT_SYNCTIME;
						if ((_mesh_events != NULL) && (_mesh_events->map_changed_cb != NULL))
						{
							_mesh_events->map_changed_cb();
						}
					}
					// MYLOG("MESH", "Sending mesh map");
					map_sync_msg.from = g_this_device_addr;
					map_sync_msg.type = LORA_NODEMAP;
					memset(map_sync_msg.nodes, 0, 48 * 5);

					// Get sub nodes
					uint8_t subs_len = node_map(map_sync_msg.nodes);

					xSemaphoreGive(access_node_list_sem);

					if (subs_len != 0)
					{
						for (int idx = 0; idx < subs_len; idx++)
						{
							uint32_t checkNode = map_sync_msg.nodes[idx][0];
							checkNode |= map_sync_msg.nodes[idx][1] << 8;
							checkNode |= map_sync_msg.nodes[idx][2] << 16;
							checkNode |= map_sync_msg.nodes[idx][3] << 24;
						}
					}
					map_sync_msg.nodes[subs_len][0] = 0xAA;
					map_sync_msg.nodes[subs_len][1] = 0x55;
					map_sync_msg.nodes[subs_len][2] = 0x00;
					map_sync_msg.nodes[subs_len][3] = 0xFF;
					map_sync_msg.nodes[subs_len][4] = 0xAA;
					subs_len++;

					subs_len = MAP_HEADER_SIZE + (subs_len * 5);

					if (!add_send_request((data_msg_s *)&map_sync_msg, subs_len))
					{
						MYLOG("MESH", "Cannot send map because send queue is full");
					}
				}
				else
				{
					MYLOG("MESH", "Cannot access map for clean up and syncing");
				}

				// Time to relax the syncing ???
				switch_sync_time_cnt--;

				if (switch_sync_time_cnt == 0)
				{
					MYLOG("MESH", "Switching sync time to DEFAULT_SYNCTIME");
					sync_time = DEFAULT_SYNCTIME;
				}

				// Restart map sync timer
				map_sync_timer.setPeriod(sync_time);
				map_sync_timer.start();
			}

			if ((mesh_event & CHECK_QUEUE) == CHECK_QUEUE)
			{
				mesh_event &= N_CHECK_QUEUE;

				// MYLOG("MESH", "Mesh task check send queue");

				// Check if we have something in the queue
				if (xQueuePeek(send_queue, &queue_index, (TickType_t)10) == pdTRUE)
				{
					if (lora_state != MESH_TX)
					{
#ifdef ESP32
						portENTER_CRITICAL(&access_msg_queue_mux);
#else
						taskENTER_CRITICAL();
#endif
						tx_buff_len = send_msg_size[queue_index];
						memset(tx_pckg, 0, 256);
						memcpy(tx_pckg, &send_msg_buffer[queue_index].mark1, tx_buff_len);
						if (xQueueReceive(send_queue, &queue_index, portMAX_DELAY) == pdTRUE)
						{
							send_msg_buffer[queue_index].type = 0;
#ifdef ESP32
							portEXIT_CRITICAL(&access_msg_queue_mux);
#else
							taskEXIT_CRITICAL();
#endif

							// MYLOG("MESH", "Sending msg #%d with len %d", queue_index, tx_buff_len);

							lora_state = MESH_TX;

							// Send packet over LoRa
							if (send_p2p_packet((uint8_t *)&tx_pckg, tx_buff_len))
							{
								MYLOG("MESH", "Packet enqueued");
							}
							else
							{
								AT_PRINTF("+EVT:SIZE_ERROR\n");
								MYLOG("MESH", "Packet too big");
							}
						}
						else
						{
#ifdef ESP32
							portEXIT_CRITICAL(&access_msg_queue_mux);
#else
							taskEXIT_CRITICAL();
#endif
						}
					}
				}
			}
		}
	}
}

// To simulate a mesh where some nodes are out of range,
// replace the 3 node addresses below with real node addresses
// Requires at least 3 nodes in the net
// NODE 1 can only connect to NODE 3
// NODE 2 can only connect to NODE 3

#define BROKEN_NODE_1 0x6FA6BC6C
#define BROKEN_NODE_2 0xCBE0E4F5
#define BROKEN_NODE_3 0x2BD56908

/**
 * Callback after a LoRa package was received
 * @param rxPayload
 * 			Pointer to the received data
 * @param rxSize
 * 			Length of the received package
 * @param rxRssi
 * 			Signal strength while the package was received
 * @param rxSnr
 * 			Signal to noise ratio while the package was received
 */
void mesh_check_rx(uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr)
{
	// Secure buffer before restart listening
	if (rxSize < 256)
	{
		memcpy(rx_buffer, rxPayload, rxSize + 1);
		// Make sure the data is null terminated
		rx_buffer[rxSize] = 0;
	}
	else
	{
		memcpy(rx_buffer, rxPayload, rxSize);
	}

	uint16_t tempSize = rxSize;

	delay(1);

	lora_state = MESH_IDLE;

	// Check the received data
	if ((rx_buffer[0] == 'L') && (rx_buffer[1] == 'o') && (rx_buffer[2] == 'R'))
	{
		// Valid Mesh data received
		map_msg_s *thisMsg = (map_msg_s *)rx_buffer;
		data_msg_s *thisDataMsg = (data_msg_s *)rx_buffer;

		if (thisMsg->type == LORA_NODEMAP)
		{
			/// \todo for debug make some nodes unreachable
#ifdef BROKEN_NET
			switch (g_this_device_addr)
			{
			case BROKEN_NODE_1:
				if (thisMsg->from == BROKEN_NODE_2)
				{
					return;
				}
			case BROKEN_NODE_2:
				if (thisMsg->from == BROKEN_NODE_1)
				{
					return;
				}
			}
#endif
			MYLOG("MESH", "Got map message");
			// Mapping received
			uint8_t subsSize = tempSize - MAP_HEADER_SIZE;
			uint8_t numSubs = subsSize / 5;

			// Serial.println("********************************");
			// for (int idx = 0; idx < tempSize; idx++)
			// {
			// 	Serial.printf("%02X ", rx_buffer[idx]);
			// }
			// Serial.println("");
			// Serial.printf("subsSize %d -> # subs %d\n", subsSize, subsSize / 5);
			// Serial.println("********************************");

			// Check if end marker is in the message
			if ((thisMsg->nodes[numSubs - 1][0] != 0xAA) ||
				(thisMsg->nodes[numSubs - 1][1] != 0x55) ||
				(thisMsg->nodes[numSubs - 1][2] != 0x00) ||
				(thisMsg->nodes[numSubs - 1][3] != 0xFF) ||
				(thisMsg->nodes[numSubs - 1][4] != 0xAA))
			{
				MYLOG("MESH", "Invalid map, end marker is missing from %08lX", thisMsg->from);
				xSemaphoreGive(access_node_list_sem);
				return;
			}
			if (xSemaphoreTake(access_node_list_sem, (TickType_t)1000) == pdTRUE)
			{
				nodes_changed = add_node(thisMsg->from, 0, 0);

				// Remove nodes that use sending node as hop
				clear_subs(thisMsg->from);

				// MYLOG("MESH", "From %08lX", thisMsg->from);
				// MYLOG("MESH", "Dest %08lX", thisMsg->dest);

				if (subsSize != 0)
				{
					// Mapping contains subs

					// MYLOG("MESH", "Msg size %d", tempSize);
					// MYLOG("MESH", "#subs %d", numSubs);

					// Serial.println("++++++++++++++++++++++++++++");
					// Serial.printf("From %08X Dest %08X #Subs %d\n", thisMsg->from, thisMsg->dest, numSubs);
					// for (int idx = 0; idx < numSubs; idx++)
					// {
					// 	uint32_t subId = (uint32_t)thisMsg->nodes[idx][0];
					// 	subId += (uint32_t)thisMsg->nodes[idx][1] << 8;
					// 	subId += (uint32_t)thisMsg->nodes[idx][2] << 16;
					// 	subId += (uint32_t)thisMsg->nodes[idx][3] << 24;
					// 	uint8_t hops = thisMsg->nodes[idx][4];
					// 	Serial.printf("ID: %08X numHops: %d\n", subId, hops);
					// }
					// Serial.println("++++++++++++++++++++++++++++");

					for (int idx = 0; idx < numSubs - 1; idx++)
					{
						uint32_t subId = (uint32_t)thisMsg->nodes[idx][0];
						subId += (uint32_t)thisMsg->nodes[idx][1] << 8;
						subId += (uint32_t)thisMsg->nodes[idx][2] << 16;
						subId += (uint32_t)thisMsg->nodes[idx][3] << 24;
						uint8_t hops = thisMsg->nodes[idx][4];
						if (subId != g_this_device_addr)
						{
							nodes_changed |= add_node(subId, thisMsg->from, hops + 1);
							// MYLOG("MESH", "Subs %08lX", subId);
						}
					}
				}
				xSemaphoreGive(access_node_list_sem);

				if (nodes_changed)
				{
					// Wake up task to handle change of nodes map
					mesh_event |= NODE_CHANGE;
					xSemaphoreGive(mesh_task_wakeup_signal);
				}
			}
			else
			{
				MYLOG("MESH", "Could not access map to add node");
			}
		}
		else if (thisDataMsg->type == LORA_DIRECT)
		{
			// MYLOG("MESH", "From %08lX", thisDataMsg->from);
			// MYLOG("MESH", "Dest %08lX", thisDataMsg->dest);

			if (thisDataMsg->dest == g_this_device_addr)
			{
				MYLOG("MESH", "LoRa Packet received size:%d, rssi:%d, snr:%d", rxSize, rxRssi, rxSnr);
#if MY_DEBUG > 0
				for (int idx = 0; idx < rxSize; idx++)
				{
					Serial.printf(" %02X", rx_buffer[idx]);
				}
				Serial.println("");
#endif
				// Message is for us, call user callback to handle the data
				// MYLOG("MESH", "Got data message type %c >%s<", thisDataMsg->data[0], (char *)&thisDataMsg->data[1]);
				if ((_mesh_events != NULL) && (_mesh_events->data_avail_cb != NULL))
				{
					if (thisDataMsg->orig == 0x00)
					{
						_mesh_events->data_avail_cb(thisDataMsg->from, thisDataMsg->data, tempSize - DATA_HEADER_SIZE, rxRssi, rxSnr);
					}
					else
					{
						_mesh_events->data_avail_cb(thisDataMsg->orig, thisDataMsg->data, tempSize - DATA_HEADER_SIZE, rxRssi, rxSnr);
					}
				}
			}
			else
			{
				// Message is not for us
			}
			// Check if we know that node
			if (!check_node(thisDataMsg->from))
			{
				// Unknown node, force a map update
				MYLOG("MESH", "Unknown node, force map update");
				send_map_request();
			}
		}
		else if (thisDataMsg->type == LORA_FORWARD)
		{
			if (thisDataMsg->dest == g_this_device_addr)
			{
				MYLOG("MESH", "LoRa Packet received size:%d, rssi:%d, snr:%d", rxSize, rxRssi, rxSnr);
#if MY_DEBUG > 0
				for (int idx = 0; idx < rxSize; idx++)
				{
					Serial.printf(" %02X", rx_buffer[idx]);
				}
				Serial.println("");
#endif
				// Message is for us, call user callback to handle the data
				// MYLOG("MESH", "Got data message type %c >%s<", thisDataMsg->data[0], (char *)&thisDataMsg->data[1]);
				if ((_mesh_events != NULL) && (_mesh_events->data_avail_cb != NULL))
				{
					if (thisDataMsg->orig == 0x00)
					{
						_mesh_events->data_avail_cb(thisDataMsg->from, thisDataMsg->data, tempSize - DATA_HEADER_SIZE, rxRssi, rxSnr);
					}
					else
					{
						_mesh_events->data_avail_cb(thisDataMsg->orig, thisDataMsg->data, tempSize - DATA_HEADER_SIZE, rxRssi, rxSnr);
					}
				}
			}
			else
			{
				// Message is for sub node, forward the message
				g_nodes_list_s route;
				if (xSemaphoreTake(access_node_list_sem, (TickType_t)1000) == pdTRUE)
				{
					if (get_route(thisDataMsg->from, &route))
					{
						// We found a route, send package to next hop
						if (route.first_hop == 0)
						{
							// Node is in range, use direct message
							// MYLOG("MESH", "Route for %lX is direct", route.node_id);
							// Destination is a direct
							thisDataMsg->dest = thisDataMsg->from;
							thisDataMsg->from = thisDataMsg->orig;
							thisDataMsg->type = LORA_DIRECT;
						}
						else
						{
							// Node is not in range, use forwarding
							// MYLOG("MESH", "Route for %lX is to %lX", route.node_id, route.first_hop);
							// Destination is a sub
							thisDataMsg->dest = route.first_hop;
							thisDataMsg->type = LORA_FORWARD;
						}

						// Put message into send queue
						if (!add_send_request(thisDataMsg, tempSize))
						{
							MYLOG("MESH", "Cannot forward message because send queue is full");
						}
					}
					else
					{
						MYLOG("MESH", "No route found for %lX", thisMsg->from);
					}
					xSemaphoreGive(access_node_list_sem);
				}
				else
				{
					MYLOG("MESH", "Could not access map to forward package");
				}
			}
			// Check if we know that node
			if (!check_node(thisDataMsg->from))
			{
				// Unknown node, force a map update
				MYLOG("MESH", "Unknown node, force map update");
				send_map_request();
			}
		}
		else if (thisDataMsg->type == LORA_BROADCAST)
		{
			// This is a broadcast. Forward to all direct nodes, but not to the one who sent it
			// MYLOG("MESH", "Handling broadcast with ID %08lX from %08lX", thisDataMsg->dest, thisDataMsg->from);
			// Check if this broadcast is coming from ourself
			if ((thisDataMsg->dest & 0xFFFFFF00) == (g_this_device_addr & 0xFFFFFF00))
			{
				// MYLOG("MESH", "We received our own broadcast, dismissing it");
				return;
			}
			// Check if we handled this broadcast already
			if (is_old_broadcast(thisDataMsg->dest))
			{
				// MYLOG("MESH", "Got an old broadcast, dismissing it");
				return;
			}

			// Put broadcast into send queue
			if (!add_send_request(thisDataMsg, tempSize))
			{
				MYLOG("MESH", "Cannot forward broadcast because send queue is full");
			}

			// This is a broadcast, call user callback to handle the data
			// MYLOG("MESH", "Got data broadcast size %ld", tempSize);
			if ((_mesh_events != NULL) && (_mesh_events->data_avail_cb != NULL))
			{
				_mesh_events->data_avail_cb(thisDataMsg->from, thisDataMsg->data, tempSize - DATA_HEADER_SIZE, rxRssi, rxSnr);
			}
			// Check if we know that node
			if (!check_node(thisDataMsg->from))
			{
				// Unknown node, force a map update
				MYLOG("MESH", "Unknown node, force map update");
				send_map_request();
			}
		}
		else if (thisDataMsg->type == LORA_MAP_REQ)
		{
			// This is a broadcast. Forward to all direct nodes, but not to the one who sent it
			MYLOG("MESH", "Handling Mesh map request with ID %08lX from %08lX", thisDataMsg->dest, thisDataMsg->from);
			// Check if this broadcast is coming from ourself
			if ((thisDataMsg->dest & 0xFFFFFF00) == (g_this_device_addr & 0xFFFFFF00))
			{
				MYLOG("MESH", "We received our own broadcast, dismissing it");
				return;
			}
			// Check if we handled this broadcast already
			if (is_old_broadcast(thisDataMsg->dest))
			{
				MYLOG("MESH", "Got an old broadcast, dismissing it");
				return;
			}

			// Put broadcast into send queue
			if (!add_send_request(thisDataMsg, tempSize))
			{
				MYLOG("MESH", "Cannot forward broadcast because send queue is full");
			}
			// Wake up task to handle request for nodes map
			mesh_event |= SYNC_MAP;
			xSemaphoreGive(mesh_task_wakeup_signal);
		}
	}
	else
	{
		// MYLOG("MESH", "Invalid package");
		for (int idx = 0; idx < tempSize; idx++)
		{
			Serial.printf("%02X ", rx_buffer[idx]);
		}
		Serial.println("");
	}
}

/**
 * @brief Callback after a package was sent
 *
 */
void mesh_check_tx(void)
{
	// MYLOG("MESH", "LoRa send finished");
	lora_state = MESH_IDLE;

	mesh_event |= CHECK_QUEUE;
	xSemaphoreGive(mesh_task_wakeup_signal);
}

/**
 * Add a data package to the queue
 * @param package
 * 			dataPckg * to the package data
 * @param msg_size
 * 			Size of the data package
 * @return result
 * 			TRUE if task could be added to queue
 * 			FALSE if queue is full or not initialized
 */
bool add_send_request(data_msg_s *package, uint8_t msg_size)
{
	if (send_queue != NULL)
	{
#ifdef ESP32
		portENTER_CRITICAL(&access_msg_queue_mux);
#else
		taskENTER_CRITICAL();
#endif
		// Find unused entry in queue list
		int next = SEND_QUEUE_SIZE;
		for (int idx = 0; idx < SEND_QUEUE_SIZE; idx++)
		{
			if (send_msg_buffer[idx].type == LORA_INVALID)
			{
				next = idx;
				break;
			}
		}

		if (next != SEND_QUEUE_SIZE)
		{
			// Found an empty entry!
			memcpy(&send_msg_buffer[next], package, msg_size);
			send_msg_size[next] = msg_size;

			// Try to add to cloudTaskQueue
			if (xQueueSend(send_queue, &next, (TickType_t)1000) != pdTRUE)
			{
				MYLOG("MESH", "Send queue is busy");
#ifdef ESP32
				portEXIT_CRITICAL(&access_msg_queue_mux);
#else
				taskEXIT_CRITICAL();
#endif
				return false;
			}
			else
			{
				// MYLOG("MESH", "Queued msg #%d with len %d", next, msg_size);
#ifdef ESP32
				portEXIT_CRITICAL(&access_msg_queue_mux);
#else
				taskEXIT_CRITICAL();
#endif
				mesh_event |= CHECK_QUEUE;
				xSemaphoreGive(mesh_task_wakeup_signal);
				return true;
			}
		}
		else
		{
			MYLOG("MESH", "Send queue is full");
			// Queue is already full!
#ifdef ESP32
			portEXIT_CRITICAL(&access_msg_queue_mux);
#else
			taskEXIT_CRITICAL();
#endif
			return false;
		}
	}
	else
	{
		MYLOG("MESH", "Send queue not yet initialized");
		return false;
	}
}

/**
 * @brief Print the current Mesh Map to Serial and BLE
 *
 */
void print_mesh_map(void)
{
	/** Node ID of the selected receiver node */
	uint32_t node_id[48];
	/** First hop ID of the selected receiver node */
	uint32_t first_hop[48];
	/** Number of hops to the selected receiver node */
	uint8_t num_hops[48];
	/** Number of nodes in the map */
	uint8_t num_elements;

	AT_PRINTF("---------------------------------------------");
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
		AT_PRINTF("%d nodes in the map", num_elements + 1);
		AT_PRINTF("Node #01 id: %08lX this node", g_this_device_addr);
		for (int idx = 0; idx < num_elements; idx++)
		{
			if (first_hop[idx] == 0)
			{
				AT_PRINTF("Node #%02d id: %08lX direct", idx + 2, node_id[idx]);
			}
			else
			{
				AT_PRINTF("Node #%02d id: %08lX first hop %08lX #hops %d", idx + 2, node_id[idx], first_hop[idx], num_hops[idx]);
			}
		}
	}
	else
	{
		AT_PRINTF("Could not access the nodes list");
	}
	AT_PRINTF("---------------------------------------------");
}

/**
 * @brief Enqueue data to be send over the Mesh Network
 *
 * @param is_broadcast if true, send as broadcast
 * @param target_addr used for direct message
 * @param tx_data pointer to data packet
 * @param data_size size of data packet
 * @return true packet is enqueued for sending
 * @return false
 */
bool send_to_mesh(bool is_broadcast, uint32_t target_addr, uint8_t *tx_data, uint8_t data_size)
{
	char data_buffer[512];
	// Check if data fits into buffer
	if (data_size > 243)
	{
		return false;
	}

	// Check if it is a broadcast
	if (is_broadcast)
	{
		// Send a broadcast
		// Prepare data
		out_data_buffer.mark1 = 'L';
		out_data_buffer.mark2 = 'o';
		out_data_buffer.mark3 = 'R';

		// Setup broadcast
		out_data_buffer.dest = get_next_broadcast_id();
		out_data_buffer.from = g_this_device_addr;
		out_data_buffer.type = LORA_BROADCAST;
		// MYLOG("MESH", "Queuing broadcast with id %08lX", out_data_buffer.dest);
		// if (g_ble_uart_is_connected)
		// {
		// 	snprintf(data_buffer, 512, "Queuing broadcast with id %08lX\n", out_data_buffer.dest);
		// 	g_ble_uart.println(data_buffer);
		// }

		memcpy(out_data_buffer.data, tx_data, data_size);

		int dataLen = DATA_HEADER_SIZE + data_size;

		// Add package to send queue
		if (!add_send_request(&out_data_buffer, dataLen))
		{
			MYLOG("MESH", "Sending package failed");
			if (g_ble_uart_is_connected)
			{
				snprintf(data_buffer, 512, "Sending package failed\n");
				g_ble_uart.println(data_buffer);
			}
		}
	}
	else // direct message
	{
		// Prepare data
		out_data_buffer.mark1 = 'L';
		out_data_buffer.mark2 = 'o';
		out_data_buffer.mark3 = 'R';

		// Prepare direct message
		out_data_buffer.dest = target_addr;
		out_data_buffer.from = g_this_device_addr;
		out_data_buffer.type = LORA_DIRECT;

		memcpy(out_data_buffer.data, tx_data, data_size);

		int dataLen = DATA_HEADER_SIZE + data_size;

		// Add package to send queue
		if (!add_send_request(&out_data_buffer, dataLen))
		{
			MYLOG("MESH", "Sending package failed");
			if (g_ble_uart_is_connected)
			{
				snprintf(data_buffer, 512, "Sending package failed\n");
				g_ble_uart.println(data_buffer);
			}
			return false;
		}
		// else
		// {
		// 	MYLOG("MESH", "Queued msg direct to %08lX", out_data_buffer.dest);
		// 	if (g_ble_uart_is_connected)
		// 	{
		// 		snprintf(data_buffer, 512, "Queued msg direct to %08lX\n", out_data_buffer.dest);
		// 		g_ble_uart.println(data_buffer);
		// 	}
		// }
	}
	return true;
}

/**
 * @brief Send a map sync request as a broadcast message
 * 
 * @return true if enqueued
 * @return false if queue is full
 */
bool send_map_request()
{
	// Send a broadcast
	// Prepare data
	out_data_buffer.mark1 = 'L';
	out_data_buffer.mark2 = 'o';
	out_data_buffer.mark3 = 'R';

	// Setup broadcast
	out_data_buffer.dest = get_next_broadcast_id();
	out_data_buffer.from = g_this_device_addr;
	out_data_buffer.type = LORA_MAP_REQ;
	// MYLOG("MESH", "Queuing map request with id %08lX", out_data_buffer.dest);
	// if (g_ble_uart_is_connected)
	// {
	// 	snprintf(data_buffer, 512, "Queuing broadcast with id %08lX\n", out_data_buffer.dest);
	// 	g_ble_uart.println(data_buffer);
	// }

	int dataLen = DATA_HEADER_SIZE;

	// Add package to send queue
	if (!add_send_request(&out_data_buffer, dataLen))
	{
		MYLOG("MESH", "Sending package failed");
		if (g_ble_uart_is_connected)
		{
			// snprintf(data_buffer, 512, "Sending package failed\n");
			g_ble_uart.println("Sending package failed\n");
		}
		return false;
	}
	return true;
}