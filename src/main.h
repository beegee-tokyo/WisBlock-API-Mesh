/**
 * @file main.h
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Defines, globals and includes
 * @version 0.1
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wisblock-API-V2.h>

// Debug output set to 0 to disable app debug output
#ifndef MY_DEBUG
#define MY_DEBUG 0
#endif

#if MY_DEBUG > 0
#define MYLOG(tag, ...)                     \
	do                                      \
	{                                       \
		if (tag)                            \
			PRINTF("[%s] ", tag);           \
		PRINTF(__VA_ARGS__);                \
		PRINTF("\n");                       \
		Serial.flush();                     \
		if (g_ble_uart_is_connected)        \
		{                                   \
			g_ble_uart.printf(__VA_ARGS__); \
			g_ble_uart.printf("\n");        \
		}                                   \
	} while (0)
#else
#define MYLOG(...)
#endif

// Generic data buffer
extern char data_buffer[];

/** Union to convert uint64_t into a byte array */
union longlong_byte_u
{
	uint64_t l_value = 0;
	uint8_t b_values[8];
};

// Mesh Network
#include "mesh.h"
void on_mesh_data(uint32_t fromID, uint8_t *rxPayload, uint16_t rxSize, int16_t rxRssi, int8_t rxSnr);
void map_changed_cb(void);

// Mesh events to be handled in the app_event_handler
#define MESH_MAP_CHANGED    0b1000000000000000
#define N_MESH_MAP_CHANGED  0b0111111111111111
extern uint32_t g_master_node_addr;

// User AT commands
 int at_set_master_node(char *str);
int at_query_master_node(void);
int at_query_node(void);
int at_query_mesh(void);
void read_master_node(void);
void save_master_node(void);
void init_user_at(void);

#endif /* MAIN_H */
