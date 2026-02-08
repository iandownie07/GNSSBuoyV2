#ifndef MESSAGING_H
#define MESSAGING_H

#include "byte_array.h"
#include "app_threadx.h"
#include "tx_api.h"
#include "tx_user.h"
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"
#include "tx_api.h"

// Message structure
typedef struct {
    int32_t vel_north;
    int32_t vel_east;
    int32_t vel_down;
} gnss_velocity_msg_t;

// Event flag definitions
#define GNSS_DATA_READY 0x01  // Bit 0: New GNSS data available

// Global queue and event flags declarations
extern TX_QUEUE gnss_to_ble_queue;
extern TX_EVENT_FLAGS_GROUP gnss_events;

// Queue initialization (called from main)
void messaging_init(void);

#endif