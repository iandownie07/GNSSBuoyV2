/*
 * BLE.h
 *
 *  Created on: 27th Jan 2026
 *      Author: Ian
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stm32u5xx_hal.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>


// 20,000 milliseconds -> 20 seconds
#ifdef DBUG
#define WARMUP_TIME 20
#else
#define WARMUP_TIME 20000
#endif


// BLE Return codes
typedef enum ble_error_code {
    // Success
    BLE_SUCCESS = 0,
    
    // General errors
    BLE_UNKNOWN_ERROR = -1,
    BLE_BUSY_ERROR = -2,
    BLE_TIMEOUT_ERROR = -3,
    
    // Communication errors
    BLE_UART_ERROR = -4,
    BLE_NO_RESPONSE = -5,
    BLE_INVALID_RESPONSE = -6,
    BLE_COMMAND_FAILED = -7,
    
    // Connection errors
    BLE_NOT_CONNECTED = -8,
    BLE_CONNECTION_FAILED = -9,
    BLE_DISCONNECTED = -10,
    BLE_CONNECTION_TIMEOUT = -11,
    
    // Configuration errors
    BLE_CONFIG_ERROR = -12,
    BLE_INVALID_PARAMETER = -13,
    BLE_NAME_SET_FAILED = -14,
    BLE_BAUD_SET_FAILED = -15,
    BLE_PIN_SET_FAILED = -16,
    
    // Data transmission errors
    BLE_TX_BUFFER_FULL = -17,
    BLE_TX_FAILED = -18,
    BLE_DATA_TOO_LARGE = -19,
    
    // Module state errors
    BLE_NOT_INITIALIZED = -20,
    BLE_ALREADY_INITIALIZED = -21,
    BLE_MODULE_NOT_RESPONDING = -22,
    BLE_SELF_TEST_FAILED = -23,
    
    // AT command specific
    BLE_AT_COMMAND_ERROR = -24,
    BLE_AT_TIMEOUT = -25,
    
    // Power/Reset errors
    BLE_RESET_FAILED = -26,
    BLE_POWER_ERROR = -27
    
} ble_error_code_t;

/* Private define ------------------------------------------------------------*/
#define BLE_DEFAULT_BAUD_RATE 9600
// UART buffer sizes
#define BLE_RX_BUFFER_SIZE 256
#define BLE_TX_BUFFER_SIZE 128
#define BLE_CMD_TIMEOUT    500 // ms

/* Example: WAKE on PB0 */
#define BLE_WAKE_GPIO_PORT GPIOB
#define BLE_WAKE_GPIO_PIN  GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */


/* USER CODE END PV */


typedef struct BLE {
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* ble_uart_handle;
	DMA_HandleTypeDef* ble_dma_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// Function pointers
	ble_error_code_t (*config)(struct BLE* self);
	ble_error_code_t (*reset_uart)(struct BLE* self, uint16_t baud_rate);
	ble_error_code_t (*send_data)(struct BLE* self, int32_t vel_n, int32_t vel_e, int32_t vel_d);
} BLE;

// Function declarations
void ble_init(BLE* self, microSWIFT_configuration* global_config,TX_EVENT_FLAGS_GROUP* control_flags, 
		 UART_HandleTypeDef* ble_uart_handle, DMA_HandleTypeDef* ble_dma_handle, TX_EVENT_FLAGS_GROUP* error_flags);
ble_error_code_t ble_send_velocity_data(BLE* self, int32_t vel_n, int32_t vel_e, int32_t vel_d);
ble_error_code_t ble_config(BLE* self);
ble_error_code_t ble_reset_uart(BLE* self, uint16_t baud_rate);
#endif /* SRC_BLE_H_ */
