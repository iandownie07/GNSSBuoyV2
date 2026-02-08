/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/ble.h"

volatile uint32_t ble_tx_complete_count = 0;
volatile uint32_t ble_tx_error_count = 0;
static volatile bool ble_tx_in_progress = false;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        ble_tx_complete_count++;
        ble_tx_in_progress = false;  // Clear busy flag
        printf("[DMA] TX Complete! Count: %lu\n", ble_tx_complete_count);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        ble_tx_error_count++;
        ble_tx_in_progress = false;  // Clear busy flag
        printf("[DMA] TX ERROR! Count: %lu, Error code: 0x%lx\n", 
               ble_tx_error_count, huart->ErrorCode);
    }
}

/**
 * Initialize the CT struct
 *
 * @return void
 */
void ble_init(BLE* self, microSWIFT_configuration* global_config, TX_EVENT_FLAGS_GROUP* control_flags, 
		 UART_HandleTypeDef* ble_uart_handle, DMA_HandleTypeDef* ble_dma_handle, TX_EVENT_FLAGS_GROUP* error_flags)
{
	self->global_config = global_config;
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->ble_uart_handle = ble_uart_handle;
	self->ble_dma_handle = ble_dma_handle;
	self->send_data = ble_send_velocity_data;
	self->config = ble_config;
	self->reset_uart = ble_reset_uart;
}

/**
 * Configure BLE
 */
ble_error_code_t ble_config(BLE* self)
{
    printf("[BLE] Configuring UART...\n");
    ble_error_code_t status = self->reset_uart(self, BLE_DEFAULT_BAUD_RATE);
    
    if (status == BLE_SUCCESS) {
        printf("[BLE] UART configured successfully\n");
    } else {
        printf("[BLE] UART configuration failed\n");
    }
    
    return status;
}

ble_error_code_t ble_send_velocity_data(BLE* self, int32_t vel_n, int32_t vel_e, int32_t vel_d) {
    HAL_StatusTypeDef hal_status;
    static char msg[64]; // DMA terminates immediately after execution so msg cannot be on stack

    int len = snprintf(msg, sizeof(msg), "%ld, %ld, %ld\r\n", vel_n, vel_e, vel_d);
    
    HAL_UART_Transmit_DMA(self->ble_uart_handle, (uint8_t*)msg, len);

    if (hal_status != HAL_OK) {
        if (hal_status == HAL_BUSY) {
            return BLE_BUSY_ERROR;
        }
        return BLE_TX_FAILED;
    }

    return BLE_SUCCESS;
}


/**
 * Reinitialize the BLE UART port. Required when switching between Tx and Rx.
 *
 * @param self - BLE struct
 * @param baud_rate - baud rate to set port to
 */
ble_error_code_t ble_reset_uart(BLE* self, uint16_t baud_rate)
{

	if (HAL_UART_DeInit(self->ble_uart_handle) != HAL_OK) {
		return BLE_UART_ERROR;
	}

	self->ble_uart_handle->Instance = self->ble_uart_handle->Instance;
	self->ble_uart_handle->Init.BaudRate = baud_rate;
	self->ble_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	self->ble_uart_handle->Init.StopBits = UART_STOPBITS_1;
	self->ble_uart_handle->Init.Parity = UART_PARITY_NONE;
	self->ble_uart_handle->Init.Mode = UART_MODE_TX_RX;
	self->ble_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->ble_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->ble_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	self->ble_uart_handle->FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(self->ble_uart_handle) != HAL_OK)
	{
		return BLE_UART_ERROR;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(self->ble_uart_handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return BLE_UART_ERROR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(self->ble_uart_handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return BLE_UART_ERROR;
	}
	if (HAL_UARTEx_DisableFifoMode(self->ble_uart_handle) != HAL_OK)
	{
		return BLE_UART_ERROR;
	}

	//LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_1);

	return BLE_SUCCESS;
}