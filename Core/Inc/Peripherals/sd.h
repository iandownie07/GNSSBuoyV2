/*
 * SD.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_SD_H_
#define SRC_SD_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "app_filex.h"


// 20,000 milliseconds -> 20 seconds
#ifdef DBUG
#define WARMUP_TIME 20
#else
#define WARMUP_TIME 20000
#endif


typedef enum sd_error_code{
	SD_SUCCESS = 0,
	SD_PARSING_ERROR = -2,
	SD_SELF_TEST_FAIL = -3,
	SD_NOT_ENOUGH_SAMPLES = -4,
	SD_DONE_SAMPLING = -5
}sd_error_code_t;

/* Private define ------------------------------------------------------------*/
/* Main thread stack size */
#define FX_APP_THREAD_STACK_SIZE         2048
/* Main thread priority */
#define FX_APP_THREAD_PRIO               10
/* USER CODE BEGIN PD */
#define DEFAULT_QUEUE_LENGTH             16

#define MAX_FILENAME_LEN 32

/* Message content*/
typedef enum {
CARD_STATUS_CHANGED             = 99,
CARD_STATUS_DISCONNECTED        = 88,
CARD_STATUS_CONNECTED           = 77
} SD_ConnectionStateTypeDef;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MEDIA_CLOSED                     1UL
#define MEDIA_OPENED                     0UL
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */


/* USER CODE END PV */

/* Main thread Name */
#ifndef FX_APP_THREAD_NAME
  #define FX_APP_THREAD_NAME "FileX app thread"
#endif

/* Main thread time slice */
#ifndef FX_APP_THREAD_TIME_SLICE
  #define FX_APP_THREAD_TIME_SLICE TX_NO_TIME_SLICE
#endif

/* Main thread auto start */
#ifndef FX_APP_THREAD_AUTO_START
  #define FX_APP_THREAD_AUTO_START TX_AUTO_START
#endif

/* Main thread preemption threshold */
#ifndef FX_APP_PREEMPTION_THRESHOLD
  #define FX_APP_PREEMPTION_THRESHOLD FX_APP_THREAD_PRIO
#endif

/* fx sd volume name */
#ifndef FX_SD_VOLUME_NAME
  #define FX_SD_VOLUME_NAME "STM32_SDIO_DISK"
#endif
/* fx sd number of FATs */
#ifndef FX_SD_NUMBER_OF_FATS
  #define FX_SD_NUMBER_OF_FATS                1
#endif

/* fx sd Hidden sectors */
#ifndef FX_SD_HIDDEN_SECTORS
  #define FX_SD_HIDDEN_SECTORS               0
#endif


typedef struct SD {
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// Handle to the RTC
	RTC_HandleTypeDef* rtc_handle;
	// pointer to the message array
	sbd_message_type_52* current_message;
	// current lat/long (for future use)
	float current_lat;
	float current_lon;
	float (*get_timestamp)(struct SD* self);
	sd_error_code_t (*config)(struct SD* self);
	sd_error_code_t (*save_message)(struct SD* self);
	sd_error_code_t (*file_counter)(struct SD *self);
	bool timer_timeout;
	char filename[MAX_FILENAME_LEN];
} SD;

void sd_init(SD* self, microSWIFT_configuration* global_config,TX_EVENT_FLAGS_GROUP* control_flags, 
		 TX_EVENT_FLAGS_GROUP* error_flags, sbd_message_type_52* current_message, RTC_HandleTypeDef* rtc_handle);

sd_error_code_t sd_config(SD* self);
sd_error_code_t sd_save_message(SD* self);
float SD_get_timestamp(SD* self);
#endif /* SRC_SD_H_ */
