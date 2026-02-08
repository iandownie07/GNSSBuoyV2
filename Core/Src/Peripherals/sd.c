/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/sd.h"

#define CSV_BUFFER_SIZE 4096



/**
 * Initialize the CT struct
 *
 * @return void
 */
void sd_init(SD* self, microSWIFT_configuration* global_config, TX_EVENT_FLAGS_GROUP* control_flags, 
		 TX_EVENT_FLAGS_GROUP* error_flags, sbd_message_type_52* current_message, RTC_HandleTypeDef* rtc_handle)
{
	self->global_config = global_config;
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->current_message = current_message;
	self->rtc_handle = rtc_handle;
	self->current_lat = 0.0;
	self->current_lon = 0.0;
	self->config = sd_config;
  self->get_timestamp = SD_get_timestamp;
  self->save_message = sd_save_message;
}

static uint8_t SD_IsDetected(uint32_t Instance);
static VOID media_close_callback (FX_MEDIA *media_ptr);
static UINT get_highest_file_index(UINT *highest_index);

	/* Buffer for FileX FX_MEDIA sector cache. */
ALIGN_32BYTES (uint32_t fx_sd_media_memory[FX_STM32_SD_DEFAULT_SECTOR_SIZE / sizeof(uint32_t)]);
/* Define FileX global data structures.  */


/* USER CODE BEGIN PV */
static UINT media_status;
FX_MEDIA        sdio_disk;
/* Define FileX global data structures.  */
FX_FILE         fx_file;
/* Define ThreadX global data structures.  */
TX_QUEUE        tx_msg_queue;

/**
 *
 *
 * @return iridium_error_code_t
 */
sd_error_code_t sd_config(SD* self)
{

  UINT ret = FX_SUCCESS;
	int fail_counter;
  ULONG r_msg;
  ULONG s_msg = CARD_STATUS_CHANGED;
  ULONG last_status = CARD_STATUS_DISCONNECTED;
	sd_error_code_t return_code = SD_SUCCESS;
  VOID *pointer;
  UINT sd_status = FX_SUCCESS;
  int index = 0;
  

  /* USER CODE BEGIN MX_FileX_Init */
  /* Create the message queue */
  ret = tx_queue_create(&tx_msg_queue, "sd_event_queue", 1, pointer, DEFAULT_QUEUE_LENGTH * sizeof(ULONG));
/* Check main thread creation */
  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

	/* Initialize FileX.  */
  	fx_system_initialize();

/* Open the SD disk driver */
sd_status =  fx_media_open(&sdio_disk, FX_SD_VOLUME_NAME, fx_stm32_sd_driver, (VOID *)FX_NULL, (VOID *) fx_sd_media_memory, sizeof(fx_sd_media_memory));

/* Check the media open sd_status */
  if (sd_status != FX_SUCCESS)
  {
     /* USER CODE BEGIN SD DRIVER get info error */
    while(1);
    /* USER CODE END SD DRIVER get info error */
  }

/* USER CODE BEGIN fx_app_thread_entry 1*/
  fx_media_close_notify_set(&sdio_disk, media_close_callback);

  if(SD_IsDetected(FX_STM32_SD_INSTANCE) == HAL_OK)
  {
    /* SD card is already inserted, place the info into the queue */
    tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
  }
  else
  {
    /* Indicate that SD card is not inserted from start */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }


      while(tx_queue_receive(&tx_msg_queue, &r_msg, TX_TIMER_TICKS_PER_SECOND / 2) != TX_SUCCESS)
      {
        /* Toggle GREEN LED to indicate idle state after a successful operation */
        if(last_status == CARD_STATUS_CONNECTED)
        {
          HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }
      }

      /* check if we received the correct event message */
      if(r_msg == CARD_STATUS_CHANGED)
      {
        /* reset the status */
        r_msg = 0;

        /* for debouncing purpose we wait a bit till it settles down */
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);

        if(SD_IsDetected(FX_STM32_SD_INSTANCE) == HAL_OK)
        {
          /* We have a valid SD insertion event, start processing.. */
          /* Update last known status */
          last_status = CARD_STATUS_CONNECTED;
          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); /*LED_RED Off*/
          //break;
        }
        else
        {
          /* Update last known status */
          last_status = CARD_STATUS_DISCONNECTED;
          HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);  /*LED_GREEN Off*/
          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); /*LED_RED On*/
        }
      }


    /* Create a file called STM32.TXT in the root directory.  */
    if (media_status == MEDIA_CLOSED)
    {
      sd_status = fx_media_open(&sdio_disk, FX_SD_VOLUME_NAME, fx_stm32_sd_driver, (VOID *)FX_NULL, (VOID *) fx_sd_media_memory, sizeof(fx_sd_media_memory));

	  /* Check the media open sd_status */
      if (sd_status != FX_SUCCESS)
      {
        /* Create error, call error handler.  */
        Error_Handler();
      }

      media_status = MEDIA_OPENED;
    }

    snprintf(self->filename, MAX_FILENAME_LEN, "MOMENTS%u.CSV", 0);
    sd_status =  fx_file_create(&sdio_disk, self->filename);

    /* Check the create status.  */
    if (sd_status != FX_SUCCESS)
    {
      /* Check for an already created status. This is expected on the
      second pass of this loop!  */
      if (sd_status != FX_ALREADY_CREATED)
      {
        /* Create error, call error handler.  */
        Error_Handler();
      }
    }

    if (sd_status == FX_ALREADY_CREATED)
    {
    UINT highest_index = 0; // I now need to add an index to the files as in esp one

    if (get_highest_file_index(&highest_index) == FX_SUCCESS)
    {
    printf("Lowest file index not found: %u\n", highest_index);
    }
    else
    {
    printf("No matching files found or error occurred.\n");
    }
    snprintf(self->filename, MAX_FILENAME_LEN, "MOMENTS%u.CSV", highest_index);
    sd_status =  fx_file_create(&sdio_disk, self->filename); 
    // IT'S NOT WORKING HERE BECAUSE WE'RE TRYING TO CREATE 1 BUT IT ALREADY EXISTS
    /* Check the file create status.  */ 
    if (sd_status != FX_SUCCESS)
    {
      /* Error opening file, call error handler.  */
      Error_Handler();
    }
    printf("The created filemane is %s\n", self->filename);
    }

    #if 0
    /* Close the media.  */
    sd_status =  fx_media_close(&sdio_disk);

    /* Check the media close status.  */
    if (sd_status != FX_SUCCESS)
    {
      /* Error closing the media, call error handler.  */
      Error_Handler();
    }
    #endif

  return return_code;
}

sd_error_code_t sd_save_message(SD* self)
{
  sd_error_code_t return_code = SD_SUCCESS;
  UINT sd_status = FX_SUCCESS;
  int pos = 0;
  int i;
  char csv_line[2048];
  memset(csv_line, 0, 2048);
  csv_line[pos] = '\0';
  //printf("sd_save_message\n");

  float Hs = halfToFloat(self->current_message->Hs);
  pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%f,", Hs);
  printf("%s\n", csv_line);

  for (i = 0; i < 42; i++)
  {
      float EW = halfToFloat(self->current_message->EW_array[i]);
      pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%f,", EW);
  }
  printf("%s\n", csv_line);

  for (i = 0; i < 42; i++)
  {
      float E = halfToFloat(self->current_message->E_array[i]);
      pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%f,", E);
  }
  printf("%s\n", csv_line);

    
    for (i = 0; i < 42; i++)
    {
        pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%d,", (int)self->current_message->a1_array[i]);
    }
    printf("%s\n", csv_line);

    for (i = 0; i < 42; i++)
    {
        pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%d,", (int)self->current_message->b1_array[i]);
    }
    printf("%s\n", csv_line);

    for (i = 0; i < 42; i++)
    {
        pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%d,", (int)self->current_message->a2_array[i]);
    }
    printf("%s\n", csv_line);

    for (i = 0; i < 42; i++)
    {
        pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%d,", (int)self->current_message->b2_array[i]);
    }
    printf("%s\n", csv_line);

    for (i = 0; i < 42; i++)
    {
        pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%u,", (unsigned int)self->current_message->cf_array[i]);
    }
    printf("%s\n", csv_line);
    

    /* Write the float fields for Latitude, Longitude, and Timestamp.
     * The last field is followed by a newline instead of a comma.
     */
    printf("Lat: %f, Lon: %f, Timestamp: %f\n", self->current_message->Lat, self->current_message->Lon, self->current_message->timestamp);
    pos += snprintf(csv_line + pos, CSV_BUFFER_SIZE - pos, "%f,%f,%f\n", self->current_message->Lat, self->current_message->Lon, self->current_message->timestamp);

    printf("%s\n", csv_line);
  
  /* Open the test file.  */
  sd_status =  fx_file_open(&sdio_disk, &fx_file, self->filename, FX_OPEN_FOR_WRITE); // THE FILENAME IS EMPTY HERE

  /* Check the file open sd_status */
  if (sd_status != FX_SUCCESS)
  {
     /* Error opening file, call error handler.  */
     Error_Handler();
  }

  // Move the file pointer to the end of the file
  sd_status = fx_file_relative_seek(&fx_file, 0, FX_SEEK_END);
  if (sd_status != FX_SUCCESS)
  {
    /* Error opening file, call error handler.  */
    Error_Handler();
  }
  char dummy [] = "hello,goodbye\n";
  /* Write a string to the test file.  */
  sd_status =  fx_file_write(&fx_file, csv_line, strlen(csv_line));

  /* Check the file write status.  */
  if (sd_status != FX_SUCCESS)
  {
    /* Error writing to a file, call error handler.  */
    Error_Handler();
  }

  /* Close the test file.  */
  sd_status =  fx_file_close(&fx_file);

  /* Check the file close status.  */
  if (sd_status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler.  */
    Error_Handler();
  }

  
  sd_status = fx_media_flush(&sdio_disk);
  /* Check the media flush  status.  */
  if (sd_status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler.  */
    Error_Handler();
  }
  
  

  return return_code;
}

/**
 * Helper method to generate a timestamp from the RTC.
 *
 * @param self - SD struct
 * @return timestamp as a float
 */
float SD_get_timestamp(SD* self)
{
	uint32_t timestamp = 0;
	bool is_leap_year = false;
	uint8_t num_leap_years_since_2000 = 0;
	uint16_t julian_date_first_of_month = 0;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	// Get the date and time
	HAL_RTC_GetTime(self->rtc_handle, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(self->rtc_handle, &rtc_date, RTC_FORMAT_BIN);

	// Let's make a timestamp (yay...)
	// Years first
	timestamp += SECONDS_1970_TO_2000;
	timestamp += rtc_date.Year * SECONDS_IN_YEAR;
	num_leap_years_since_2000 = rtc_date.Year / 4;
	timestamp += num_leap_years_since_2000 * SECONDS_IN_DAY;

	// Years are only represented with 2 digits. We'll set 0 as the year 2000, so anything
	// evenly divisible by 4 is a leap year (2000, 2004, 2008, etc)
	is_leap_year = rtc_date.Year % 4 == 0;

	switch (rtc_date.Month) {
		case RTC_MONTH_JANUARY:
			// No months to account for!!!
			break;

		case RTC_MONTH_FEBRUARY:
			julian_date_first_of_month = 32;
			break;

		case RTC_MONTH_MARCH:
			julian_date_first_of_month = (is_leap_year) ? 61 : 60;
			break;

		case RTC_MONTH_APRIL:
			julian_date_first_of_month = (is_leap_year) ? 92 : 91;
			break;

		case RTC_MONTH_MAY:
			julian_date_first_of_month = (is_leap_year) ? 122 : 121;
			break;

		case RTC_MONTH_JUNE:
			julian_date_first_of_month = (is_leap_year) ? 153 : 152;
			break;

		case RTC_MONTH_JULY:
			julian_date_first_of_month = (is_leap_year) ? 183 : 182;
			break;

		case RTC_MONTH_AUGUST:
			julian_date_first_of_month = (is_leap_year) ? 214 : 213;
			break;

		case RTC_MONTH_SEPTEMBER:
			julian_date_first_of_month = (is_leap_year) ? 245 : 244;
			break;

		case RTC_MONTH_OCTOBER:
			julian_date_first_of_month = (is_leap_year) ? 275 : 274;
			break;

		case RTC_MONTH_NOVEMBER:
			julian_date_first_of_month = (is_leap_year) ? 306 : 305;
			break;

		case RTC_MONTH_DECEMBER:
			julian_date_first_of_month = (is_leap_year) ? 336 : 335;
			break;

		default:
			break;
	}
	timestamp += (julian_date_first_of_month) * SECONDS_IN_DAY;
	timestamp += (rtc_date.Date - 1) * SECONDS_IN_DAY;
	timestamp += rtc_time.Hours * SECONDS_IN_HOUR;
	timestamp += rtc_time.Minutes * SECONDS_IN_MIN;
	timestamp += rtc_time.Seconds;
	// Not including fractions of a second
	return (float)timestamp;
}

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param Instance  SD Instance
 * @retval Returns if SD is detected or not
 */
static uint8_t SD_IsDetected(uint32_t Instance)
{
  uint8_t ret;
  if(Instance >= 1)
  {
    ret = HAL_ERROR;
  }
  else
  {
    /* Check SD card detect pin */
    if (HAL_GPIO_ReadPin(SDCARD_DETECT_GPIO_Port, SDCARD_DETECT_Pin) == GPIO_PIN_SET)
    {
      ret = HAL_ERROR;
    }
    else
    {
      ret = HAL_OK;
    }
  }

  return ret;
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  ULONG s_msg = CARD_STATUS_CHANGED;

  if(GPIO_Pin == SDCARD_DETECT_Pin)
  {
    tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
  }
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  ULONG s_msg = CARD_STATUS_CHANGED;

  if(GPIO_Pin == SDCARD_DETECT_Pin)
  {
    tx_queue_send(&tx_msg_queue, &s_msg, TX_NO_WAIT);
  }
}

/**
  * @brief  Media close notify callback function.
  * @param  media_ptr: Media control block pointer
  * @retval None
  */
static VOID media_close_callback(FX_MEDIA *media_ptr)
{
  media_status = MEDIA_CLOSED;
}


#define BASE_FILENAME "MOMENTS"
#define FILE_EXTENSION ".CSV"

static UINT get_highest_file_index(UINT *highest_index)
{
    CHAR filename[MAX_FILENAME_LEN];
    UINT attributes, year, month, day, hour, minute, second;
    ULONG size;
    int index = 0;
    UINT sd_status = FX_SUCCESS;
    bool found = 0;

    while (found != 1) 
    {
      // Generate file name (e.g., "data_1.csv", "data_2.csv", etc.)
      memset(filename, 0, sizeof(filename));
      snprintf(filename, MAX_FILENAME_LEN, "%s%d%s", BASE_FILENAME, index, FILE_EXTENSION);

      // Try opening the file
      sd_status = fx_file_open(&sdio_disk, &fx_file, filename, FX_OPEN_FOR_WRITE);

      if (sd_status == FX_NOT_FOUND) {
          // File does not exist, so we can use this index
          *highest_index = index;
          return FX_SUCCESS;
          found = 1;
      }

      // If file exists, close it and check the next index
      fx_file_close(&fx_file);
      index++;
  } 
    return FX_SUCCESS;
}
