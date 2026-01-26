/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "main.h"
#include "stdbool.h"
#include "stdint.h"
#include "NEDWaves/rtwhalf.h"



// Number of bands in each spectrum
#define TOTAL_BANDS_PER_SPECTRUM 42

typedef struct sbd_message_type_52 {
//		 char			legacy_number_7;
//		 uint8_t 		type;
//		 uint8_t 		port;
//__packed uint16_t 		size;
__packed real16_T 		Hs;
//__packed real16_T		Tp;
//__packed real16_T 		Dp;
__packed real16_T	 	EW_array[TOTAL_BANDS_PER_SPECTRUM];
__packed real16_T	 	E_array[TOTAL_BANDS_PER_SPECTRUM];
//__packed real16_T		f_min;
//__packed real16_T		f_max;
		 signed char	a1_array[TOTAL_BANDS_PER_SPECTRUM];
		 signed char	b1_array[TOTAL_BANDS_PER_SPECTRUM];
		 signed char	a2_array[TOTAL_BANDS_PER_SPECTRUM];
		 signed char	b2_array[TOTAL_BANDS_PER_SPECTRUM];
		 unsigned char	cf_array[TOTAL_BANDS_PER_SPECTRUM];
__packed float			Lat;
__packed float			Lon;
//__packed real16_T		mean_temp;
//__packed real16_T		mean_salinity;
//__packed real16_T		mean_voltage;
__packed float			timestamp;
//		 uint8_t		checksum_a;
//		 uint8_t		checksum_b;
} sbd_message_type_52;

// For testing and debugging with a very short sample window
//#define DEBUGGING_FAST_CYCLE

// If a 1 min sleep window is desired
#define SHORT_SLEEP

#ifdef DEBUGGING_FAST_CYCLE

#define TOTAL_SAMPLES_PER_WINDOW 256
#define IRIDIUM_MAX_TRANSMIT_TIME 10
#define GNSS_MAX_ACQUISITION_WAIT_TIME 15

#else
// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 8192	//8192

// The max time in MINUTES to try to get an Iridium message off
#define IRIDIUM_MAX_TRANSMIT_TIME 10
// The max time in MINUTES without good data from GNSS before commanding to sleep
// !! Must be greater than 0
#define GNSS_MAX_ACQUISITION_WAIT_TIME 15
#endif

// Sampling rate in Hz for the GNSS sensor
#define GNSS_SAMPLING_RATE 5
// The number of samples for the CT sensor to take. Result will be averaged
#define TOTAL_CT_SAMPLES 10
// If the IMU will be utilized or not
#define IMU_ENABLED false
// If there is a CT sensor present
#define CT_ENABLED false
// If there is IRIDIUM present
#define IRIDIUM_ENABLED false
// If there is GNSS present
#define GNSS_ENABLED true
// If there is WAVES present
#define WAVES_ENABLED true

typedef struct microSWIFT_configuration{
	uint32_t samples_per_window;
	uint32_t iridium_max_transmit_time;
	uint32_t gnss_max_acquisition_wait_time;
	uint32_t gnss_sampling_rate;
	uint32_t total_ct_samples;
} microSWIFT_configuration;


#endif /* INC_CONFIGURATION_H_ */
