/*
 * ekf.h
 *
 *  Created on: Jan 2025
 *      Author: Ian
 */

#ifndef SRC_EKF_H_
#define SRC_EKF_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"

/* ========================================================================== */
/* CONSTANTS */
/* ========================================================================== */

#define GRAVITY 9.81f  // m/s^2

#define EKF_STATE_SIZE 16           // Nominal state dimension
#define EKF_ERROR_STATE_SIZE 15     // Error state dimension
#define EKF_MEAS_SIZE 6             // Measurement dimension (GNSS pos + vel)

/* ========================================================================== */
/* ERROR CODES */
/* ========================================================================== */

typedef enum ekf_error_code {
    EKF_SUCCESS = 0,
    EKF_IMU_READ_ERROR = -1,
    EKF_INIT_ERROR = -2,
    EKF_MATRIX_ERROR = -3,
    EKF_INVALID_GNSS = -4
} ekf_error_code_t;

/* ========================================================================== */
/* DATA STRUCTURES */
/* ========================================================================== */

/**
 * @brief Nominal navigation state (16 elements)
 */
typedef struct {
    float pos_n;     // North position (m)
    float pos_e;     // East position (m)
    float pos_d;     // Down position (m)
    
    float vel_n;     // North velocity (m/s)
    float vel_e;     // East velocity (m/s)
    float vel_d;     // Down velocity (m/s)
    
    float q0;        // Quaternion scalar part
    float q1;        // Quaternion vector x
    float q2;        // Quaternion vector y
    float q3;        // Quaternion vector z
    
    float b_gx;      // Gyro bias x (rad/s)
    float b_gy;      // Gyro bias y (rad/s)
    float b_gz;      // Gyro bias z (rad/s)
    
    float b_ax;      // Accel bias x (m/s^2)
    float b_ay;      // Accel bias y (m/s^2)
    float b_az;      // Accel bias z (m/s^2)
} nav_state_t;

/**
 * @brief IMU measurements
 */
typedef struct {
    float accel_x;   // Body frame acceleration (m/s^2)
    float accel_y;
    float accel_z;
    
    float gyro_x;    // Body frame angular rate (rad/s)
    float gyro_y;
    float gyro_z;
    
    float dt;        // Time since last measurement (s)
} imu_data_t;

/**
 * @brief GNSS measurements
 */
typedef struct {
    float pos_n;     // North position (m)
    float pos_e;     // East position (m)
    float pos_d;     // Down position (m)
    
    float vel_n;     // North velocity (m/s)
    float vel_e;     // East velocity (m/s)
    float vel_d;     // Down velocity (m/s)
    
    uint8_t valid;   // Measurement validity flag
} gnss_data_t;

/**
 * @brief Process noise parameters (Q matrix diagonal)
 */
typedef struct {
    float sigma_pos;      // Position process noise (m)
    float sigma_vel;      // Velocity process noise (m/s)
    float sigma_att;      // Attitude process noise (rad)
    float sigma_bg;       // Gyro bias random walk (rad/s)
    float sigma_ba;       // Accel bias random walk (m/s^2)
} process_noise_params_t;

/**
 * @brief Measurement noise parameters (R matrix diagonal)
 */
typedef struct {
    float sigma_pos_n;    // North position noise (m)
    float sigma_pos_e;    // East position noise (m)
    float sigma_pos_d;    // Down position noise (m)
    float sigma_vel_n;    // North velocity noise (m/s)
    float sigma_vel_e;    // East velocity noise (m/s)
    float sigma_vel_d;    // Down velocity noise (m/s)
} meas_noise_params_t;

/**
 * @brief Velocity buffer for spectral analysis
 */
typedef struct {
    float vel_n[TOTAL_SAMPLES_PER_WINDOW];
    float vel_e[TOTAL_SAMPLES_PER_WINDOW];
    float vel_d[TOTAL_SAMPLES_PER_WINDOW];
    uint32_t index;
    uint8_t full;
} velocity_buffer_t;

/**
 * @brief EKF statistics for diagnostics
 */
typedef struct {
    uint32_t predict_count;
    uint32_t update_count;
    uint32_t imu_errors;
    uint32_t gnss_rejected;
} ekf_stats_t;

/**
 * @brief Main EKF structure (following SD card pattern)
 */
typedef struct EKF {
    // Configuration
    microSWIFT_configuration* global_config;
    
    // Event flags and control
    TX_EVENT_FLAGS_GROUP* control_flags;
    TX_EVENT_FLAGS_GROUP* error_flags;
    
    // I2C handle for IMU
    //I2C_HandleTypeDef* imu_i2c_handle;
    
    // Pointers to external arrays (spectral analysis)
    float* GNSS_N_Array;
    float* GNSS_E_Array;
    float* GNSS_D_Array;
    volatile uint32_t* total_samples;
    
    // Current state
    nav_state_t state;
    
    // Covariance matrix (15x15)
    float32_t P_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
    arm_matrix_instance_f32 P;
    
    // Tuning parameters
    process_noise_params_t Q_params;
    meas_noise_params_t R_params;
    
    // Velocity buffer for spectral analysis
    velocity_buffer_t vel_buffer;
    
    // Statistics
    ekf_stats_t stats;
    
    // Reference position for NED frame
    double ref_lat;
    double ref_lon;
    double ref_alt;
    bool ref_initialized;
    
    // Timestamps
    uint32_t last_predict_time;
    uint32_t last_update_time;
    
    // Method pointers (following SD pattern)
    ekf_error_code_t (*config)(struct EKF* self);
    ekf_error_code_t (*predict)(struct EKF* self, const imu_data_t* imu);
    ekf_error_code_t (*update)(struct EKF* self, const gnss_data_t* gnss);
    ekf_error_code_t (*get_state)(struct EKF* self, nav_state_t* state);
    ekf_error_code_t (*get_velocity_buffer)(struct EKF* self, float** vel_n, float** vel_e, 
                                            float** vel_d, uint32_t* size, uint8_t* full);
    ekf_error_code_t (*set_process_noise)(struct EKF* self, const process_noise_params_t* params);
    ekf_error_code_t (*set_meas_noise)(struct EKF* self, const meas_noise_params_t* params);
    ekf_error_code_t (*get_uncertainties)(struct EKF* self, float* sigma_pos, float* sigma_vel, 
                                          float* sigma_att);
    void (*print_stats)(struct EKF* self);
    
} EKF;

/* ========================================================================== */
/* PUBLIC FUNCTIONS */
/* ========================================================================== */

/**
 * @brief Initialize EKF structure
 * 
 * @param self EKF structure pointer
 * @param global_config Global configuration
 * @param control_flags Control event flags
 * @param error_flags Error event flags
 * @param imu_i2c_handle I2C handle for IMU communication
 * @param gnss_n_array Pointer to North velocity array for spectral analysis
 * @param gnss_e_array Pointer to East velocity array for spectral analysis
 * @param gnss_d_array Pointer to Down velocity array for spectral analysis
 * @param total_samples Pointer to sample counter
 */
void ekf_init(EKF* self, 
              microSWIFT_configuration* global_config,
              TX_EVENT_FLAGS_GROUP* control_flags, 
              TX_EVENT_FLAGS_GROUP* error_flags,
              //I2C_HandleTypeDef* imu_i2c_handle,
              float* gnss_n_array,
              float* gnss_e_array,
              float* gnss_d_array,
              volatile uint32_t* total_samples);

/**
 * @brief Configure and initialize EKF with initial state
 * 
 * @param self EKF structure pointer
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_config(EKF* self);

/**
 * @brief Run EKF prediction step with IMU data
 * 
 * @param self EKF structure pointer
 * @param imu IMU measurement data
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_predict(EKF* self, const imu_data_t* imu);

/**
 * @brief Run EKF update step with GNSS data
 * 
 * @param self EKF structure pointer
 * @param gnss GNSS measurement data
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_update(EKF* self, const gnss_data_t* gnss);

/**
 * @brief Get current state estimate
 * 
 * @param self EKF structure pointer
 * @param state Output state structure
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_get_state(EKF* self, nav_state_t* state);

/**
 * @brief Get velocity buffer for spectral analysis
 * 
 * @param self EKF structure pointer
 * @param vel_n Output pointer to North velocity buffer
 * @param vel_e Output pointer to East velocity buffer
 * @param vel_d Output pointer to Down velocity buffer
 * @param size Output current buffer size
 * @param full Output flag indicating if buffer has been filled
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_get_velocity_buffer(EKF* self, float** vel_n, float** vel_e, 
                                         float** vel_d, uint32_t* size, uint8_t* full);

/**
 * @brief Set process noise parameters
 * 
 * @param self EKF structure pointer
 * @param params Process noise parameters
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_set_process_noise(EKF* self, const process_noise_params_t* params);

/**
 * @brief Set measurement noise parameters
 * 
 * @param self EKF structure pointer
 * @param params Measurement noise parameters
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_set_meas_noise(EKF* self, const meas_noise_params_t* params);

/**
 * @brief Get state uncertainties
 * 
 * @param self EKF structure pointer
 * @param sigma_pos Position uncertainty (m)
 * @param sigma_vel Velocity uncertainty (m/s)
 * @param sigma_att Attitude uncertainty (rad)
 * @return ekf_error_code_t
 */
ekf_error_code_t ekf_get_uncertainties(EKF* self, float* sigma_pos, 
                                       float* sigma_vel, float* sigma_att);

/**
 * @brief Print EKF statistics
 * 
 * @param self EKF structure pointer
 */
void ekf_print_stats(EKF* self);

/**
 * @brief Convert latitude/longitude/altitude to local NED frame
 * 
 * @param lat Latitude (degrees)
 * @param lon Longitude (degrees)
 * @param alt Altitude (m)
 * @param lat0 Reference latitude (degrees)
 * @param lon0 Reference longitude (degrees)
 * @param alt0 Reference altitude (m)
 * @param n North position output (m)
 * @param e East position output (m)
 * @param d Down position output (m)
 */
void lla_to_ned(double lat, double lon, double alt,
                double lat0, double lon0, double alt0,
                float* n, float* e, float* d);

#endif /* SRC_EKF_H_ */
