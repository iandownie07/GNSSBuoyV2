/**
 * @file buoy_ekf_3d.h
 * @brief 3D Extended Kalman Filter for INS/GNSS Fusion on Buoy
 */

#ifndef BUOY_EKF_3D_H
#define BUOY_EKF_3D_H

#include <stdint.h>

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
 * @brief Process noise standard deviations (Q matrix diagonal)
 */
typedef struct {
    float sigma_pos;      // Position process noise (m)
    float sigma_vel;      // Velocity process noise (m/s)
    float sigma_att;      // Attitude process noise (rad)
    float sigma_bg;       // Gyro bias random walk (rad/s)
    float sigma_ba;       // Accel bias random walk (m/s^2)
} process_noise_params_t;

/**
 * @brief Measurement noise standard deviations (R matrix diagonal)
 */
typedef struct {
    float sigma_pos_n;    // North position noise (m)
    float sigma_pos_e;    // East position noise (m)
    float sigma_pos_d;    // Down position noise (m)
    float sigma_vel_n;    // North velocity noise (m/s)
    float sigma_vel_e;    // East velocity noise (m/s)
    float sigma_vel_d;    // Down velocity noise (m/s)
} meas_noise_params_t;

/* ========================================================================== */
/* PUBLIC FUNCTIONS */
/* ========================================================================== */

/**
 * @brief Initialize EKF with initial state and covariance
 * 
 * @param pos_n Initial north position (m)
 * @param pos_e Initial east position (m)
 * @param pos_d Initial down position (m)
 * @param vel_n Initial north velocity (m/s)
 * @param vel_e Initial east velocity (m/s)
 * @param vel_d Initial down velocity (m/s)
 * @param roll Initial roll angle (rad)
 * @param pitch Initial pitch angle (rad)
 * @param yaw Initial yaw angle (rad)
 */
void ekf_init(float pos_n, float pos_e, float pos_d,
              float vel_n, float vel_e, float vel_d,
              float roll, float pitch, float yaw);

/**
 * @brief EKF prediction step (call at IMU rate, 100-200 Hz)
 * 
 * @param imu IMU measurement data
 */
void ekf_predict(const imu_data_t *imu);

/**
 * @brief EKF update step (call when GNSS available, 1-5 Hz)
 * 
 * @param gnss GNSS measurement data
 */
void ekf_update(const gnss_data_t *gnss);

/**
 * @brief Get current state estimate
 * 
 * @param state Pointer to store current state
 */
void ekf_get_state(nav_state_t *state);

/**
 * @brief Get velocity buffer for spectral analysis
 * 
 * @param vel_n Pointer to north velocity buffer
 * @param vel_e Pointer to east velocity buffer
 * @param vel_d Pointer to down velocity buffer
 * @param size Current buffer size (index)
 * @param full Flag indicating if buffer has been filled once
 */
void ekf_get_velocity_buffer(float **vel_n, float **vel_e, float **vel_d, 
                             uint32_t *size, uint8_t *full);

/**
 * @brief Set process noise parameters
 * 
 * @param params Process noise parameters
 */
void ekf_set_process_noise(const process_noise_params_t *params);

/**
 * @brief Set measurement noise parameters
 * 
 * @param params Measurement noise parameters
 */
void ekf_set_meas_noise(const meas_noise_params_t *params);

/**
 * @brief Get position uncertainty (standard deviation)
 * 
 * @param sigma_n North position uncertainty (m)
 * @param sigma_e East position uncertainty (m)
 * @param sigma_d Down position uncertainty (m)
 */
void ekf_get_position_uncertainty(float *sigma_n, float *sigma_e, float *sigma_d);

/**
 * @brief Get velocity uncertainty (standard deviation)
 * 
 * @param sigma_vn North velocity uncertainty (m/s)
 * @param sigma_ve East velocity uncertainty (m/s)
 * @param sigma_vd Down velocity uncertainty (m/s)
 */
void ekf_get_velocity_uncertainty(float *sigma_vn, float *sigma_ve, float *sigma_vd);

/**
 * @brief Get attitude uncertainty (standard deviation in radians)
 * 
 * @param sigma_roll Roll uncertainty (rad)
 * @param sigma_pitch Pitch uncertainty (rad)
 * @param sigma_yaw Yaw uncertainty (rad)
 */
void ekf_get_attitude_uncertainty(float *sigma_roll, float *sigma_pitch, float *sigma_yaw);

#endif /* BUOY_EKF_3D_H */
