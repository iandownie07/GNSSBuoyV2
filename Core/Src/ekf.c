/*
 * ekf.c
 *
 *  Created on: Jan 2025
 *      Author: Ian
 */

#include "ekf.h"
#include <math.h>

/* ========================================================================== */
/* STATIC MEMORY ALLOCATION */
/* ========================================================================== */

/* Working matrices for EKF computations (statically allocated) */
static float32_t F_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t Phi_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t Q_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t H_data[EKF_MEAS_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t R_data[EKF_MEAS_SIZE * EKF_MEAS_SIZE];
static float32_t K_data[EKF_ERROR_STATE_SIZE * EKF_MEAS_SIZE];

static float32_t temp1_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t temp2_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
static float32_t temp3_data[EKF_ERROR_STATE_SIZE * EKF_MEAS_SIZE];
static float32_t temp4_data[EKF_MEAS_SIZE * EKF_MEAS_SIZE];
static float32_t temp5_data[EKF_MEAS_SIZE * EKF_ERROR_STATE_SIZE];

static float32_t innovation_data[EKF_MEAS_SIZE];
static float32_t error_state_data[EKF_ERROR_STATE_SIZE];

static arm_matrix_instance_f32 F, Phi, Q, H, R, K;
static arm_matrix_instance_f32 temp1, temp2, temp3, temp4, temp5;
static arm_matrix_instance_f32 innovation, error_state;

/* ========================================================================== */
/* QUATERNION OPERATIONS */
/* ========================================================================== */

/**
 * @brief Normalize quaternion to unit length
 */
static void quat_normalize(float *q0, float *q1, float *q2, float *q3) {
    float norm = sqrtf(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    if (norm > 1e-8f) {
        float inv_norm = 1.0f / norm;
        *q0 *= inv_norm;
        *q1 *= inv_norm;
        *q2 *= inv_norm;
        *q3 *= inv_norm;
    }
}

/**
 * @brief Quaternion multiplication: q_out = q_a ⊗ q_b
 */
static void quat_multiply(float a0, float a1, float a2, float a3,
                          float b0, float b1, float b2, float b3,
                          float *c0, float *c1, float *c2, float *c3) {
    *c0 = a0*b0 - a1*b1 - a2*b2 - a3*b3;
    *c1 = a0*b1 + a1*b0 + a2*b3 - a3*b2;
    *c2 = a0*b2 - a1*b3 + a2*b0 + a3*b1;
    *c3 = a0*b3 + a1*b2 - a2*b1 + a3*b0;
}

/**
 * @brief Convert quaternion to rotation matrix (body to NED)
 */
static void quat_to_rotation_matrix(float q0, float q1, float q2, float q3, float *C) {
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    
    C[0] = q0q0 + q1q1 - q2q2 - q3q3;
    C[1] = 2.0f * (q1q2 + q0q3);
    C[2] = 2.0f * (q1q3 - q0q2);
    
    C[3] = 2.0f * (q1q2 - q0q3);
    C[4] = q0q0 - q1q1 + q2q2 - q3q3;
    C[5] = 2.0f * (q2q3 + q0q1);
    
    C[6] = 2.0f * (q1q3 + q0q2);
    C[7] = 2.0f * (q2q3 - q0q1);
    C[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**
 * @brief Create skew-symmetric matrix from 3-vector
 */
static void skew_symmetric(float v1, float v2, float v3, float *S) {
    S[0] =   0.0f;  S[1] = -v3;     S[2] =  v2;
    S[3] =   v3;    S[4] =  0.0f;   S[5] = -v1;
    S[6] =  -v2;    S[7] =  v1;     S[8] =  0.0f;
}

/* ========================================================================== */
/* UTILITY FUNCTIONS */
/* ========================================================================== */

/**
 * @brief Convert latitude/longitude/altitude to local NED frame
 */
void lla_to_ned(double lat, double lon, double alt,
                double lat0, double lon0, double alt0,
                float* n, float* e, float* d) {
    const double R_earth = 6378137.0;  // WGS84 equatorial radius (m)
    const double deg2rad = M_PI / 180.0;
    
    double lat_rad = lat * deg2rad;
    double lon_rad = lon * deg2rad;
    double lat0_rad = lat0 * deg2rad;
    double lon0_rad = lon0 * deg2rad;
    
    double dlat = lat_rad - lat0_rad;
    double dlon = lon_rad - lon0_rad;
    
    *n = (float)(R_earth * dlat);
    *e = (float)(R_earth * cos(lat0_rad) * dlon);
    *d = (float)(alt0 - alt);  // Down is negative altitude
}

/* ========================================================================== */
/* INITIALIZATION */
/* ========================================================================== */

/**
 * @brief Initialize EKF structure (following SD pattern)
 */
void ekf_init(EKF* self, 
              microSWIFT_configuration* global_config,
              TX_EVENT_FLAGS_GROUP* control_flags, 
              TX_EVENT_FLAGS_GROUP* error_flags,
              //I2C_HandleTypeDef* imu_i2c_handle,
              float* gnss_n_array,
              float* gnss_e_array,
              float* gnss_d_array,
              volatile uint32_t* total_samples)
{
    // Store configuration and handles
    self->global_config = global_config;
    self->control_flags = control_flags;
    self->error_flags = error_flags;
    //self->imu_i2c_handle = imu_i2c_handle;
    
    // Store pointers to spectral analysis arrays
    self->GNSS_N_Array = gnss_n_array;
    self->GNSS_E_Array = gnss_e_array;
    self->GNSS_D_Array = gnss_d_array;
    self->total_samples = total_samples;
    
    // Initialize state to zero
    memset(&self->state, 0, sizeof(nav_state_t));
    
    // Initialize quaternion to identity (no rotation)
    self->state.q0 = 1.0f;
    self->state.q1 = 0.0f;
    self->state.q2 = 0.0f;
    self->state.q3 = 0.0f;
    
    // Initialize covariance matrix wrapper
    arm_mat_init_f32(&self->P, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, self->P_data);
    
    // Set default tuning parameters
    self->Q_params.sigma_pos = 0.01f;
    self->Q_params.sigma_vel = 0.1f;
    self->Q_params.sigma_att = 0.01f;
    self->Q_params.sigma_bg = 1e-4f;
    self->Q_params.sigma_ba = 1e-3f;
    
    self->R_params.sigma_pos_n = 2.0f;
    self->R_params.sigma_pos_e = 2.0f;
    self->R_params.sigma_pos_d = 3.0f;
    self->R_params.sigma_vel_n = 0.05f;
    self->R_params.sigma_vel_e = 0.05f;
    self->R_params.sigma_vel_d = 0.1f;
    
    // Initialize velocity buffer
    memset(&self->vel_buffer, 0, sizeof(velocity_buffer_t));
    
    // Initialize statistics
    memset(&self->stats, 0, sizeof(ekf_stats_t));
    
    // Initialize reference position flag
    self->ref_initialized = false;
    
    // Initialize timestamps
    self->last_predict_time = 0;
    self->last_update_time = 0;
    
    // Assign method pointers (following SD pattern)
    self->config = ekf_config;
    self->predict = ekf_predict;
    self->update = ekf_update;
    self->get_state = ekf_get_state;
    self->get_velocity_buffer = ekf_get_velocity_buffer;
    self->set_process_noise = ekf_set_process_noise;
    self->set_meas_noise = ekf_set_meas_noise;
    self->get_uncertainties = ekf_get_uncertainties;
    self->print_stats = ekf_print_stats;
    
    // Initialize working matrix instances
    arm_mat_init_f32(&F, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, F_data);
    arm_mat_init_f32(&Phi, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, Phi_data);
    arm_mat_init_f32(&Q, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, Q_data);
    arm_mat_init_f32(&H, EKF_MEAS_SIZE, EKF_ERROR_STATE_SIZE, H_data);
    arm_mat_init_f32(&R, EKF_MEAS_SIZE, EKF_MEAS_SIZE, R_data);
    arm_mat_init_f32(&K, EKF_ERROR_STATE_SIZE, EKF_MEAS_SIZE, K_data);
    
    arm_mat_init_f32(&temp1, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, temp1_data);
    arm_mat_init_f32(&temp2, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, temp2_data);
    arm_mat_init_f32(&temp3, EKF_ERROR_STATE_SIZE, EKF_MEAS_SIZE, temp3_data);
    arm_mat_init_f32(&temp4, EKF_MEAS_SIZE, EKF_MEAS_SIZE, temp4_data);
    arm_mat_init_f32(&temp5, EKF_MEAS_SIZE, EKF_ERROR_STATE_SIZE, temp5_data);
    
    arm_mat_init_f32(&innovation, EKF_MEAS_SIZE, 1, innovation_data);
    arm_mat_init_f32(&error_state, EKF_ERROR_STATE_SIZE, 1, error_state_data);
}

/**
 * @brief Configure and initialize EKF with initial state from GNSS
 */
ekf_error_code_t ekf_config(EKF* self)
{
    printf("EKF: Configuring...\n");
    
    // Initialize covariance matrix P with initial uncertainties
    memset(self->P_data, 0, sizeof(self->P_data));
    
    float sigma_pos_init = 10.0f;      // 10 m initial position uncertainty
    float sigma_vel_init = 1.0f;       // 1 m/s initial velocity uncertainty
    float sigma_att_init = 0.1f;       // ~5.7 deg initial attitude uncertainty
    float sigma_bg_init = 0.01f;       // Initial gyro bias uncertainty
    float sigma_ba_init = 0.1f;        // Initial accel bias uncertainty
    
    for (int i = 0; i < 3; i++) {
        self->P_data[i * EKF_ERROR_STATE_SIZE + i] = sigma_pos_init * sigma_pos_init;
        self->P_data[(i+3) * EKF_ERROR_STATE_SIZE + (i+3)] = sigma_vel_init * sigma_vel_init;
        self->P_data[(i+6) * EKF_ERROR_STATE_SIZE + (i+6)] = sigma_att_init * sigma_att_init;
        self->P_data[(i+9) * EKF_ERROR_STATE_SIZE + (i+9)] = sigma_bg_init * sigma_bg_init;
        self->P_data[(i+12) * EKF_ERROR_STATE_SIZE + (i+12)] = sigma_ba_init * sigma_ba_init;
    }
    
    printf("EKF: Configuration complete\n");
    return EKF_SUCCESS;
}

/* ========================================================================== */
/* PREDICTION STEP */
/* ========================================================================== */

/**
 * @brief Propagate nominal state using IMU measurements
 */
static void propagate_nominal_state(EKF* self, const imu_data_t* imu) {
    float dt = imu->dt;
    
    // Compensate for biases
    float omega_x = imu->gyro_x - self->state.b_gx;
    float omega_y = imu->gyro_y - self->state.b_gy;
    float omega_z = imu->gyro_z - self->state.b_gz;
    
    float accel_x = imu->accel_x - self->state.b_ax;
    float accel_y = imu->accel_y - self->state.b_ay;
    float accel_z = imu->accel_z - self->state.b_az;
    
    // Update quaternion
    float q0 = self->state.q0;
    float q1 = self->state.q1;
    float q2 = self->state.q2;
    float q3 = self->state.q3;
    
    float q0_dot = 0.5f * (-q1*omega_x - q2*omega_y - q3*omega_z);
    float q1_dot = 0.5f * ( q0*omega_x + q2*omega_z - q3*omega_y);
    float q2_dot = 0.5f * ( q0*omega_y - q1*omega_z + q3*omega_x);
    float q3_dot = 0.5f * ( q0*omega_z + q1*omega_y - q2*omega_x);
    
    self->state.q0 += q0_dot * dt;
    self->state.q1 += q1_dot * dt;
    self->state.q2 += q2_dot * dt;
    self->state.q3 += q3_dot * dt;
    
    quat_normalize(&self->state.q0, &self->state.q1, &self->state.q2, &self->state.q3);
    
    // Transform acceleration to NED frame
    float C[9];
    quat_to_rotation_matrix(self->state.q0, self->state.q1, self->state.q2, self->state.q3, C);
    
    float accel_n = C[0]*accel_x + C[1]*accel_y + C[2]*accel_z;
    float accel_e = C[3]*accel_x + C[4]*accel_y + C[5]*accel_z;
    float accel_d = C[6]*accel_x + C[7]*accel_y + C[8]*accel_z;
    
    // Subtract gravity
    accel_d -= GRAVITY;
    
    // Update velocity
    self->state.vel_n += accel_n * dt;
    self->state.vel_e += accel_e * dt;
    self->state.vel_d += accel_d * dt;
    
    // Update position
    self->state.pos_n += self->state.vel_n * dt;
    self->state.pos_e += self->state.vel_e * dt;
    self->state.pos_d += self->state.vel_d * dt;
}

/**
 * @brief Compute error-state transition matrix F
 */
static void compute_F_matrix(EKF* self, const imu_data_t* imu) {
    memset(F_data, 0, sizeof(F_data));
    
    float C[9];
    quat_to_rotation_matrix(self->state.q0, self->state.q1, self->state.q2, self->state.q3, C);
    
    float accel_x = imu->accel_x - self->state.b_ax;
    float accel_y = imu->accel_y - self->state.b_ay;
    float accel_z = imu->accel_z - self->state.b_az;
    
    float f_n = C[0]*accel_x + C[1]*accel_y + C[2]*accel_z;
    float f_e = C[3]*accel_x + C[4]*accel_y + C[5]*accel_z;
    float f_d = C[6]*accel_x + C[7]*accel_y + C[8]*accel_z - GRAVITY;
    
    float S_f[9];
    skew_symmetric(f_n, f_e, f_d, S_f);
    
    // Position error from velocity error = I₃
    F_data[0*15 + 3] = 1.0f;
    F_data[1*15 + 4] = 1.0f;
    F_data[2*15 + 5] = 1.0f;
    
    // Velocity error from attitude error = S(f)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(3+i)*15 + (6+j)] = S_f[i*3 + j];
        }
    }
    
    // Velocity error from accel bias error = -C_b^l
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(3+i)*15 + (12+j)] = -C[i*3 + j];
        }
    }
    
    // Attitude error from gyro bias error = -C_b^l
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(6+i)*15 + (9+j)] = -C[i*3 + j];
        }
    }
}

/**
 * @brief Discretize F to get state transition matrix Phi
 */
static void compute_Phi_matrix(float dt) {
    memset(Phi_data, 0, sizeof(Phi_data));
    for (int i = 0; i < EKF_ERROR_STATE_SIZE; i++) {
        Phi_data[i*EKF_ERROR_STATE_SIZE + i] = 1.0f;
    }
    
    for (int i = 0; i < EKF_ERROR_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_ERROR_STATE_SIZE; j++) {
            Phi_data[i*EKF_ERROR_STATE_SIZE + j] += F_data[i*EKF_ERROR_STATE_SIZE + j] * dt;
        }
    }
}

/**
 * @brief Compute process noise covariance matrix Q
 */
static void compute_Q_matrix(EKF* self, float dt) {
    memset(Q_data, 0, sizeof(Q_data));
    float dt2 = dt * dt;
    
    for (int i = 0; i < 3; i++) {
        Q_data[i*EKF_ERROR_STATE_SIZE + i] = self->Q_params.sigma_pos * self->Q_params.sigma_pos * dt2;
    }
    
    for (int i = 0; i < 3; i++) {
        Q_data[(3+i)*EKF_ERROR_STATE_SIZE + (3+i)] = self->Q_params.sigma_vel * self->Q_params.sigma_vel * dt2;
    }
    
    for (int i = 0; i < 3; i++) {
        Q_data[(6+i)*EKF_ERROR_STATE_SIZE + (6+i)] = self->Q_params.sigma_att * self->Q_params.sigma_att * dt2;
    }
    
    for (int i = 0; i < 3; i++) {
        Q_data[(9+i)*EKF_ERROR_STATE_SIZE + (9+i)] = self->Q_params.sigma_bg * self->Q_params.sigma_bg * dt2;
    }
    
    for (int i = 0; i < 3; i++) {
        Q_data[(12+i)*EKF_ERROR_STATE_SIZE + (12+i)] = self->Q_params.sigma_ba * self->Q_params.sigma_ba * dt2;
    }
}

/**
 * @brief Propagate error covariance
 */
static void propagate_covariance(EKF* self) {
    arm_mat_mult_f32(&Phi, &self->P, &temp1);
    arm_mat_trans_f32(&Phi, &temp2);
    arm_mat_mult_f32(&temp1, &temp2, &self->P);
    arm_mat_add_f32(&self->P, &Q, &self->P);
}

/**
 * @brief EKF prediction step
 */
ekf_error_code_t ekf_predict(EKF* self, const imu_data_t* imu)
{
    // Propagate nominal state
    propagate_nominal_state(self, imu);
    
    // Compute F matrix
    compute_F_matrix(self, imu);
    
    // Discretize to get Phi
    compute_Phi_matrix(imu->dt);
    
    // Compute Q matrix
    compute_Q_matrix(self, imu->dt);
    
    // Propagate covariance
    propagate_covariance(self);
    
    // Store velocity in buffer for spectral analysis
    if (self->vel_buffer.index < TOTAL_SAMPLES_PER_WINDOW ) {
        self->vel_buffer.vel_n[self->vel_buffer.index] = self->state.vel_n;
        self->vel_buffer.vel_e[self->vel_buffer.index] = self->state.vel_e;
        self->vel_buffer.vel_d[self->vel_buffer.index] = self->state.vel_d;
        
        self->vel_buffer.index++;
        if (self->vel_buffer.index >= TOTAL_SAMPLES_PER_WINDOW ) {
            self->vel_buffer.index = 0;
            self->vel_buffer.full = 1;
        }
    }
    
    // Update statistics
    self->stats.predict_count++;
    self->last_predict_time = HAL_GetTick();
    
    return EKF_SUCCESS;
}

/* ========================================================================== */
/* UPDATE STEP */
/* ========================================================================== */

/**
 * @brief Compute measurement matrix H
 */
static void compute_H_matrix(void) {
    memset(H_data, 0, sizeof(H_data));
    for (int i = 0; i < EKF_MEAS_SIZE; i++) {
        H_data[i*EKF_ERROR_STATE_SIZE + i] = 1.0f;
    }
}

/**
 * @brief Compute measurement noise covariance matrix R
 */
static void compute_R_matrix(EKF* self) {
    memset(R_data, 0, sizeof(R_data));
    R_data[0*EKF_MEAS_SIZE + 0] = self->R_params.sigma_pos_n * self->R_params.sigma_pos_n;
    R_data[1*EKF_MEAS_SIZE + 1] = self->R_params.sigma_pos_e * self->R_params.sigma_pos_e;
    R_data[2*EKF_MEAS_SIZE + 2] = self->R_params.sigma_pos_d * self->R_params.sigma_pos_d;
    R_data[3*EKF_MEAS_SIZE + 3] = self->R_params.sigma_vel_n * self->R_params.sigma_vel_n;
    R_data[4*EKF_MEAS_SIZE + 4] = self->R_params.sigma_vel_e * self->R_params.sigma_vel_e;
    R_data[5*EKF_MEAS_SIZE + 5] = self->R_params.sigma_vel_d * self->R_params.sigma_vel_d;
}

/**
 * @brief Compute innovation (measurement residual)
 */
static void compute_innovation(EKF* self, const gnss_data_t* gnss) {
    innovation_data[0] = gnss->pos_n - self->state.pos_n;
    innovation_data[1] = gnss->pos_e - self->state.pos_e;
    innovation_data[2] = gnss->pos_d - self->state.pos_d;
    innovation_data[3] = gnss->vel_n - self->state.vel_n;
    innovation_data[4] = gnss->vel_e - self->state.vel_e;
    innovation_data[5] = gnss->vel_d - self->state.vel_d;
}

/**
 * @brief Compute Kalman gain
 */
static void compute_kalman_gain(EKF* self) {
    arm_mat_mult_f32(&H, &self->P, &temp5);
    
    arm_matrix_instance_f32 H_trans;
    float32_t H_trans_data[EKF_ERROR_STATE_SIZE * EKF_MEAS_SIZE];
    arm_mat_init_f32(&H_trans, EKF_ERROR_STATE_SIZE, EKF_MEAS_SIZE, H_trans_data);
    arm_mat_trans_f32(&H, &H_trans);
    arm_mat_mult_f32(&temp5, &H_trans, &temp4);
    
    arm_mat_add_f32(&temp4, &R, &temp4);
    
    arm_matrix_instance_f32 temp4_inv;
    float32_t temp4_inv_data[EKF_MEAS_SIZE * EKF_MEAS_SIZE];
    arm_mat_init_f32(&temp4_inv, EKF_MEAS_SIZE, EKF_MEAS_SIZE, temp4_inv_data);
    arm_mat_inverse_f32(&temp4, &temp4_inv);
    
    arm_mat_mult_f32(&self->P, &H_trans, &temp3);
    arm_mat_mult_f32(&temp3, &temp4_inv, &K);
}

/**
 * @brief Apply state corrections
 */
static void apply_state_correction(EKF* self) {
    // Additive corrections
    self->state.pos_n += error_state_data[0];
    self->state.pos_e += error_state_data[1];
    self->state.pos_d += error_state_data[2];
    
    self->state.vel_n += error_state_data[3];
    self->state.vel_e += error_state_data[4];
    self->state.vel_d += error_state_data[5];
    
    self->state.b_gx += error_state_data[9];
    self->state.b_gy += error_state_data[10];
    self->state.b_gz += error_state_data[11];
    
    self->state.b_ax += error_state_data[12];
    self->state.b_ay += error_state_data[13];
    self->state.b_az += error_state_data[14];
    
    // Quaternion correction (multiplicative)
    float dq0 = 1.0f;
    float dq1 = 0.5f * error_state_data[6];
    float dq2 = 0.5f * error_state_data[7];
    float dq3 = 0.5f * error_state_data[8];
    
    float q0_new, q1_new, q2_new, q3_new;
    quat_multiply(dq0, dq1, dq2, dq3,
                  self->state.q0, self->state.q1, self->state.q2, self->state.q3,
                  &q0_new, &q1_new, &q2_new, &q3_new);
    
    self->state.q0 = q0_new;
    self->state.q1 = q1_new;
    self->state.q2 = q2_new;
    self->state.q3 = q3_new;
    
    quat_normalize(&self->state.q0, &self->state.q1, &self->state.q2, &self->state.q3);
}

/**
 * @brief Update error covariance (Joseph form)
 */
static void update_covariance(EKF* self) {
    arm_mat_mult_f32(&K, &H, &temp1);
    
    for (int i = 0; i < EKF_ERROR_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_ERROR_STATE_SIZE; j++) {
            if (i == j) {
                temp2_data[i*EKF_ERROR_STATE_SIZE + j] = 1.0f - temp1_data[i*EKF_ERROR_STATE_SIZE + j];
            } else {
                temp2_data[i*EKF_ERROR_STATE_SIZE + j] = -temp1_data[i*EKF_ERROR_STATE_SIZE + j];
            }
        }
    }
    
    arm_mat_mult_f32(&temp2, &self->P, &temp1);
    
    arm_matrix_instance_f32 temp2_trans;
    float32_t temp2_trans_data[EKF_ERROR_STATE_SIZE * EKF_ERROR_STATE_SIZE];
    arm_mat_init_f32(&temp2_trans, EKF_ERROR_STATE_SIZE, EKF_ERROR_STATE_SIZE, temp2_trans_data);
    arm_mat_trans_f32(&temp2, &temp2_trans);
    arm_mat_mult_f32(&temp1, &temp2_trans, &self->P);
    
    arm_mat_mult_f32(&K, &R, &temp3);
    
    arm_matrix_instance_f32 K_trans;
    float32_t K_trans_data[EKF_MEAS_SIZE * EKF_ERROR_STATE_SIZE];
    arm_mat_init_f32(&K_trans, EKF_MEAS_SIZE, EKF_ERROR_STATE_SIZE, K_trans_data);
    arm_mat_trans_f32(&K, &K_trans);
    arm_mat_mult_f32(&temp3, &K_trans, &temp1);
    
    arm_mat_add_f32(&self->P, &temp1, &self->P);
}

/**
 * @brief EKF update step
 */
ekf_error_code_t ekf_update(EKF* self, const gnss_data_t* gnss)
{
    if (!gnss->valid) {
        self->stats.gnss_rejected++;
        return EKF_INVALID_GNSS;
    }
    
    compute_H_matrix();
    compute_R_matrix(self);
    compute_innovation(self, gnss);
    compute_kalman_gain(self);
    
    arm_mat_mult_f32(&K, &innovation, &error_state);
    
    apply_state_correction(self);
    update_covariance(self);
    
    self->stats.update_count++;
    self->last_update_time = HAL_GetTick();
    
    return EKF_SUCCESS;
}

/* ========================================================================== */
/* ACCESSOR METHODS (FOLLOWING SD PATTERN) */
/* ========================================================================== */

ekf_error_code_t ekf_get_state(EKF* self, nav_state_t* state)
{
    *state = self->state;
    return EKF_SUCCESS;
}

ekf_error_code_t ekf_get_velocity_buffer(EKF* self, float** vel_n, float** vel_e, 
                                         float** vel_d, uint32_t* size, uint8_t* full)
{
    *vel_n = self->vel_buffer.vel_n;
    *vel_e = self->vel_buffer.vel_e;
    *vel_d = self->vel_buffer.vel_d;
    *size = self->vel_buffer.index;
    *full = self->vel_buffer.full;
    return EKF_SUCCESS;
}

ekf_error_code_t ekf_set_process_noise(EKF* self, const process_noise_params_t* params)
{
    self->Q_params = *params;
    return EKF_SUCCESS;
}

ekf_error_code_t ekf_set_meas_noise(EKF* self, const meas_noise_params_t* params)
{
    self->R_params = *params;
    return EKF_SUCCESS;
}

ekf_error_code_t ekf_get_uncertainties(EKF* self, float* sigma_pos, 
                                       float* sigma_vel, float* sigma_att)
{
    *sigma_pos = sqrtf(self->P_data[0*EKF_ERROR_STATE_SIZE + 0]);
    *sigma_vel = sqrtf(self->P_data[3*EKF_ERROR_STATE_SIZE + 3]);
    *sigma_att = sqrtf(self->P_data[6*EKF_ERROR_STATE_SIZE + 6]);
    return EKF_SUCCESS;
}

void ekf_print_stats(EKF* self)
{
    printf("\n=== EKF Statistics ===\n");
    printf("Predictions:    %lu\n", self->stats.predict_count);
    printf("Updates:        %lu\n", self->stats.update_count);
    printf("IMU errors:     %lu\n", self->stats.imu_errors);
    printf("GNSS rejected:  %lu\n", self->stats.gnss_rejected);
    printf("======================\n\n");
}
