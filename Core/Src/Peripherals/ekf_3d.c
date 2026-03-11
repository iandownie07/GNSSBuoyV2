/**
 * @file buoy_ekf_3d.c
 * @brief 3D Extended Kalman Filter for INS/GNSS Fusion on Buoy
 * 
 * State vector (16 elements nominal):
 *   Position NED: [n, e, d]           (3)
 *   Velocity NED: [v_n, v_e, v_d]     (3)
 *   Quaternion:   [q0, q1, q2, q3]    (4)
 *   Gyro bias:    [b_gx, b_gy, b_gz]  (3)
 *   Accel bias:   [b_ax, b_ay, b_az]  (3)
 * 
 * Error state vector (15 elements):
 *   Position error: [δr_n, δr_e, δr_d]         (3)
 *   Velocity error: [δv_n, δv_e, δv_d]         (3)
 *   Attitude error: [δφ_x, δφ_y, δφ_z]         (3)
 *   Gyro bias error: [δb_gx, δb_gy, δb_gz]     (3)
 *   Accel bias error: [δb_ax, δb_ay, δb_az]    (3)
 * 
 * Measurements from GNSS (6 elements):
 *   Position: [n, e, d]
 *   Velocity: [v_n, v_e, v_d]
 */

#include <math.h>
#include <string.h>
#include "arm_math.h"

/* ========================================================================== */
/* CONSTANTS */
/* ========================================================================== */

#define GRAVITY 9.81f  // m/s^2

#define STATE_SIZE 16      // Nominal state dimension
#define ERROR_STATE_SIZE 15 // Error state dimension
#define MEAS_SIZE 6        // Measurement dimension (GNSS pos + vel)

#define VELOCITY_BUFFER_SIZE 8192  // For spectral analysis

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
 * @brief EKF state and covariance
 */
typedef struct {
    nav_state_t x;                                    // Nominal state
    float32_t P_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];  // Covariance matrix (15x15)
    arm_matrix_instance_f32 P;                        // CMSIS matrix wrapper
} ekf_state_t;

/**
 * @brief Velocity data buffer for spectral analysis
 */
typedef struct {
    float vel_n[VELOCITY_BUFFER_SIZE];
    float vel_e[VELOCITY_BUFFER_SIZE];
    float vel_d[VELOCITY_BUFFER_SIZE];
    uint32_t index;
    uint8_t full;
} velocity_buffer_t;

/* ========================================================================== */
/* TUNING PARAMETERS */
/* ========================================================================== */

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

/* Default tuning parameters for buoy application */
static const process_noise_params_t default_process_noise = {
    .sigma_pos = 0.01f,      // 1 cm position process noise
    .sigma_vel = 0.1f,       // 0.1 m/s velocity process noise
    .sigma_att = 0.01f,      // ~0.6 deg attitude process noise
    .sigma_bg = 1e-4f,       // Gyro bias random walk
    .sigma_ba = 1e-3f        // Accel bias random walk
};

static const meas_noise_params_t default_meas_noise = {
    .sigma_pos_n = 2.0f,     // 2 m horizontal position accuracy
    .sigma_pos_e = 2.0f,
    .sigma_pos_d = 3.0f,     // 3 m vertical position accuracy
    .sigma_vel_n = 0.05f,    // 0.05 m/s horizontal velocity accuracy
    .sigma_vel_e = 0.05f,
    .sigma_vel_d = 0.1f      // 0.1 m/s vertical velocity accuracy
};

/* ========================================================================== */
/* GLOBAL VARIABLES (STATIC ALLOCATION) */
/* ========================================================================== */

static ekf_state_t ekf;
static velocity_buffer_t vel_buffer;
static process_noise_params_t Q_params;
static meas_noise_params_t R_params;

/* Working matrices for EKF computations */
static float32_t F_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
static float32_t Phi_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
static float32_t Q_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
static float32_t H_data[MEAS_SIZE * ERROR_STATE_SIZE];
static float32_t R_data[MEAS_SIZE * MEAS_SIZE];
static float32_t K_data[ERROR_STATE_SIZE * MEAS_SIZE];

static float32_t temp1_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
static float32_t temp2_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
static float32_t temp3_data[ERROR_STATE_SIZE * MEAS_SIZE];
static float32_t temp4_data[MEAS_SIZE * MEAS_SIZE];
static float32_t temp5_data[MEAS_SIZE * ERROR_STATE_SIZE];

static float32_t innovation_data[MEAS_SIZE];
static float32_t error_state_data[ERROR_STATE_SIZE];

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
 * 
 * C_b^l = R(q) is the 3x3 rotation matrix from body frame to local NED frame
 * Stored in row-major order
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
    
    // Row 0
    C[0] = q0q0 + q1q1 - q2q2 - q3q3;
    C[1] = 2.0f * (q1q2 + q0q3);
    C[2] = 2.0f * (q1q3 - q0q2);
    
    // Row 1
    C[3] = 2.0f * (q1q2 - q0q3);
    C[4] = q0q0 - q1q1 + q2q2 - q3q3;
    C[5] = 2.0f * (q2q3 + q0q1);
    
    // Row 2
    C[6] = 2.0f * (q1q3 + q0q2);
    C[7] = 2.0f * (q2q3 - q0q1);
    C[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**
 * @brief Create skew-symmetric matrix from 3-vector
 * 
 * S(v) = [  0   -v3   v2 ]
 *        [ v3    0   -v1 ]
 *        [-v2   v1    0  ]
 * 
 * Stored in row-major order
 */
static void skew_symmetric(float v1, float v2, float v3, float *S) {
    S[0] =   0.0f;  S[1] = -v3;     S[2] =  v2;
    S[3] =   v3;    S[4] =  0.0f;   S[5] = -v1;
    S[6] =  -v2;    S[7] =  v1;     S[8] =  0.0f;
}

/* ========================================================================== */
/* EKF INITIALIZATION */
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
              float roll, float pitch, float yaw) {
    
    /* Initialize nominal state */
    ekf.x.pos_n = pos_n;
    ekf.x.pos_e = pos_e;
    ekf.x.pos_d = pos_d;
    
    ekf.x.vel_n = vel_n;
    ekf.x.vel_e = vel_e;
    ekf.x.vel_d = vel_d;
    
    /* Convert Euler angles to quaternion (ZYX convention) */
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    ekf.x.q0 = cr * cp * cy + sr * sp * sy;
    ekf.x.q1 = sr * cp * cy - cr * sp * sy;
    ekf.x.q2 = cr * sp * cy + sr * cp * sy;
    ekf.x.q3 = cr * cp * sy - sr * sp * cy;
    
    quat_normalize(&ekf.x.q0, &ekf.x.q1, &ekf.x.q2, &ekf.x.q3);
    
    /* Initialize biases to zero */
    ekf.x.b_gx = 0.0f;
    ekf.x.b_gy = 0.0f;
    ekf.x.b_gz = 0.0f;
    
    ekf.x.b_ax = 0.0f;
    ekf.x.b_ay = 0.0f;
    ekf.x.b_az = 0.0f;
    
    /* Initialize covariance matrix P */
    memset(ekf.P_data, 0, sizeof(ekf.P_data));
    
    /* Set initial uncertainties (diagonal elements) */
    float sigma_pos_init = 10.0f;      // 10 m initial position uncertainty
    float sigma_vel_init = 1.0f;       // 1 m/s initial velocity uncertainty
    float sigma_att_init = 0.1f;       // ~5.7 deg initial attitude uncertainty
    float sigma_bg_init = 0.01f;       // Initial gyro bias uncertainty
    float sigma_ba_init = 0.1f;        // Initial accel bias uncertainty
    
    for (int i = 0; i < 3; i++) {
        ekf.P_data[i * ERROR_STATE_SIZE + i] = sigma_pos_init * sigma_pos_init;           // Position
        ekf.P_data[(i+3) * ERROR_STATE_SIZE + (i+3)] = sigma_vel_init * sigma_vel_init;   // Velocity
        ekf.P_data[(i+6) * ERROR_STATE_SIZE + (i+6)] = sigma_att_init * sigma_att_init;   // Attitude
        ekf.P_data[(i+9) * ERROR_STATE_SIZE + (i+9)] = sigma_bg_init * sigma_bg_init;     // Gyro bias
        ekf.P_data[(i+12) * ERROR_STATE_SIZE + (i+12)] = sigma_ba_init * sigma_ba_init;   // Accel bias
    }
    
    /* Initialize CMSIS matrix wrapper */
    arm_mat_init_f32(&ekf.P, ERROR_STATE_SIZE, ERROR_STATE_SIZE, ekf.P_data);
    
    /* Initialize velocity buffer */
    memset(&vel_buffer, 0, sizeof(vel_buffer));
    
    /* Initialize tuning parameters */
    Q_params = default_process_noise;
    R_params = default_meas_noise;
    
    /* Initialize working matrix instances */
    arm_mat_init_f32(&F, ERROR_STATE_SIZE, ERROR_STATE_SIZE, F_data);
    arm_mat_init_f32(&Phi, ERROR_STATE_SIZE, ERROR_STATE_SIZE, Phi_data);
    arm_mat_init_f32(&Q, ERROR_STATE_SIZE, ERROR_STATE_SIZE, Q_data);
    arm_mat_init_f32(&H, MEAS_SIZE, ERROR_STATE_SIZE, H_data);
    arm_mat_init_f32(&R, MEAS_SIZE, MEAS_SIZE, R_data);
    arm_mat_init_f32(&K, ERROR_STATE_SIZE, MEAS_SIZE, K_data);
    
    arm_mat_init_f32(&temp1, ERROR_STATE_SIZE, ERROR_STATE_SIZE, temp1_data);
    arm_mat_init_f32(&temp2, ERROR_STATE_SIZE, ERROR_STATE_SIZE, temp2_data);
    arm_mat_init_f32(&temp3, ERROR_STATE_SIZE, MEAS_SIZE, temp3_data);
    arm_mat_init_f32(&temp4, MEAS_SIZE, MEAS_SIZE, temp4_data);
    arm_mat_init_f32(&temp5, MEAS_SIZE, ERROR_STATE_SIZE, temp5_data);
    
    arm_mat_init_f32(&innovation, MEAS_SIZE, 1, innovation_data);
    arm_mat_init_f32(&error_state, ERROR_STATE_SIZE, 1, error_state_data);
}

/* ========================================================================== */
/* EKF PREDICTION STEP */
/* ========================================================================== */

/**
 * @brief Propagate nominal state using IMU measurements
 * 
 * This implements the nonlinear INS equations:
 * - Quaternion kinematics
 * - Velocity kinematics
 * - Position kinematics
 */
static void propagate_nominal_state(const imu_data_t *imu) {
    float dt = imu->dt;
    
    /* Compensate for biases */
    float omega_x = imu->gyro_x - ekf.x.b_gx;
    float omega_y = imu->gyro_y - ekf.x.b_gy;
    float omega_z = imu->gyro_z - ekf.x.b_gz;
    
    float accel_x = imu->accel_x - ekf.x.b_ax;
    float accel_y = imu->accel_y - ekf.x.b_ay;
    float accel_z = imu->accel_z - ekf.x.b_az;
    
    /* ======================================================================
     * STEP 1: Update quaternion
     * ====================================================================== */
    
    /* Quaternion derivative: q_dot = 0.5 * Omega(omega) * q */
    /* Where Omega is the 4x4 matrix form of quaternion product */
    
    float q0 = ekf.x.q0;
    float q1 = ekf.x.q1;
    float q2 = ekf.x.q2;
    float q3 = ekf.x.q3;
    
    /* Simple first-order integration */
    float q0_dot = 0.5f * (-q1*omega_x - q2*omega_y - q3*omega_z);
    float q1_dot = 0.5f * ( q0*omega_x + q2*omega_z - q3*omega_y);
    float q2_dot = 0.5f * ( q0*omega_y - q1*omega_z + q3*omega_x);
    float q3_dot = 0.5f * ( q0*omega_z + q1*omega_y - q2*omega_x);
    
    ekf.x.q0 += q0_dot * dt;
    ekf.x.q1 += q1_dot * dt;
    ekf.x.q2 += q2_dot * dt;
    ekf.x.q3 += q3_dot * dt;
    
    /* Normalize quaternion */
    quat_normalize(&ekf.x.q0, &ekf.x.q1, &ekf.x.q2, &ekf.x.q3);
    
    /* ======================================================================
     * STEP 2: Transform acceleration to NED frame
     * ====================================================================== */
    
    float C[9];
    quat_to_rotation_matrix(ekf.x.q0, ekf.x.q1, ekf.x.q2, ekf.x.q3, C);
    
    /* Rotate body-frame acceleration to NED frame */
    float accel_n = C[0]*accel_x + C[1]*accel_y + C[2]*accel_z;
    float accel_e = C[3]*accel_x + C[4]*accel_y + C[5]*accel_z;
    float accel_d = C[6]*accel_x + C[7]*accel_y + C[8]*accel_z;
    
    /* Subtract gravity (pointing down in NED frame) */
    accel_d -= GRAVITY;
    
    /* ======================================================================
     * STEP 3: Update velocity
     * ====================================================================== */
    
    ekf.x.vel_n += accel_n * dt;
    ekf.x.vel_e += accel_e * dt;
    ekf.x.vel_d += accel_d * dt;
    
    /* ======================================================================
     * STEP 4: Update position
     * ====================================================================== */
    
    ekf.x.pos_n += ekf.x.vel_n * dt;
    ekf.x.pos_e += ekf.x.vel_e * dt;
    ekf.x.pos_d += ekf.x.vel_d * dt;
    
    /* Biases remain constant (random walk model) */
}

/**
 * @brief Compute error-state transition matrix F
 * 
 * F is the 15x15 Jacobian of the error dynamics
 * 
 * Structure (simplified local-level frame, no Coriolis/transport):
 * 
 *     [ 0₃    I₃    0₃    0₃    0₃   ]
 *     [ 0₃    0₃   S(f)   0₃   -C_b^l ]
 * F = [ 0₃    0₃    0₃   -C_b^l  0₃   ]
 *     [ 0₃    0₃    0₃    0₃    0₃   ]
 *     [ 0₃    0₃    0₃    0₃    0₃   ]
 * 
 * Where:
 *   S(f) = skew-symmetric matrix of specific force in NED frame
 *   C_b^l = rotation matrix from body to local (NED) frame
 */
static void compute_F_matrix(const imu_data_t *imu) {
    /* Clear F matrix */
    memset(F_data, 0, sizeof(F_data));
    
    /* Get rotation matrix from quaternion */
    float C[9];
    quat_to_rotation_matrix(ekf.x.q0, ekf.x.q1, ekf.x.q2, ekf.x.q3, C);
    
    /* Compensated acceleration in body frame */
    float accel_x = imu->accel_x - ekf.x.b_ax;
    float accel_y = imu->accel_y - ekf.x.b_ay;
    float accel_z = imu->accel_z - ekf.x.b_az;
    
    /* Transform to NED frame */
    float f_n = C[0]*accel_x + C[1]*accel_y + C[2]*accel_z;
    float f_e = C[3]*accel_x + C[4]*accel_y + C[5]*accel_z;
    float f_d = C[6]*accel_x + C[7]*accel_y + C[8]*accel_z - GRAVITY;
    
    /* Skew-symmetric matrix of specific force */
    float S_f[9];
    skew_symmetric(f_n, f_e, f_d, S_f);
    
    /* ======================================================================
     * Fill F matrix (row-major order, 15x15)
     * ====================================================================== */
    
    /* Block (0,1): Position error from velocity error = I₃ */
    F_data[0*15 + 3] = 1.0f;  // δṙ_n = δv_n
    F_data[1*15 + 4] = 1.0f;  // δṙ_e = δv_e
    F_data[2*15 + 5] = 1.0f;  // δṙ_d = δv_d
    
    /* Block (1,2): Velocity error from attitude error = S(f) */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(3+i)*15 + (6+j)] = S_f[i*3 + j];
        }
    }
    
    /* Block (1,4): Velocity error from accel bias error = -C_b^l */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(3+i)*15 + (12+j)] = -C[i*3 + j];
        }
    }
    
    /* Block (2,3): Attitude error from gyro bias error = -C_b^l */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[(6+i)*15 + (9+j)] = -C[i*3 + j];
        }
    }
    
    /* Blocks (3,*) and (4,*): Bias errors are zero (random walk) */
}

/**
 * @brief Discretize F to get state transition matrix Phi
 * 
 * For small dt: Phi ≈ I + F*dt
 */
static void compute_Phi_matrix(float dt) {
    /* Phi = I + F * dt */
    
    /* Start with identity */
    memset(Phi_data, 0, sizeof(Phi_data));
    for (int i = 0; i < ERROR_STATE_SIZE; i++) {
        Phi_data[i*ERROR_STATE_SIZE + i] = 1.0f;
    }
    
    /* Add F * dt */
    for (int i = 0; i < ERROR_STATE_SIZE; i++) {
        for (int j = 0; j < ERROR_STATE_SIZE; j++) {
            Phi_data[i*ERROR_STATE_SIZE + j] += F_data[i*ERROR_STATE_SIZE + j] * dt;
        }
    }
}

/**
 * @brief Compute process noise covariance matrix Q
 * 
 * Q = G * G^T * dt^2
 * 
 * Where G is diagonal with process noise standard deviations
 */
static void compute_Q_matrix(float dt) {
    /* Clear Q matrix */
    memset(Q_data, 0, sizeof(Q_data));
    
    float dt2 = dt * dt;
    
    /* Position process noise */
    for (int i = 0; i < 3; i++) {
        Q_data[i*ERROR_STATE_SIZE + i] = Q_params.sigma_pos * Q_params.sigma_pos * dt2;
    }
    
    /* Velocity process noise */
    for (int i = 0; i < 3; i++) {
        Q_data[(3+i)*ERROR_STATE_SIZE + (3+i)] = Q_params.sigma_vel * Q_params.sigma_vel * dt2;
    }
    
    /* Attitude process noise */
    for (int i = 0; i < 3; i++) {
        Q_data[(6+i)*ERROR_STATE_SIZE + (6+i)] = Q_params.sigma_att * Q_params.sigma_att * dt2;
    }
    
    /* Gyro bias process noise */
    for (int i = 0; i < 3; i++) {
        Q_data[(9+i)*ERROR_STATE_SIZE + (9+i)] = Q_params.sigma_bg * Q_params.sigma_bg * dt2;
    }
    
    /* Accel bias process noise */
    for (int i = 0; i < 3; i++) {
        Q_data[(12+i)*ERROR_STATE_SIZE + (12+i)] = Q_params.sigma_ba * Q_params.sigma_ba * dt2;
    }
}

/**
 * @brief Propagate error covariance
 * 
 * P⁻ = Phi * P⁺ * Phi^T + Q
 */
static void propagate_covariance(void) {
    /* temp1 = Phi * P */
    arm_mat_mult_f32(&Phi, &ekf.P, &temp1);
    
    /* temp2 = temp1 * Phi^T = Phi * P * Phi^T */
    arm_mat_trans_f32(&Phi, &temp2);  // temp2 = Phi^T temporarily
    arm_mat_mult_f32(&temp1, &temp2, &ekf.P);  // P = Phi * P * Phi^T (reuse P as output)
    
    /* P = P + Q */
    arm_mat_add_f32(&ekf.P, &Q, &ekf.P);
}

/**
 * @brief EKF prediction step
 * 
 * This is called at IMU rate (100-200 Hz)
 */
void ekf_predict(const imu_data_t *imu) {
    /* Step 1: Propagate nominal state */
    propagate_nominal_state(imu);
    
    /* Step 2: Compute F matrix */
    compute_F_matrix(imu);
    
    /* Step 3: Discretize to get Phi */
    compute_Phi_matrix(imu->dt);
    
    /* Step 4: Compute Q matrix */
    compute_Q_matrix(imu->dt);
    
    /* Step 5: Propagate covariance */
    propagate_covariance();
    
    /* Step 6: Store velocity in buffer for spectral analysis */
    vel_buffer.vel_n[vel_buffer.index] = ekf.x.vel_n;
    vel_buffer.vel_e[vel_buffer.index] = ekf.x.vel_e;
    vel_buffer.vel_d[vel_buffer.index] = ekf.x.vel_d;
    
    vel_buffer.index++;
    if (vel_buffer.index >= VELOCITY_BUFFER_SIZE) {
        vel_buffer.index = 0;
        vel_buffer.full = 1;
    }
}

/* ========================================================================== */
/* EKF UPDATE STEP */
/* ========================================================================== */

/**
 * @brief Compute measurement matrix H
 * 
 * H is 6x15 for GNSS position and velocity measurements
 * 
 * H = [ I₆ | 0₆ₓ₉ ]
 * 
 * This directly observes position and velocity, but not attitude or biases
 */
static void compute_H_matrix(void) {
    /* Clear H matrix */
    memset(H_data, 0, sizeof(H_data));
    
    /* H is identity for first 6 states (position and velocity) */
    for (int i = 0; i < MEAS_SIZE; i++) {
        H_data[i*ERROR_STATE_SIZE + i] = 1.0f;
    }
}

/**
 * @brief Compute measurement noise covariance matrix R
 */
static void compute_R_matrix(void) {
    /* Clear R matrix */
    memset(R_data, 0, sizeof(R_data));
    
    /* Diagonal with measurement noise variances */
    R_data[0*MEAS_SIZE + 0] = R_params.sigma_pos_n * R_params.sigma_pos_n;
    R_data[1*MEAS_SIZE + 1] = R_params.sigma_pos_e * R_params.sigma_pos_e;
    R_data[2*MEAS_SIZE + 2] = R_params.sigma_pos_d * R_params.sigma_pos_d;
    R_data[3*MEAS_SIZE + 3] = R_params.sigma_vel_n * R_params.sigma_vel_n;
    R_data[4*MEAS_SIZE + 4] = R_params.sigma_vel_e * R_params.sigma_vel_e;
    R_data[5*MEAS_SIZE + 5] = R_params.sigma_vel_d * R_params.sigma_vel_d;
}

/**
 * @brief Compute innovation (measurement residual)
 * 
 * ν = y - h(x⁻)
 */
static void compute_innovation(const gnss_data_t *gnss) {
    innovation_data[0] = gnss->pos_n - ekf.x.pos_n;
    innovation_data[1] = gnss->pos_e - ekf.x.pos_e;
    innovation_data[2] = gnss->pos_d - ekf.x.pos_d;
    innovation_data[3] = gnss->vel_n - ekf.x.vel_n;
    innovation_data[4] = gnss->vel_e - ekf.x.vel_e;
    innovation_data[5] = gnss->vel_d - ekf.x.vel_d;
}

/**
 * @brief Compute Kalman gain
 * 
 * K = P⁻ * H^T * (H * P⁻ * H^T + R)⁻¹
 */
static void compute_kalman_gain(void) {
    /* temp5 = H * P (6x15) */
    arm_mat_mult_f32(&H, &ekf.P, &temp5);
    
    /* temp4 = temp5 * H^T = H * P * H^T (6x6) */
    arm_mat_trans_f32(&H, &temp3);  // temp3 = H^T temporarily (15x6)
    arm_mat_mult_f32(&temp5, &temp3, &temp4);
    
    /* temp4 = temp4 + R = H * P * H^T + R (6x6) */
    arm_mat_add_f32(&temp4, &R, &temp4);
    
    /* Invert temp4 */
    arm_matrix_instance_f32 temp4_inv;
    float32_t temp4_inv_data[MEAS_SIZE * MEAS_SIZE];
    arm_mat_init_f32(&temp4_inv, MEAS_SIZE, MEAS_SIZE, temp4_inv_data);
    arm_mat_inverse_f32(&temp4, &temp4_inv);
    
    /* temp3 = P * H^T (15x6) - recompute since we used temp3 above */
    arm_matrix_instance_f32 H_trans;
    float32_t H_trans_data[ERROR_STATE_SIZE * MEAS_SIZE];
    arm_mat_init_f32(&H_trans, ERROR_STATE_SIZE, MEAS_SIZE, H_trans_data);
    arm_mat_trans_f32(&H, &H_trans);
    arm_mat_mult_f32(&ekf.P, &H_trans, &temp3);
    
    /* K = temp3 * temp4_inv = P * H^T * (H*P*H^T + R)⁻¹ (15x6) */
    arm_mat_mult_f32(&temp3, &temp4_inv, &K);
}

/**
 * @brief Apply state corrections
 * 
 * Position, velocity, and bias corrections are additive
 * Attitude correction is multiplicative via quaternion
 */
static void apply_state_correction(void) {
    /* Extract error state corrections */
    float delta_pos_n = error_state_data[0];
    float delta_pos_e = error_state_data[1];
    float delta_pos_d = error_state_data[2];
    
    float delta_vel_n = error_state_data[3];
    float delta_vel_e = error_state_data[4];
    float delta_vel_d = error_state_data[5];
    
    float delta_phi_x = error_state_data[6];
    float delta_phi_y = error_state_data[7];
    float delta_phi_z = error_state_data[8];
    
    float delta_b_gx = error_state_data[9];
    float delta_b_gy = error_state_data[10];
    float delta_b_gz = error_state_data[11];
    
    float delta_b_ax = error_state_data[12];
    float delta_b_ay = error_state_data[13];
    float delta_b_az = error_state_data[14];
    
    /* Apply additive corrections */
    ekf.x.pos_n += delta_pos_n;
    ekf.x.pos_e += delta_pos_e;
    ekf.x.pos_d += delta_pos_d;
    
    ekf.x.vel_n += delta_vel_n;
    ekf.x.vel_e += delta_vel_e;
    ekf.x.vel_d += delta_vel_d;
    
    ekf.x.b_gx += delta_b_gx;
    ekf.x.b_gy += delta_b_gy;
    ekf.x.b_gz += delta_b_gz;
    
    ekf.x.b_ax += delta_b_ax;
    ekf.x.b_ay += delta_b_ay;
    ekf.x.b_az += delta_b_az;
    
    /* Apply quaternion correction (multiplicative) */
    /* Convert rotation vector to quaternion: δq = [1, 0.5*δφ] */
    float dq0 = 1.0f;
    float dq1 = 0.5f * delta_phi_x;
    float dq2 = 0.5f * delta_phi_y;
    float dq3 = 0.5f * delta_phi_z;
    
    /* q⁺ = δq ⊗ q⁻ */
    float q0_new, q1_new, q2_new, q3_new;
    quat_multiply(dq0, dq1, dq2, dq3,
                  ekf.x.q0, ekf.x.q1, ekf.x.q2, ekf.x.q3,
                  &q0_new, &q1_new, &q2_new, &q3_new);
    
    ekf.x.q0 = q0_new;
    ekf.x.q1 = q1_new;
    ekf.x.q2 = q2_new;
    ekf.x.q3 = q3_new;
    
    /* Normalize quaternion */
    quat_normalize(&ekf.x.q0, &ekf.x.q1, &ekf.x.q2, &ekf.x.q3);
}

/**
 * @brief Update error covariance (Joseph form for numerical stability)
 * 
 * P⁺ = (I - K*H) * P⁻ * (I - K*H)^T + K * R * K^T
 */
static void update_covariance(void) {
    /* temp1 = K * H (15x15) */
    arm_mat_mult_f32(&K, &H, &temp1);
    
    /* temp2 = I - K*H (15x15) */
    for (int i = 0; i < ERROR_STATE_SIZE; i++) {
        for (int j = 0; j < ERROR_STATE_SIZE; j++) {
            if (i == j) {
                temp2_data[i*ERROR_STATE_SIZE + j] = 1.0f - temp1_data[i*ERROR_STATE_SIZE + j];
            } else {
                temp2_data[i*ERROR_STATE_SIZE + j] = -temp1_data[i*ERROR_STATE_SIZE + j];
            }
        }
    }
    
    /* temp1 = temp2 * P = (I - K*H) * P (15x15) */
    arm_mat_mult_f32(&temp2, &ekf.P, &temp1);
    
    /* ekf.P = temp1 * temp2^T = (I - K*H) * P * (I - K*H)^T (15x15) */
    arm_matrix_instance_f32 temp2_trans;
    float32_t temp2_trans_data[ERROR_STATE_SIZE * ERROR_STATE_SIZE];
    arm_mat_init_f32(&temp2_trans, ERROR_STATE_SIZE, ERROR_STATE_SIZE, temp2_trans_data);
    arm_mat_trans_f32(&temp2, &temp2_trans);
    arm_mat_mult_f32(&temp1, &temp2_trans, &ekf.P);
    
    /* Add K * R * K^T term */
    /* temp3 = K * R (15x6) */
    arm_mat_mult_f32(&K, &R, &temp3);
    
    /* temp1 = temp3 * K^T = K * R * K^T (15x15) */
    arm_matrix_instance_f32 K_trans;
    float32_t K_trans_data[MEAS_SIZE * ERROR_STATE_SIZE];
    arm_mat_init_f32(&K_trans, MEAS_SIZE, ERROR_STATE_SIZE, K_trans_data);
    arm_mat_trans_f32(&K, &K_trans);
    arm_mat_mult_f32(&temp3, &K_trans, &temp1);
    
    /* P = P + K*R*K^T */
    arm_mat_add_f32(&ekf.P, &temp1, &ekf.P);
}

/**
 * @brief EKF update step
 * 
 * This is called when GNSS measurements are available (1-5 Hz)
 */
void ekf_update(const gnss_data_t *gnss) {
    if (!gnss->valid) {
        return;  // Skip update if GNSS invalid
    }
    
    /* Step 1: Compute H matrix */
    compute_H_matrix();
    
    /* Step 2: Compute R matrix */
    compute_R_matrix();
    
    /* Step 3: Compute innovation */
    compute_innovation(gnss);
    
    /* Step 4: Compute Kalman gain */
    compute_kalman_gain();
    
    /* Step 5: Compute error state correction: δx = K * ν */
    arm_mat_mult_f32(&K, &innovation, &error_state);
    
    /* Step 6: Apply corrections to nominal state */
    apply_state_correction();
    
    /* Step 7: Update covariance */
    update_covariance();
}

/* ========================================================================== */
/* ACCESSOR FUNCTIONS */
/* ========================================================================== */

/**
 * @brief Get current state estimate
 */
void ekf_get_state(nav_state_t *state) {
    *state = ekf.x;
}

/**
 * @brief Get velocity buffer for spectral analysis
 */
void ekf_get_velocity_buffer(float **vel_n, float **vel_e, float **vel_d, 
                             uint32_t *size, uint8_t *full) {
    *vel_n = vel_buffer.vel_n;
    *vel_e = vel_buffer.vel_e;
    *vel_d = vel_buffer.vel_d;
    *size = vel_buffer.index;
    *full = vel_buffer.full;
}

/**
 * @brief Set process noise parameters
 */
void ekf_set_process_noise(const process_noise_params_t *params) {
    Q_params = *params;
}

/**
 * @brief Set measurement noise parameters
 */
void ekf_set_meas_noise(const meas_noise_params_t *params) {
    R_params = *params;
}

/**
 * @brief Get position uncertainty (standard deviation)
 */
void ekf_get_position_uncertainty(float *sigma_n, float *sigma_e, float *sigma_d) {
    *sigma_n = sqrtf(ekf.P_data[0*ERROR_STATE_SIZE + 0]);
    *sigma_e = sqrtf(ekf.P_data[1*ERROR_STATE_SIZE + 1]);
    *sigma_d = sqrtf(ekf.P_data[2*ERROR_STATE_SIZE + 2]);
}

/**
 * @brief Get velocity uncertainty (standard deviation)
 */
void ekf_get_velocity_uncertainty(float *sigma_vn, float *sigma_ve, float *sigma_vd) {
    *sigma_vn = sqrtf(ekf.P_data[3*ERROR_STATE_SIZE + 3]);
    *sigma_ve = sqrtf(ekf.P_data[4*ERROR_STATE_SIZE + 4]);
    *sigma_vd = sqrtf(ekf.P_data[5*ERROR_STATE_SIZE + 5]);
}

/**
 * @brief Get attitude uncertainty (standard deviation in radians)
 */
void ekf_get_attitude_uncertainty(float *sigma_roll, float *sigma_pitch, float *sigma_yaw) {
    *sigma_roll = sqrtf(ekf.P_data[6*ERROR_STATE_SIZE + 6]);
    *sigma_pitch = sqrtf(ekf.P_data[7*ERROR_STATE_SIZE + 7]);
    *sigma_yaw = sqrtf(ekf.P_data[8*ERROR_STATE_SIZE + 8]);
}
