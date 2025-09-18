/**
 * @file device_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Device configuration interface 
 * 
 * @version 0.1
 * @date 2025-04-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _DEVICE_CONFIG_H_ 
#define _DEVICE_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "esc_driver.h" 
#include "lsm303agr_driver.h" 
#include "mpu6050_driver.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// ESC 
#define ESC_NO_THRUST 0   // Force thruster output to zero 

// M8Q 
#define M8Q_CONFIG_MSG_NUM 12        // Number of messages in a configuration packet 
#define M8Q_CONFIG_MSG_MAX_LEN 150   // Max length of a single config message in a packet 

//=======================================================================================


//=======================================================================================
// ESC 

// PWM timer period (auto-reload register) 
extern const uint16_t esc_period; 

// PWM limits (us) 
extern const uint16_t esc_fwd_speed_lim; 
extern const uint16_t esc_rev_speed_lim; 

//=======================================================================================


//=======================================================================================
// LSM303AGR

// Hard and soft-iron driver calibration values 
extern const float lsm303agr_hi_offset[NUM_AXES];         // Hard-iron offsets 
extern const float lsm303agr_sid_values[NUM_AXES];        // Soft-iron diagonal values 
extern const float lsm303agr_sio_values[NUM_AXES];        // Soft-iron off-diagonal values 

//=======================================================================================


//=======================================================================================
// M8Q 

// Message configuration 
extern const char m8q_config_msgs[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN];

// Device data 
extern const float gps_vel_variance[NUM_AXES];   // GPS velocity measurement variance 

//=======================================================================================


//=======================================================================================
// MPU6050 

extern const uint8_t mpu6050_standby_mask;          // Axis standby status mask 
extern const uint8_t mpu6050_sample_rate_divider;   // Sample Rate Divider 

// When setting accelerometer offsets, be mindful not to cancel out gravity (unless that 
// is desired). The gravity axis offset (typically z when stationary and level) will be 
// the raw reading minus the scalar listed in the description of mpu6050_fs_sel_set_t 
// depending on the full scale range set. 
extern const int16_t mpu6050_accel_offsets[NUM_AXES];   // Accelerometer axis offsets 
extern const int16_t mpu6050_gyro_offsets[NUM_AXES];    // Gyroscope axis offsets 

//=======================================================================================


//=======================================================================================
// WS2812 

//==================================================
// LED colours 

// LED off 
extern const uint32_t ws2812_led_off; 

extern const uint32_t ws2812_led_standby_not_ready; 
extern const uint32_t ws2812_led_standby_ready; 

// Autonomous state 
extern const uint32_t ws2812_led_auto_star;       // Starbird side indicator 
extern const uint32_t ws2812_led_auto_port;       // Port side indicator 
extern const uint32_t ws2812_led_auto_strobe;     // Strobe light 

// Manual control state 
extern const uint32_t ws2812_led_manual_strobe; 

// Low power state 
extern const uint32_t ws2812_led_low_pwr; 

// Not ready state 
extern const uint32_t ws2812_led_not_ready; 

// Ready state 
extern const uint32_t ws2812_led_ready; 

//==================================================

//==================================================
// Controller settings 

// LEDs used as strobe lights - 1-bit per LED (8 total) 
extern const uint8_t ws2812_strobe_leds; 
// LED update software timer x this gives strobe period (s) 
extern const uint8_t ws2812_strobe_period; 

//==================================================

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _DEVICE_CONFIG_H_ 
