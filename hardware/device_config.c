/**
 * @file device_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Device configuration 
 * 
 * @version 0.1
 * @date 2025-04-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "device_config.h" 

//=======================================================================================


//=======================================================================================
// ESC 

// PWM timer period (auto-reload register) 
const uint16_t esc_period = 20000; 

// PWM limits (us) 
const uint16_t esc_fwd_speed_lim = 2000; 
const uint16_t esc_rev_speed_lim = 1000; 

//=======================================================================================


//=======================================================================================
// LSM303AGR 

const float lsm303agr_hi_offset[NUM_AXES] = 
{
    39.8f,     // X 
    -84.3f,    // Y 
    -354.3f    // Z 
};

const float lsm303agr_sid_values[NUM_AXES] = 
{
    0.963f,   // X 
    1.022f,   // Y 
    1.020f    // Z 
};

const float lsm303agr_sio_values[NUM_AXES] = 
{
    0.019f,    // X 
    -0.003f,   // Y 
    -0.016f    // Z 
};

//=======================================================================================


//=======================================================================================
// M8Q 

// M8Q configuration messages 
const char m8q_config_msgs[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN] = 
{
    // Disable default NMEA messages 
    "$PUBX,40,GGA,0,0,0,0,0,0*",    // GGA disable
    "$PUBX,40,GLL,0,0,0,0,0,0*",    // GLL disable
    "$PUBX,40,GSA,0,0,0,0,0,0*",    // GSA disable
    "$PUBX,40,GSV,0,0,0,0,0,0*",    // GSV disable
    "$PUBX,40,RMC,0,0,0,0,0,0*",    // RMC disable
    "$PUBX,40,VTG,0,0,0,0,0,0*",    // VTG disable 

    // UBX config messages 
    "B562,06,01,0800,F1,00,01,00,00,00,00,00*",      // POSITION enable 
    "B562,06,01,0800,F1,04,0A,00,00,00,00,00*",      // TIME enable 

    // Power configuration 
    "B562,06,3B,3000,02,00,00,00,60104201,E8030000,10270000,00000000,"
    "0000,0000,0000000000000000000000000000000000000000,00000000*",

    // Port configuration 
    "B562,06,00,1400,01,00,0000,C0080000,80250000,0000,0000,0000,0000*",
    "B562,06,00,1400,00,00,9902,84000000,00000000,0700,0300,0200,0000*", 

    // Save the settings (save mask) 
    "B562,06,09,0C00,00000000,FFFFFFFF,00000000*" 
};

// GPS velocity measurement variance 
const float gps_vel_variance[NUM_AXES] = 
{
    1.00f,   // Speed over ground (SOG) 
    0.30f,   // Course over ground (COG) 
    1.00f    // Vertical velocity (vVel) 
};

//=======================================================================================


//=======================================================================================
// MPU6050 

const uint8_t mpu6050_standby_mask = 0x00;
const uint8_t mpu6050_sample_rate_divider = 0;

const int16_t mpu6050_accel_offsets[NUM_AXES] = 
{
    -204,   // X 
    -86,    // Y 
    52      // Z 
};

const int16_t mpu6050_gyro_offsets[NUM_AXES] = 
{
    537,    // X 
    -197,   // Y 
    21      // Z 
};

//=======================================================================================


//=======================================================================================
// WS2812 

//==================================================
// LED colours 

// LED off 
const uint32_t ws2812_led_off = 0x00000000; 

// Standby state 
const uint32_t ws2812_led_standby_not_ready = 0x0044EE00;   // Ready indicator (orange) 
const uint32_t ws2812_led_standby_ready = 0x0044EE00;       // Not ready indicator (orange) 

// Autonomous state 
const uint32_t ws2812_led_auto_star = 0x001E0000;           // Starbird side indicator (green) 
const uint32_t ws2812_led_auto_port = 0x00001E00;           // Port side indicator (red) 
const uint32_t ws2812_led_auto_strobe = 0x005E5E5E;         // Strobe light (white) 

// Manual state 
const uint32_t ws2812_led_manual_strobe = 0x000038FF;       // Strobe light (purple) 

// Low power state 
const uint32_t ws2812_led_low_pwr = 0x00FFC823;             // Low power (yellow) 

// Not ready state 
const uint32_t ws2812_led_not_ready = 0x00001E00;           // Not ready indicator (red) 

// Ready state 
const uint32_t ws2812_led_ready = 0x00FF6600;               // Ready indicator (orange) 

//==================================================

//==================================================
// Controller settings 

// LEDs used as strobe lights - 1-bit per LED (8 total) 
const uint8_t ws2812_strobe_leds = 0x3C; 
// LED update software timer x this gives strobe period (s) 
const uint8_t ws2812_strobe_period = 10; 

//==================================================

//=======================================================================================
