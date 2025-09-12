/**
 * @file system_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System configuration 
 * 
 * @details This is the configuration file for the autopilot software. See the system 
 *          configuration template file from the autopilot. 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _SYSTEM_CONFIG_H_ 
#define _SYSTEM_CONFIG_H_ 

//=======================================================================================
// Vehicle Type (VT) 
//=======================================================================================


//=======================================================================================
// Vehicle Hardware (VH) 

#define VH_DEBUG_OUTPUT 0 

//=======================================================================================


//=======================================================================================
// Vehicle Settings (VS) 

// Telemetry ID 
#define VS_SYSTEM_ID 1 
#define VS_SYSTEM_ID_GCS 255 

// Timing 
#define VS_HEARTBEAT_TIMEOUT 100   // Max count before heartbeat timeout 
#define VS_MISSION_TIMEOUT 10      // Max count before mission protocol timeout 
#define VS_MISSION_RESEND 5        // Max tries to resend a mission message 
#define VS_NAV_DEVICE_TIMEOUT 10   // Max count before navigation devices are considered lost 
#define VS_RC_TIMEOUT 10           // Max count before RC comms are considered lost 

// Data sizes 
#define VS_TELEMETRY_BUFF 1000     // Telemetry data buffer size (bytes) 

// Propulsion and steering 
#define VS_MOTOR_PWM_OFF 1520      // PWM to turn motor(s) off - can vary between motors/ESCs 

// Navigation 
#define VS_MAG_CAL 1               // Include magnetometer calibration correction 

//==================================================
// To be made into parameters 

#define VS_TN_OFFSET 134           // Offset between true and magnetic North (degrees*10) 
#define VS_WAYPOINT_RADIUS 3.0f    // Vehicle acceptance distance to waypoint 

#define VS_COMPASS_HIX 19.7f       // Compass X-axis hard-iron offset 
#define VS_COMPASS_HIY 60.1f       // Compass Y-axis hard-iron offset 
#define VS_COMPASS_HIZ 264.5f      // Compass Z-axis hard-iron offset 

#define VS_COMPASS_SIDX 0.960f     // Compass X-axis soft-iron diagonal correction 
#define VS_COMPASS_SIDY 1.024f     // Compass Y-axis soft-iron diagonal correction 
#define VS_COMPASS_SIDZ 1.019f     // Compass Z-axis soft-iron diagonal correction 

#define VS_COMPASS_SIOX -0.014f    // Compass X-axis soft-iron off-diagonal correction 
#define VS_COMPASS_SIOY 0.004f     // Compass Y-axis soft-iron off-diagonal correction 
#define VS_COMPASS_SIOZ -0.023f    // Compass Z-axis soft-iron off-diagonal correction 

//==================================================

// The vehicle specific settings below only have an affect when the corresponding vehicle 
// type (VT) is selected. 

// Boat 
#define VS_BOAT_K1 1               // Kinematics 1 - Differential thruster - 2 propellers 
#define VS_BOAT_K2 0               // Kinematics 2 - Rudder - 1 propeller + 1 rudder 

//=======================================================================================

#endif   // _SYSTEM_CONFIG_H_ 
