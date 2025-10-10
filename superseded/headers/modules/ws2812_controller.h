/**
 * @file ws2812_controller.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 (Neopixel) controller interface 
 * 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _WS2812_CONTROLLER_H_ 
#define _WS2812_CONTROLLER_H_ 

//=======================================================================================
// Includes 

#include "ws2812_driver.h" 

//=======================================================================================


//=======================================================================================
// LED object 

class WS2812_Controller 
{
private:   // Private members 

    // Device info 
    device_number_t device_num; 

    // Colour data data 
    uint32_t led_colours[WS2812_LED_NUM]; 

    // Strobe info 
    uint8_t strobe_mask;         // 8-bit bitmask - indicates which LEDs are strobe LEDs 
    uint8_t strobe_counter;      // Counter that dictates the strobe state 
    uint8_t strobe_period;       // Number of counts between strobes 
    uint32_t strobe_colour;      // Colour of the strobe LEDs (settable) 

public:   // Public member functions 
    
    // Constructor(s) 
    WS2812_Controller(device_number_t device_number); 
    WS2812_Controller(
        device_number_t device_number, 
        uint8_t strobe_led_mask, 
        uint8_t strobe_counter_period); 

    // Destructor 
    ~WS2812_Controller() {} 

    //==================================================
    // Setters 

    // Set strobe colour 
    void SetStrobeColour(uint32_t led_colour); 

    // Set LED to the specified colour 
    void SetLEDColour(
        ws2812_led_index_t led_num, 
        uint32_t led_colour); 

    // Set LEDs to the specified colour 
    void SetLEDsColour(
        uint8_t led_nums, 
        uint32_t led_colour); 

    //==================================================

    //==================================================
    // Write/update user functions 
    
    // Strobe control 
    void Strobe(void); 
    
    // Turns strobe light off 
    void StrobeOff(void); 

    // Write the current values to the device 
    void LEDWrite(void); 
    
    //==================================================

private:   // Private member functions 

    // Update strobe LEDs to the specified colour 
    void StrobeColourUpdateWrite(uint32_t led_colour); 

    // Update specified LEDs to the specified colour 
    void LEDColourUpdate(
        uint32_t led_colour, 
        uint8_t mask); 
}; 

//=======================================================================================

#endif   // _WS2812_CONTROLLER_H_ 
