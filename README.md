# Autonomous Boat 

Unmanned boat running <a href="https://github.com/samdonnelly/autonomous-navigation">TerraPilot</a>. This repository contains the hardware specific code. Check out my <a href="https://github.com/samdonnelly/autonomous-navigation">project build page</a> for more info. 

### Hardware 

The main controller for this build is an STM32F4. 

### Dependencies 

The following repositories are used to build this project: 
- <a href="https://github.com/samdonnelly/autonomous-navigation">TerraPilot</a> - autopilot software 
- <a href="https://github.com/samdonnelly/STM32F4-driver-library">STM32F4 driver library</a> - hardware and peripheral code for the STM32F4 
- <a href="https://github.com/mavlink/c_library_v2">MAVLink</a> - telemetry protocol (used by the autopilot) 
- <a href="https://github.com/ARM-software/CMSIS_6">CMSIS</a> - Standard embedded software interface (used by the autopilot) 
- <a href="https://github.com/ARM-software/CMSIS-FreeRTOS">CMSIS FreeRTOS</a> - CMSIS adoption of FreeRTOS (used by the autopilot) 
