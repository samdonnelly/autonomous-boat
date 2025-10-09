/**
 * @file hardware.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware 
 * 
 * @details This file is added by the project (not the autopilot) to define the vehicle 
 *          hardware functions. These functions provide a hardware specific interface 
 *          that's unique to each project that uses the autopilot. The autopilot 
 *          purposely does not define these because their definition will change 
 *          depending on what the user wants to use. 
 * 
 * @version 0.1
 * @date 2025-02-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Include 

// Autopilot 
#include "hardware.h" 

// Project 
#include "stm32f4xx_it.h" 
#include "includes_drivers.h" 
#include "device_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Buffer sizes 
static constexpr uint16_t rc_msg_buff_size = 500;
static constexpr uint8_t adc_buff_size = 3;
static constexpr uint16_t user_max_output_str_size = 200;
static constexpr uint8_t path_size = 200;

// Memory 
static constexpr char memory_dir_path[] = "autonomous_boat";
static constexpr uint32_t memory_min_free = 100000;   // KB 

//=======================================================================================


//=======================================================================================
// Hardware data 

class Hardware final
{
public:

    /**
     * @brief Constructor 
     */
    Hardware();

    /**
     * @brief Destructor 
     */
    ~Hardware() = default;

    template <size_t SIZE>
    struct SerialData 
    {
        USART_TypeDef *uart; 
        DMA_Stream_TypeDef *dma_stream; 
        uint8_t cb[SIZE];                 // Circular buffer populated by DMA 
        cb_index_t cb_index;              // Circular buffer indexing info 
        dma_index_t dma_index;            // DMA transfer indexing info 
        uint8_t data_in[SIZE];            // Buffer that stores latest UART input 
        uint16_t data_in_index;           // Data input buffer index 
        uint8_t data_out[SIZE];           // Buffer that stores outgoing data 
        uint16_t data_out_size;           // Size of the outgoing data 
    };

    // Peripherals 
    I2C_TypeDef *i2c;
    SPI_TypeDef *spi;
    USART_TypeDef *uart_user;
    TIM_TypeDef *timer_esc;
    TIM_TypeDef *timer_generic;
    ADC_TypeDef *adc;
    DMA_Stream_TypeDef *adc_dma_stream;
    GPIO_TypeDef *gpio_sd;

    // ADC 
    uint16_t adc_buff[adc_buff_size];     // ADC buffer - battery and PSU voltage 

    // Actuators 
    device_number_t esc_left, esc_right;

    // Debug 
    char user_output_str[user_max_output_str_size];

    // GPS 
    M8Q_STATUS m8q_status;

    // IMU 
    device_number_t mpu6050_device_num;
    uint8_t mpu6050_st_result;
    MPU6050_STATUS mpu6050_status;
    LSM303AGR_STATUS lsm303agr_status;
    std::array<float, NUM_AXES> accel, gyro, mag;

    // Memory 
    pin_selector_t sd_ss_pin;   // Slave select pin number 
    FRESULT fresult;            // Result of file system operation 
    FATFS file_sys;             // File system 
    FILINFO fno;                // File information 
    FIL file;                   // File object 
    TCHAR path[path_size];
    TCHAR file_read_data[memory_buff_size], file_write_data[memory_buff_size];
    FSIZE_t file_pos;

    // RC 
    SerialData<rc_msg_buff_size> rc;

    // Telemetry 
    SerialData<telemetry_buff_size> telemetry;
};

static Hardware hardware; 

//=======================================================================================


//=======================================================================================
// Initialization 

// Constructor 
Hardware::Hardware()
    : i2c(I2C1),
      spi(SPI2),
      uart_user(USART2),
      timer_esc(TIM3),
      timer_generic(TIM9),
      adc(ADC1),
      adc_dma_stream(DMA2_Stream0),
      gpio_sd(GPIOB),
      adc_buff{},
      esc_left(DEVICE_TWO), esc_right(DEVICE_ONE),
      user_output_str{},
      m8q_status(M8Q_OK),
      mpu6050_device_num(DEVICE_ONE),
      mpu6050_st_result(RESET),
      mpu6050_status(MPU6050_OK),
      lsm303agr_status(LSM303AGR_OK),
      accel{}, gyro{}, mag{},
      sd_ss_pin(PIN_12),
      fresult(FR_OK),
      file_sys(),
      fno(),
      file(),
      path{},
      file_read_data{}, file_write_data{},
      rc(),
      telemetry()
{
    // RC 
    rc.uart = USART6;
    rc.dma_stream = DMA2_Stream1;
    rc.cb_index.cb_size = rc_msg_buff_size;
    rc.dma_index.ndt_old = dma_ndt_read(hardware.rc.dma_stream);

    // Telemetry 
    telemetry.uart = USART1;
    telemetry.dma_stream = DMA2_Stream2;
    telemetry.cb_index.cb_size = telemetry_buff_size;
    telemetry.dma_index.ndt_old = dma_ndt_read(hardware.telemetry.dma_stream);
}


/**
 * @brief Vehicle hardware setup code 
 * 
 * @details This function is called before the autopilot scheduler starts and it provides 
 *          an opportunity for devices and peripherals to be initialized for the chosen 
 *          hardware. 
 */
void VehicleHardware::HardwareSetup(void)
{
    //==================================================
    // General 

    // Initialize GPIO ports 
    gpio_port_init();

    //==================================================

    //==================================================
    // Timers 

    // General purpose 1us counter 
    tim_9_to_11_counter_init(
        hardware.timer_generic, 
        TIM_84MHZ_1US_PSC, 
        HIGH_16BIT,  // Max ARR value 
        TIM_UP_INT_DISABLE); 
    tim_enable(hardware.timer_generic); 

    //==================================================

    //==================================================
    // UART 
    
    // UART1 init - SiK radio module 
    uart_init(
        hardware.telemetry.uart, 
        GPIOA, 
        PIN_10, 
        PIN_9, 
        UART_PARAM_DISABLE,    // Word length 
        RESET,                 // STOP bits 
        UART_FRAC_84_57600, 
        UART_MANT_84_57600, 
        UART_PARAM_DISABLE, 
        UART_PARAM_ENABLE); 

    // UART1 interrupt init - SiK radio module - IDLE line (RX) interrupts 
    uart_interrupt_init(
        hardware.telemetry.uart, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_ENABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE); 

    // UART2 init - Serial terminal 
    uart_init(
        hardware.uart_user, 
        GPIOA, 
        PIN_3, 
        PIN_2, 
        UART_PARAM_DISABLE, 
        RESET, 
        UART_FRAC_42_115200, 
        UART_MANT_42_115200, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE); 

    // UART6 init - RC receiver 
    ibus_init(
        hardware.rc.uart, 
        GPIOA, 
        PIN_12, 
        PIN_11, 
        UART_PARAM_DISABLE, 
        UART_PARAM_ENABLE); 

    // UART6 interrupt init - RC receiver - IDLE line (RX) interrupts 
    uart_interrupt_init(
        hardware.rc.uart, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_ENABLE, 
        UART_PARAM_DISABLE, 
        UART_PARAM_DISABLE); 

    //==================================================

    //==================================================
    // SPI 
    
    // SPI and slave select pins: SD card 
    spi_init(
        hardware.spi, 
        hardware.gpio_sd,    // GPIO port for SCK pin 
        PIN_10,              // SCK pin 
        hardware.gpio_sd,    // GPIO port for data (MISO/MOSI) pins 
        PIN_14,              // MISO pin 
        PIN_15,              // MOSI pin 
        SPI_BR_FPCLK_8, 
        SPI_CLOCK_MODE_0);
    spi_ss_init(hardware.gpio_sd, hardware.sd_ss_pin);

    //==================================================

    //==================================================
    // I2C 
    
    // I2C: GPS, IMU and magnetometer 
    i2c_init(
        hardware.i2c, 
        PIN_9, 
        GPIOB, 
        PIN_8, 
        GPIOB, 
        I2C_MODE_SM,
        I2C_APB1_42MHZ,
        I2C_CCR_SM_42_100,
        I2C_TRISE_1000_42); 
    
    //==================================================

    //==================================================
    // ADC 

    // // Main and auxiliary battery voltages and 5V PSU voltage 

    // // Initialize the ADC port 
    // adc1_clock_enable(RCC); 
    // adc_port_init(
    //     hardware.adc, 
    //     ADC1_COMMON, 
    //     ADC_PCLK2_4, 
    //     ADC_RES_8, 
    //     ADC_PARAM_ENABLE,      // ADC_EOC_EACH 
    //     ADC_PARAM_DISABLE,     // ADC_EOC_INT_DISABLE 
    //     ADC_PARAM_ENABLE,      // ADC_SCAN_ENABLE 
    //     ADC_PARAM_DISABLE,     // ADC_CONT_DISABLE 
    //     ADC_PARAM_ENABLE,      // ADC_DMA_ENABLE 
    //     ADC_PARAM_ENABLE,      // ADC_DDS_ENABLE 
    //     ADC_PARAM_DISABLE);    // ADC_OVR_INT_DISABLE 

    // // Initialize the ADC pins and channels 
    // adc_pin_init(hardware.adc, GPIOA, PIN_6, ADC_CHANNEL_6, ADC_SMP_15);   // Main battery voltage 
    // adc_pin_init(hardware.adc, GPIOA, PIN_7, ADC_CHANNEL_7, ADC_SMP_15);   // Auxiliary battery voltage 
    // adc_pin_init(hardware.adc, GPIOA, PIN_4, ADC_CHANNEL_4, ADC_SMP_15);   // 5V PSU voltage 

    // // Set the ADC conversion sequence 
    // adc_seq(hardware.adc, ADC_CHANNEL_6, ADC_SEQ_1);   // Main battery voltage 
    // adc_seq(hardware.adc, ADC_CHANNEL_7, ADC_SEQ_2);   // Auxiliary battery voltage 
    // adc_seq(hardware.adc, ADC_CHANNEL_4, ADC_SEQ_3);   // 5V PSU voltage 

    // // Set the sequence length (called once and only for more than one channel) 
    // adc_seq_len_set(hardware.adc, (adc_seq_num_t)adc_buff_size); 

    // // Turn the ADC on 
    // adc_on(hardware.adc); 

    //==================================================

    //==================================================
    // DMA 
    
    // DMA2 stream init - UART1 - SiK radio module 
    dma_stream_init(
        DMA2, 
        hardware.telemetry.dma_stream, 
        DMA_CHNL_4, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_HI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
        DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
        DMA_DATA_SIZE_BYTE, 
        DMA_DATA_SIZE_BYTE); 
        
    // DMA2 stream config - UART1 - SiK radio module 
    dma_stream_config(
        hardware.telemetry.dma_stream, 
        (uint32_t)(&hardware.telemetry.uart->DR), 
        (uint32_t)hardware.telemetry.cb, 
        (uint32_t)NULL, 
        (uint16_t)telemetry_buff_size); 

    // DMAX stream init - UART6 - RC receiver 
    dma_stream_init(
        DMA2, 
        hardware.rc.dma_stream, 
        DMA_CHNL_5, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_HI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
        DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
        DMA_DATA_SIZE_BYTE, 
        DMA_DATA_SIZE_BYTE); 
    
    // DMAX stream config - UART6 - RC receiver 
    dma_stream_config(
        hardware.rc.dma_stream, 
        (uint32_t)(&hardware.rc.uart->DR), 
        (uint32_t)hardware.rc.cb, 
        (uint32_t)NULL, 
        (uint16_t)rc_msg_buff_size); 

    // // DMA2 stream init - ADC1 - Voltages 
    // dma_stream_init(
    //     DMA2, 
    //     hardware.adc_dma_stream, 
    //     DMA_CHNL_0, 
    //     DMA_DIR_PM, 
    //     DMA_CM_ENABLE,
    //     DMA_PRIOR_VHI, 
    //     DMA_DBM_DISABLE, 
    //     DMA_ADDR_INCREMENT, 
    //     DMA_ADDR_FIXED, 
    //     DMA_DATA_SIZE_HALF, 
    //     DMA_DATA_SIZE_HALF); 

    // // DMA2 stream config - ADC1 - Voltages 
    // dma_stream_config(
    //     hardware.adc_dma_stream, 
    //     (uint32_t)(&hardware.adc->DR), 
    //     (uint32_t)hardware.adc_buff, 
    //     (uint32_t)NULL, 
    //     (uint16_t)adc_buff_size); 

    // Enable DMA streams 
    dma_stream_enable(hardware.telemetry.dma_stream);   // UART1 - Sik radio 
    dma_stream_enable(hardware.rc.dma_stream);          // UART6 - RC receiver 
    // dma_stream_enable(hardware.adc_dma_stream);         // ADC1 - Voltages 

    //==================================================

    //==================================================
    // Interrupts 

    // Initialize interrupt handler flags 
    int_handler_init(); 

    // Enable the interrupt handlers 
    nvic_config(USART1_IRQn, EXTI_PRIORITY_0);   // UART1 - SiK radio 
    nvic_config(USART6_IRQn, EXTI_PRIORITY_1);   // UART6 - RC receiver 
    // nvic_config(ADC_IRQn, EXTI_PRIORITY_2);      // ADC1 - voltages 

    //==================================================

    //==================================================
    // GPS 

    // SAM-M8Q driver init 
    hardware.m8q_status = m8q_init(
        hardware.i2c, 
        &m8q_config_msgs[0][0], 
        M8Q_CONFIG_MSG_NUM, 
        M8Q_CONFIG_MSG_MAX_LEN, 
        CLEAR); 

    // Set up low power and TX ready pins 
    m8q_pwr_pin_init(GPIOC, PIN_10); 
    m8q_txr_pin_init(GPIOC, PIN_11); 

    //==================================================

    //==================================================
    // IMUs 

    // MPU-6050 driver init 
    hardware.mpu6050_status |= mpu6050_init(
        DEVICE_ONE, 
        hardware.i2c, 
        MPU6050_ADDR_1,
        mpu6050_standby_mask, 
        MPU6050_DLPF_CFG_1,
        mpu6050_sample_rate_divider,
        MPU6050_AFS_SEL_4,
        MPU6050_FS_SEL_500);
    mpu6050_set_offsets(DEVICE_ONE, mpu6050_accel_offsets, mpu6050_gyro_offsets);

    // LSM303AGR driver init 
    hardware.lsm303agr_status |= lsm303agr_m_init(
        hardware.i2c, 
        LSM303AGR_M_ODR_50, 
        LSM303AGR_M_MODE_CONT, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE);
    lsm303agr_m_calibration_set(lsm303agr_hi_offset, lsm303agr_sid_values, lsm303agr_sio_values); 

    //==================================================

    //==================================================
    // Radios 
    
    // SiK telemetry radio driver init 
    sik_init(hardware.telemetry.uart); 

    //==================================================

    //==================================================
    // Memory 

    // SD card driver init 
    fatfs_user_init(hardware.spi, hardware.gpio_sd, static_cast<uint16_t>(SET_BIT << hardware.sd_ss_pin));

    //==================================================

    //==================================================
    // LEDs 

    // The timer port (not just the channel) used for the LEDs must be different than the 
    // timer used for the ESCs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // // WS2812 module driver init (Neopixel LEDs) - two LEDs run on one signal 
    // ws2812_init(
    //     DEVICE_ONE, 
    //     TIM4, 
    //     TIMER_CH2, 
    //     GPIOB, 
    //     PIN_7); 
    
    //==================================================

    //==================================================
    // ESCs 

    // The timer port (not just the channel) used for the ESCs must be different than the 
    // timer used for the LEDs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // ESC driver init - right thruster (ESC1) 
    esc_init(
        hardware.esc_right, 
        hardware.timer_esc, 
        TIMER_CH4, 
        GPIOB, 
        PIN_1, 
        TIM_84MHZ_1US_PSC, 
        esc_period, 
        esc_fwd_speed_lim, 
        esc_rev_speed_lim); 

    // ESC driver init - left thruster (ESC2) 
    esc_init(
        hardware.esc_left, 
        hardware.timer_esc, 
        TIMER_CH3, 
        GPIOB, 
        PIN_0, 
        TIM_84MHZ_1US_PSC, 
        esc_period, 
        esc_fwd_speed_lim, 
        esc_rev_speed_lim); 

    // Enable the PWM timer 
    tim_enable(hardware.timer_esc); 

    //==================================================

    //==================================================
    // Device setup processes 

    // Some devices have checks to see if the device is present or data is accurate. 
    
    //==================================================
}

//=======================================================================================


//=======================================================================================
// Interrupt callbacks 

// Any needed callbacks are overridden here so hardware data doesn't need to be included 
// in the interrupt file. 

//=======================================================================================


//=======================================================================================
// Actuators 

/**
 * @brief Set the motor outpur PWM 
 * 
 * @details This function is called from the autopilots main thread to set the propulsion 
 *          motor(s) PWM output during travelling modes. Two motor outputs are provided 
 *          for vehicles that use differential thrust but only throttle_1 will be used 
 *          when a vehicle is configured for one propulsion ouptut. 
 * 
 * @param throttle_1 : motor 1 PWM (1000-2000) 
 * @param throttle_2 : motor 2 PWM (optional) (1000-2000) 
 */
void VehicleHardware::PropulsionSet(
    uint16_t throttle_1, 
    uint16_t throttle_2)
{
    esc_pwm_set(hardware.esc_left, throttle_1); 
    esc_pwm_set(hardware.esc_right, throttle_2); 
}


/**
 * @brief Set the steering servo/motor output PWM 
 * 
 * @details This function is called from the autopilots main thread to set the steering 
 *          motor(s) PWM output during travelling modes. 
 * 
 * @param roll : roll control PWM (1000-2000) 
 * @param pitch : pitch control PWM (1000-2000) 
 * @param yaw : yaw control PWM (1000-2000) 
 */
void VehicleHardware::SteeringSet(
    uint16_t roll, 
    uint16_t pitch, 
    uint16_t yaw)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Debug 

/**
 * @brief Write data to a serial connection 
 * 
 * @details This function is used when the user needs to temporarily send data to an 
 *          external device for things such as debugging or IMU calibration through 
 *          3rd party software. VH_DEBUG_OUTPUT must be enabled in the system config 
 *          for this function to be called. This function will be called regardless of 
 *          vehicle state. 
 */
void VehicleHardware::DebugWrite(void)
{
    //==================================================
    // IMU calibration with MotionCal 

    int16_t mag_data[NUM_AXES]; 
    
    lsm303agr_m_get_axis(mag_data); 
    
    // The following string format is needed for MotionCal to read it. 
    snprintf(
        hardware.user_output_str, 
        user_max_output_str_size, 
        "Raw:0,0,0,0,0,0,%d,%d,%d\r\n", 
        mag_data[X_AXIS], 
        -mag_data[Y_AXIS], 
        -mag_data[Z_AXIS]); 
    uart_send_str(hardware.uart_user, hardware.user_output_str); 
    
    //==================================================
}

//=======================================================================================


//=======================================================================================
// GPS 

/**
 * @brief Read from a GPS device 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from aGPSU device. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.gps_ready flag should be set if new 
 *          data is available and the autopilot will call GPSGet to collect the data 
 *          during the main thread. 
 * 
 * @see GPSGet 
 */
void VehicleHardware::GPSRead(void)
{
    if (m8q_get_tx_ready() == GPIO_HIGH)
    {
        hardware.m8q_status = m8q_read_data();
        
        if (hardware.m8q_status == M8Q_OK)
        {
            data_ready.gps_ready = FLAG_SET; 
        }
    }
}


/**
 * @brief Get the data read from a GPS device 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from a GPS device if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.gps_ready flag is set in the 
 *          GPSRead function. 
 *          
 *          The latitude, longitude and altitude must be supplied in both unsigned int 
 *          and float forms as both are used by the autopilot. The GPS position lock 
 *          status must also be returned. 
 *          
 *          Integer coordinates are expressed in degrees*10^7 to maintain precision. 
 *          Float coordinates are expressed in degrees with decimal values. 
 * 
 * @see GPSRead 
 * 
 * @param location : buffer to store latitude, longitude and altitude from the GPS 
 * @param location_uncertainty : buffer to store location uncertainty from the GPS measurement 
 * @param velocity : buffer to store speed over ground, course over ground and vertical velocity from the GPS 
 * @param velocity_uncertainty : buffer to store velocity uncertainty from the GPS measurement 
 * @return true/false : GPS position lock status 
 */
bool VehicleHardware::GPSGet(
    VehicleNavigation::Location &location,
    VehicleNavigation::Location &location_uncertainty,
    VehicleNavigation::Velocity &velocity,
    VehicleNavigation::Velocity &velocity_uncertainty)
{
    location.lat = m8q_get_position_lat();                                    // degrees 
    location.lon = m8q_get_position_lon();                                    // degrees 
    location.alt = m8q_get_position_altref();                                 // meters 
    location.latI = static_cast<int32_t>(location.lat * coordinate_scalar);   // degrees*E7 
    location.lonI = static_cast<int32_t>(location.lon * coordinate_scalar);   // degrees*E7 
    location.altI = static_cast<int32_t>(location.alt * altitude_scalar);     // millimeters 
    location_uncertainty.lat = m8q_get_position_hacc();                       // meters 
    location_uncertainty.lon = location_uncertainty.lat;                      // meters 
    location_uncertainty.alt = m8q_get_position_vacc();                       // meters 

    velocity.sog = m8q_get_position_sog() / kph_to_mps;                       // m/s 
    velocity.cog = m8q_get_position_cog();                                    // degrees 
    velocity.vvel = m8q_get_position_vvel();                                  // m/s 
    velocity_uncertainty.sog = gps_vel_variance[X_AXIS];                      // m/s 
    velocity_uncertainty.cog = gps_vel_variance[Y_AXIS];                      // degrees 
    velocity_uncertainty.vvel = gps_vel_variance[Z_AXIS];                     // m/s 

    return m8q_get_position_navstat_lock(); 
}

//=======================================================================================


//=======================================================================================
// IMU 

/**
 * @brief Read from IMU device(s) 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from an IMU device. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.imu_ready flag should be set if new 
 *          data is available and the autopilot will call IMUGet to collect the data 
 *          during the main thread. 
 * 
 * @see IMUGet 
 */
void VehicleHardware::IMURead(void)
{
    hardware.mpu6050_status = mpu6050_update(hardware.mpu6050_device_num);
    hardware.lsm303agr_status = lsm303agr_m_update();

    if ((hardware.mpu6050_status == MPU6050_OK) && (hardware.lsm303agr_status == LSM303AGR_OK))
    {
        data_ready.imu_ready = FLAG_SET; 
    }
}


/**
 * @brief Get the most recent IMU data 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from an IMU device if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.imu_ready flag is set in the 
 *          IMURead function. 
 *          
 *          IMUs should have their positive X-axis in the vehicles forward direction, 
 *          their positive y-axis pointing to the left and the positive z-axis pointing 
 *          up (NWU orientation following right hand rule) to ensure orientation is 
 *          calculated properly. If the accelerometer, gyroscope or magnetometer axes 
 *          don't align with this then just invert the sign of the axis reading. If signs 
 *          are inverted, make sure calibration values accommodate this. 
 *          
 *          Magnetometer readings can be supplied as calibrated or uncalibrated values. 
 *          If not pre-calibrated, make sure to enable calibration correction and set 
 *          the magnetometer calibration parameters (hard and soft iron offsets). See 
 *          VS_MAG_CAL in the system configuration file. 
 *          
 *          Note that the accelerometer and gyroscope units matter but the magnetometer 
 *          units don't. Acceleration should be provided in g's and angular rate should 
 *          be provided in deg/s. Calculations that use magnetic fiels only care about 
 *          the magnitude between axes. However, typical magnetometer units are mG or uT. 
 * 
 * @see IMURead 
 * 
 * @param accel : 3-axis accelerometer data (g's)
 * @param gyro : 3-axis gyroscope data (deg/s) 
 * @param mag : 3-axis magnetometer axis data (typically mG or uT) 
 */
void VehicleHardware::IMUGet(
    VehicleNavigation::Vector<float> &accel, 
    VehicleNavigation::Vector<float> &gyro, 
    VehicleNavigation::Vector<float> &mag)
{
    // Accelerometer 
    mpu6050_get_accel_axis_gs(DEVICE_ONE, hardware.accel.data()); 
    accel.x = hardware.accel[X_AXIS]; 
    accel.y = hardware.accel[Y_AXIS]; 
    accel.z = hardware.accel[Z_AXIS]; 

    // Gyroscope 
    mpu6050_get_gyro_axis_rate(DEVICE_ONE, hardware.gyro.data()); 
    gyro.x = hardware.gyro[X_AXIS]; 
    gyro.y = hardware.gyro[Y_AXIS]; 
    gyro.z = hardware.gyro[Z_AXIS]; 
    
    // Magnetometer - uncalibrated 
    lsm303agr_m_get_axis_f(hardware.mag.data());
    mag.x = hardware.mag[X_AXIS];
    mag.y = -hardware.mag[Y_AXIS];   // Sign inverted to change y-axis direction 
    mag.z = hardware.mag[Z_AXIS];
}

//=======================================================================================


//=======================================================================================
// Memory 

/**
 * @brief Setup the external memory device for file access 
 * 
 * @details The autopilot calls this function to establish the external memory device. 
 *          Everything needed to get talking to the external memory should be done here. 
 *          This could include, but is not limited to, mounting the device, checking for 
 *          sufficient free space, or creating the directory from which to operate from. 
 *          See the description of the MemorySetFileName function for options on file 
 *          paths and directories. No files should be created or opened here as that is 
 *          handled through the implementation of the other memory functions. By the 
 *          end of this function, the device should be ready for the autopilot to use. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetFileName
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_CAP_ERROR --> Insufficient free space on device 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Memory ready for access, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemorySetup(void)
{
    //==================================================
    // Mount the drive (required before it can be accessed). 

    hardware.fresult = f_mount(&hardware.file_sys, "", FATFS_MOUNT_NOW);

    if (hardware.fresult != FR_OK)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }
    
    //==================================================

    //==================================================
    // Check free space on card 

    DWORD free_clusters, free_space;
    FATFS *fs_ptr;
    
    hardware.fresult = f_getfree("", &free_clusters, &fs_ptr);

    if (hardware.fresult == FR_OK)
    {
        // Calculate the free space 
        free_space = free_clusters * static_cast<DWORD>(fs_ptr->csize) / 2;

        if (free_space < memory_min_free)
        {
            return MemoryStatus::MEMORY_CAP_ERROR;
        }
    }

    //==================================================

    //==================================================
    // Check for existance of the chosen directory 

    hardware.fresult = f_stat(memory_dir_path, &hardware.fno);
    
    if (hardware.fresult == FR_NO_FILE)
    {
        // If the directory does not exist then create the directory 
        hardware.fresult = f_mkdir(memory_dir_path);
    }
    
    if (hardware.fresult != FR_OK)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }
    
    //==================================================

    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Set the name of the file to access 
 * 
 * @details The autopilot will call this function when it needs to specify a file to 
 *          open. The file name is provided and this function must use the name to 
 *          establish a path to the file. That path will then be used to open the file 
 *          when the autopilot calls the MemoryOpenFile function. The user can handle the 
 *          path in one of two ways: 
 *          1. The external memory device is already in the directory where files should 
 *             be accessed so only the file name is used as the path. Changing the 
 *             directory should be done in the MemorySetup function, NOT here. 
 *          2. Alternatively, the file name can be appended to the path of the directory 
 *             where files should be accessed. This way the full path from the root is 
 *             used to open the file. 
 *          
 *          The buffer used to store the path should be large enough to store whatever 
 *          path method the user chooses. 
 * 
 * @see MemoryOpenFile
 * @see MemorySetup
 * 
 * @param file_name : string containing the name of the file to be opened 
 * @param file_name_size : size of the file name 
 */
void VehicleHardware::MemorySetFileName(
    const char *file_name, 
    uint16_t file_name_size)
{
    if ((file_name != nullptr) && 
        ((file_name_size + static_cast<uint16_t>(sizeof(memory_dir_path))) < path_size))
    {
        snprintf(hardware.path, path_size, "%s%s%s", memory_dir_path, "/", file_name);
    }
}


/**
 * @brief Open a file on the external memory device 
 * 
 * @details This function will be called by the autopilot when a file needs to be opened 
 *          for reading or writing. The file name is set in MemorySetFileName which will 
 *          be called by the autopilot to establish a path before attempting to open the 
 *          file. This function must use the set path to open the file if it exists 
 *          (without overwritting it) or create and open the file if it does not exist. 
 *          The file should be opened with both read and write permissions and the 
 *          position in the file should be set to the start. The return status must 
 *          indicate if the file previously existed or not so the autopilot knows how to 
 *          handle the file. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetFileName
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_FILE_OPENED --> File exists and was opened 
 *                                         MEMORY_FILE_CREATED --> File did not exist, was created and opened 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryOpenFile(void)
{
    MemoryStatus memory_status;

    // Check for the existance of the specified file 
    hardware.fresult = f_stat(hardware.path, &hardware.fno);

    switch(hardware.fresult)
    {
        case FR_OK:
            // File exists. Autopilot can read from it. 
            memory_status = MemoryStatus::MEMORY_FILE_OPENED;
            break;

        case FR_NO_FILE:
            // File does not exist. Autopilot can write to it. 
            memory_status = MemoryStatus::MEMORY_FILE_CREATED;
            break;

        default:
            memory_status = MemoryStatus::MEMORY_ACCESS_ERROR;
            break;
    }

    if (memory_status != MemoryStatus::MEMORY_ACCESS_ERROR)
    {
        // Attempt to open the file with read and write permissions. The access mode used will 
        // open the file if it exists, or create and open a new file if it does not exist. The 
        // file opens with the position set to the start of the file. 
        hardware.fresult = f_open(&hardware.file, hardware.path, FATFS_MODE_OAWR);
    
        if (hardware.fresult != FR_OK)
        {
            memory_status = MemoryStatus::MEMORY_ACCESS_ERROR;
        }
    }

    return memory_status;
}


/**
 * @brief Close a file on the external memory device 
 * 
 * @details This function will be called by the autopilot when an open file needs to be 
 *          closed. The status of the close operation must be returned. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> File closed, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryCloseFile(void)
{
    hardware.fresult = f_close(&hardware.file);

    if (hardware.fresult != FR_OK)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }

    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Read data from the external memory device 
 * 
 * @details This function will be called by the autopilot when data needs to be read from 
 *          a file. The file that requires reading from should have already been opened 
 *          using the MemoryOpenFile function which the autopilot will have called before 
 *          attempting to read any data. This function must read one line of data from 
 *          the open file, save it and return the status of the operation. Saved data is 
 *          retrieved by the autopilot using the MemoryGetData function which will be 
 *          called following the call to this function. Data in files are formatted into 
 *          individual lines of information and each line gets read one at a time through 
 *          repeated calls to this function. The buffer that read data is saved to must 
 *          be at least the size of 'memory_buff_size' so that all data from a line can 
 *          be captured. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemoryGetData
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_EOF --> End of file reached (no more data) 
 *                                         MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data read, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryRead(void)
{
    TCHAR *read_status = f_gets(hardware.file_read_data, memory_buff_size, &hardware.file);

    if (read_status == nullptr)
    {
        if (f_eof(&hardware.file) != 0)
        {
            return MemoryStatus::MEMORY_EOF;
        }
        else
        {
            return MemoryStatus::MEMORY_ACCESS_ERROR;
        }
    }

    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Get the data read from external memory 
 * 
 * @details The autopilot will call this function to fetch new data that was read from 
 *          external memory device. Data that was saved in the MemoryRead function should 
 *          be copied to the provided buffer so the autopilot can use it. Data is 
 *          formatted into individual lines of information and each line gets read one 
 *          at a time through repeated calls to the read and get functions.
 * 
 * @see MemoryRead
 * 
 * @param data_buff : buffer to store data read from the device 
 * @param data_buff_size : size of buffer that will store the data 
 */
void VehicleHardware::MemoryGetData(
    char *data_buff, 
    uint16_t data_buff_size)
{
    if ((data_buff != nullptr) && (data_buff_size >= memory_buff_size))
    {
        strcpy(data_buff, hardware.file_read_data);
    }
}


/**
 * @brief Set the data to be written to external memory 
 * 
 * @details The autopilot will call this function to stage new data to be writing to the 
 *          external memory device. The provided data should be copied so that it can be 
 *          written when the MemoryWrite function is called. The buffer that the data is 
 *          copied to must be large enough to hold all the provided data. Data is 
 *          formatted into individual lines of information and each line gets written one 
 *          at a time through repeated calls to the set and write functions.
 * 
 * @see MemoryWrite
 * 
 * @param data_buff : buffer containing data to write to the device 
 * @param data_buff_size : size of data in the buffer 
 */
void VehicleHardware::MemorySetData(
    const char *data_buff, 
    uint16_t data_buff_size)
{
    if ((data_buff != nullptr) && (memory_buff_size >= data_buff_size))
    {
        strcpy(hardware.file_write_data, data_buff);
    }
}


/**
 * @brief Write data to the external memory device 
 * 
 * @details This function will be called by the autopilot when data needs to be written 
 *          to a file. The data to be written should have already been copied/saved in 
 *          the MemorySetData function which the autopilot will have called before this. 
 *          The file that requires writing to should have already been opened using the 
 *          MemoryOpenFile function which the autopilot will have also called before 
 *          attempting to write any data. This function must write the data to the open 
 *          file and return the status of the operation. Data is formatted into individual 
 *          lines of information and each line gets written one at a time through repeated 
 *          calls to the set and write functions. Each new line of data must be appended 
 *          to the end of the file. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemorySetData
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data written, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryWrite(void)
{
    int num_chars = f_puts(hardware.file_write_data, &hardware.file);
    
    if (num_chars < 0)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }
    
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Truncate data in an open file 
 * 
 * @details The autopilot will call this function when old file data needs to be removed. 
 *          In some cases, the autopilot needs to overwrite old data saved in external 
 *          memory, and truncating the old data ensures no old data will be left behind 
 *          once new data is done being written. This function should remove all the 
 *          existing file data from the current position within the open file until the 
 *          end of the file. This function shouldn't navigate to a certain point within 
 *          the file as the autopilot will call this function when it needs to. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Data truncated, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryTruncate(void)
{
    hardware.fresult = f_truncate(&hardware.file);
    
    if (hardware.fresult != FR_OK)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }
    
    return MemoryStatus::MEMORY_OK;
}


/**
 * @brief Set the desired position to navigate to within the open file 
 * 
 * @details The autopilot will call this function to stage the desired position within 
 *          the open file. This function should save the position so that when 
 *          MemoryNavigate is called, the position within the file can be updated. The 
 *          autopilot determines the position within the file it needs. If the max 
 *          possible file position is provided (file_position = ~0), this means the 
 *          autopilot needs to navigate to the end of the file. 
 * 
 * @see MemoryNavigate
 * 
 * @param file_position : position within file (bytes) 
 */
void VehicleHardware::MemoryFilePositionSet(uint32_t file_position)
{
    hardware.file_pos = (file_position == ~0U) ? f_size(&hardware.file) : 
                                                 static_cast<FSIZE_t>(file_position);
}


/**
 * @brief Navigate to a position within an open file 
 * 
 * @details This function will be called by the autopilot when the position within the 
 *          open file needs to be updated. The position should have already been saved in 
 *          the MemoryFilePositionSet function which the autopilot will call before this. 
 *          This function must take the set position and move to that point within the 
 *          open file then return the status of the operation. 
 *          
 *          Note that the autopilot makes decisions based on the status of these functions 
 *          so it's important the user provides the correct return values. 
 * 
 * @see MemoryFilePositionSet
 * 
 * @return VehicleHardware::MemoryStatus : MEMORY_ACCESS_ERROR --> Problem accessing device 
 *                                         MEMORY_OK --> Position updated, everything OK 
 */
VehicleHardware::MemoryStatus VehicleHardware::MemoryNavigate(void)
{
    hardware.fresult = f_lseek(&hardware.file, hardware.file_pos);

    if (hardware.fresult != FR_OK)
    {
        return MemoryStatus::MEMORY_ACCESS_ERROR;
    }

    return MemoryStatus::MEMORY_OK;
}

//=======================================================================================


//=======================================================================================
// RC 

/**
 * @brief Read from an RC receiver 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from an RC receiver. Data is read here but not retreived by 
 *          the autopilot. Instead, the data_ready.rc_ready flag should be set if new 
 *          data is available and the autopilot will call RCGet to collect the data 
 *          during the main thread. 
 * 
 * @see RCGet 
 */
void VehicleHardware::RCRead(void)
{
    // There isn't a universal way to check for a transmitter connection loss across all 
    // receivers and transmitters. The user must make sure the proper failsafes are 
    // enabled for their receiver and transmitter setup so their vehicle does not travel 
    // out of reach on it's own if radio control is lost. 

    // The RC receiver communicates via UART (IBUS) and incoming data is processed using 
    // DMA and an interrupt. So instead of a direct call to a read function we check if 
    // the interrupt was run to set the data ready flag. 

    if (handler_flags.usart6_flag)
    {
        handler_flags.usart6_flag = CLEAR_BIT; 
        data_ready.rc_ready = FLAG_SET; 

        // Parse the new radio data from the circular buffer into the data buffer. 
        dma_cb_index(hardware.rc.dma_stream, &hardware.rc.dma_index, &hardware.rc.cb_index); 
        cb_parse(hardware.rc.cb, &hardware.rc.cb_index, hardware.rc.data_in); 
    }
}


/**
 * @brief Get data read from an RC receiver 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from an RC receiver if it's available. Data is copied here but not read. 
 *          This function is only called if the data_ready.rc_ready flag is set in the 
 *          RCRead function. 
 * 
 * @see RCRead 
 * 
 * @param channels : RC channels to populate 
 */
void VehicleHardware::RCGet(VehicleControl::ChannelFunctions &channels)
{
    // Assign packet channel data to their function within the autopilot. Each user can 
    // map their channels how they like. 

    // Get the first full packet in the data buffer from the most recent sampling 
    // interval. 
    ibus_packet_t *packet = ibus_packet_align(hardware.rc.data_in, 
                                              hardware.rc.dma_index.data_size); 

    if (packet != nullptr)
    {
        // Main controls 
        channels.throttle = packet->items[IBUS_CH3]; 
        channels.roll = packet->items[IBUS_CH1]; 
        channels.pitch = packet->items[IBUS_CH2]; 
        channels.yaw = packet->items[IBUS_CH4]; 
        
        // Auxiliary controls 
        channels.mode_control = packet->items[IBUS_CH6]; 
        channels.mode = packet->items[IBUS_CH7]; 
    }
}

//=======================================================================================


//=======================================================================================
// Telemetry 

/**
 * @brief Read from a telemetry device 
 * 
 * @details This function is called periodically from the autopilots communication thread 
 *          to read new data from a telemetry device. Data is read here but not retreived 
 *          by the autopilot. Instead, the data_ready.telemetry_ready flag should be set 
 *          if new data is available and the autopilot will call TelemetryGet to collect 
 *          the data during the main thread. 
 * 
 * @see TelemetryGet 
 */
void VehicleHardware::TelemetryRead(void)
{
    // The telemetry radio communicates via UART and incoming data is processed using DMA 
    // and an interrupt. So instead of a direct call to a read function we check if the 
    // interrupt was run to set the data ready flag. 

    if (handler_flags.usart1_flag)
    {
        handler_flags.usart1_flag = CLEAR_BIT; 
        data_ready.telemetry_ready = FLAG_SET; 

        // Parse the new radio data from the circular buffer into the data buffer. 
        dma_cb_index(
            hardware.telemetry.dma_stream, 
            &hardware.telemetry.dma_index, 
            &hardware.telemetry.cb_index); 
        cb_parse(
            hardware.telemetry.cb, 
            &hardware.telemetry.cb_index, 
            hardware.telemetry.data_in); 
    }
}


/**
 * @brief Get data read from a telemetry device 
 * 
 * @details This function is called from the autopilots main thread to collect new data 
 *          read from a telemetry device if it's available. Data is copied here but not 
 *          read. This function is only called if the data_ready.telemetry_ready flag is 
 *          set in the TelemetryRead function. 
 * 
 * @see TelemetryRead 
 * 
 * @param size : buffer to store the size of data read 
 * @param buffer : buffer to store new data 
 */
void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    size = hardware.telemetry.dma_index.data_size; 
    memcpy((void *)buffer, (void *)hardware.telemetry.data_in, size); 
}


/**
 * @brief Set data to write to a telemtry device 
 * 
 * @details This function is called from the autopilots main thread to set data to write 
 *          to a telemetry device if there's new data to be sent. Data is copied here but 
 *          not written. If this function is called then TelemetryWrite will be executed 
 *          shortly afterwards. 
 * 
 * @see TelemetryWrite 
 * 
 * @param size : size of data to write 
 * @param buffer : buffer containing the data to write 
 */
void VehicleHardware::TelemetrySet(uint16_t &size, uint8_t *buffer)
{
    hardware.telemetry.data_out_size = size; 
    memcpy((void *)hardware.telemetry.data_out, (void *)buffer, size); 
}


/**
 * @brief Write to a telemetry device 
 * 
 * @details This function is called after the TelemetrySet function from the autopilots 
 *          communications thread to write data to a telemetry device. Data is written 
 *          here but not set. 
 * 
 * @see TelemetrySet 
 */
void VehicleHardware::TelemetryWrite(void)
{
    sik_send_data(hardware.telemetry.data_out, hardware.telemetry.data_out_size); 
}

//=======================================================================================
