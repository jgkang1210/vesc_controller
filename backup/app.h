// This file is autogenerated by VESC Tool

#ifndef APP_H_
#define APP_H_

// VESC ID
#define APPCONF_CONTROLLER_ID 29

// Timeout
#define APPCONF_TIMEOUT_MSEC 1000

// Timeout Brake Current
#define APPCONF_TIMEOUT_BRAKE_CURRENT 0

// Can Status Message Mode
#define APPCONF_SEND_CAN_STATUS 4

// Can Status Rate
#define APPCONF_SEND_CAN_STATUS_RATE_HZ 50

// CAN Baud Rate
#define APPCONF_CAN_BAUD_RATE 2

// Pairing Done
#define APPCONF_PAIRING_DONE 0

// Enable Permanent UART
#define APPCONF_PERMANENT_UART_ENABLED 1

// Shutdown Mode
#define APPCONF_SHUTDOWN_MODE 7

// CAN Mode
#define APPCONF_CAN_MODE 0

// UAVCAN ESC Index
#define APPCONF_UAVCAN_ESC_INDEX 0

// UAVCAN Raw Throttle Mode
#define APPCONF_UAVCAN_RAW_MODE 0

// APP to Use
#define APPCONF_APP_TO_USE 8

// Control Type
#define APPCONF_PPM_CTRL_TYPE 6

// PID Max ERPM
#define APPCONF_PPM_PID_MAX_ERPM 15000

// Input Deadband
#define APPCONF_PPM_HYST 0.15

// Pulselength Start
#define APPCONF_PPM_PULSE_START 1

// Pulselength End
#define APPCONF_PPM_PULSE_END 2

// Pulselength Center
#define APPCONF_PPM_PULSE_CENTER 1.5

// Median Filter
#define APPCONF_PPM_MEDIAN_FILTER 1

// Safe Start
#define APPCONF_PPM_SAFE_START 1

// Throttle Expo
#define APPCONF_PPM_THROTTLE_EXP 0

// Throttle Expo Brake
#define APPCONF_PPM_THROTTLE_EXP_BRAKE 0

// Throttle Expo Mode
#define APPCONF_PPM_THROTTLE_EXP_MODE 2

// Positive Ramping Time
#define APPCONF_PPM_RAMP_TIME_POS 0.4

// Negative Ramping Time
#define APPCONF_PPM_RAMP_TIME_NEG 0.2

// Multiple VESCs Over CAN
#define APPCONF_PPM_MULTI_ESC 1

// Traction Control
#define APPCONF_PPM_TC 0

// TC Max ERPM Difference
#define APPCONF_PPM_TC_MAX_DIFF 3000

// Max ERPM for direction switch
#define APPCONF_PPM_MAX_ERPM_FOR_DIR 4000

// Smart Reverse Max Duty Cycle
#define APPCONF_PPM_SMART_REV_MAX_DUTY 0.07

// Smart Reverse Ramp Time
#define APPCONF_PPM_SMART_REV_RAMP_TIME 3

// Control Type
#define APPCONF_ADC_CTRL_TYPE 0

// Input Deadband
#define APPCONF_ADC_HYST 0.15

// ADC1 Min Voltage
#define APPCONF_ADC_VOLTAGE_START 0.9

// ADC1 Max Voltage
#define APPCONF_ADC_VOLTAGE_END 3

// ADC1 Center Voltage
#define APPCONF_ADC_VOLTAGE_CENTER 2

// ADC2 Min Voltage
#define APPCONF_ADC_VOLTAGE2_START 0.9

// ADC2 Max Voltage
#define APPCONF_ADC_VOLTAGE2_END 3

// Use Filter
#define APPCONF_ADC_USE_FILTER 1

// Safe Start
#define APPCONF_ADC_SAFE_START 1

// Invert Cruise Control Button
#define APPCONF_ADC_CC_BUTTON_INVERTED 0

// Invert Reverse Button
#define APPCONF_ADC_REV_BUTTON_INVERTED 0

// Invert ADC1 Voltage
#define APPCONF_ADC_VOLTAGE_INVERTED 0

// Invert ADC2 Voltage
#define APPCONF_ADC_VOLTAGE2_INVERTED 0

// Throttle Expo
#define APPCONF_ADC_THROTTLE_EXP 0

// Throttle Expo Brake
#define APPCONF_ADC_THROTTLE_EXP_BRAKE 0

// Throttle Expo Mode
#define APPCONF_ADC_THROTTLE_EXP_MODE 2

// Positive Ramping Time
#define APPCONF_ADC_RAMP_TIME_POS 0.3

// Negative Ramping Time
#define APPCONF_ADC_RAMP_TIME_NEG 0.1

// Multiple VESCs Over CAN
#define APPCONF_ADC_MULTI_ESC 1

// Traction Control
#define APPCONF_ADC_TC 0

// TC Max ERPM Difference
#define APPCONF_ADC_TC_MAX_DIFF 3000

// Update Rate
#define APPCONF_ADC_UPDATE_RATE_HZ 500

// Baudrate
#define APPCONF_UART_BAUDRATE 115200

// Control Type
#define APPCONF_CHUK_CTRL_TYPE 1

// Input Deadband
#define APPCONF_CHUK_HYST 0.15

// Positive Ramping Time
#define APPCONF_CHUK_RAMP_TIME_POS 0.4

// Negative Ramping Time
#define APPCONF_CHUK_RAMP_TIME_NEG 0.2

// ERPM Per Second Cruise Control
#define APPCONF_STICK_ERPM_PER_S_IN_CC 3000

// Throttle Expo
#define APPCONF_CHUK_THROTTLE_EXP 0

// Throttle Expo Brake
#define APPCONF_CHUK_THROTTLE_EXP_BRAKE 0

// Throttle Expo Mode
#define APPCONF_CHUK_THROTTLE_EXP_MODE 2

// Multiple VESCs Over CAN
#define APPCONF_CHUK_MULTI_ESC 1

// Traction Control
#define APPCONF_CHUK_TC 0

// TC Max ERPM Difference
#define APPCONF_CHUK_TC_MAX_DIFF 3000

// Use Smart Reverse
#define APPCONF_CHUK_USE_SMART_REV 1

// Smart Reverse Max Duty Cycle
#define APPCONF_CHUK_SMART_REV_MAX_DUTY 0.07

// Smart Reverse Ramp Time
#define APPCONF_CHUK_SMART_REV_RAMP_TIME 3

// Speed
#define APPCONF_NRF_SPEED 1

// TX Power
#define APPCONF_NRF_POWER 3

// CRC
#define APPCONF_NRF_CRC 1

// Retry Delay
#define APPCONF_NRF_RETR_DELAY 0

// Retries
#define APPCONF_NRF_RETRIES 3

// Radio Channel
#define APPCONF_NRF_CHANNEL 76

// Address 0
#define APPCONF_NRF_ADDR_B0 198

// Address 1
#define APPCONF_NRF_ADDR_B1 199

// Address 2
#define APPCONF_NRF_ADDR_B2 0

// Send ACK
#define APPCONF_NRF_SEND_CRC_ACK 1

// P
#define APPCONF_BALANCE_KP 0

// I
#define APPCONF_BALANCE_KI 0

// D
#define APPCONF_BALANCE_KD 0

// Loop Hertz
#define APPCONF_BALANCE_HERTZ 1000

// Pitch Axis Fault Cutoff
#define APPCONF_BALANCE_FAULT_PITCH 20

// Roll Axis Fault Cutoff
#define APPCONF_BALANCE_FAULT_ROLL 45

// Duty Cycle Fault Cutoff
#define APPCONF_BALANCE_FAULT_DUTY 0.9

// ADC1 Switch Voltage
#define APPCONF_BALANCE_FAULT_ADC1 0

// ADC2 Switch Voltage
#define APPCONF_BALANCE_FAULT_ADC2 0

// Pitch Fault Delay
#define APPCONF_BALANCE_FAULT_DELAY_PITCH 0

// Roll Fault Delay
#define APPCONF_BALANCE_FAULT_DELAY_ROLL 0

// Duty Fault Delay
#define APPCONF_BALANCE_FAULT_DELAY_DUTY 0

// Half Switch Fault Delay
#define APPCONF_BALANCE_FAULT_DELAY_SWITCH_HALF 0

// Full Switch Fault Delay
#define APPCONF_BALANCE_FAULT_DELAY_SWITCH_FULL 0

// ADC Half State Fault ERPM
#define APPCONF_BALANCE_FAULT_ADC_HALF_ERPM 1000

// Tiltback Angle
#define APPCONF_BALANCE_TILTBACK_ANGLE 15

// Tiltback Speed
#define APPCONF_BALANCE_TILTBACK_SPEED 5

// Duty Cycle Tiltback
#define APPCONF_BALANCE_TILTBACK_DUTY 0.75

// High Voltage Tiltback
#define APPCONF_BALANCE_TILTBACK_HIGH_V 200

// Low Voltage Tiltback
#define APPCONF_BALANCE_TILTBACK_LOW_V 0

// Constant Tiltback
#define APPCONF_BALANCE_TILTBACK_CONSTANT 0

// Constant Tiltback ERPM
#define APPCONF_BALANCE_TILTBACK_CONSTANT_ERPM 500

// Startup Pitch Axis Angle Tolerance
#define APPCONF_BALANCE_STARTUP_PITCH_TOLERANCE 20

// Startup Roll Axis Angle Tolerance
#define APPCONF_BALANCE_STARTUP_ROLL_TOLERANCE 8

// Startup Centering Speed
#define APPCONF_BALANCE_STARTUP_SPEED 30

// Deadzone
#define APPCONF_BALANCE_DEADZONE 0

// Current Boost
#define APPCONF_BALANCE_CURRENT_BOOST 0

// Multiple VESCs Over CAN
#define APPCONF_BALANCE_MULTI_ESC 0

// Yaw P
#define APPCONF_BALANCE_YAW_KP 0

// Yaw I
#define APPCONF_BALANCE_YAW_KI 0

// Yaw D
#define APPCONF_BALANCE_YAW_KD 0

// Roll Steer KP
#define APPCONF_BALANCE_ROLL_STEER_KP 0

// Roll Steer ERPM KP
#define APPCONF_BALANCE_ROLL_STEER_ERPM_KP 0

// Brake Current
#define APPCONF_BALANCE_BRAKE_CURRENT 0

// Yaw Current Clamp
#define APPCONF_BALANCE_YAW_CURRENT_CLAMP 0

// Setpoint Pitch Low Pass Filter
#define APPCONF_BALANCE_SETPOINT_PITCH_FILTER 0

// Setpoint Target Low Pass Filter
#define APPCONF_BALANCE_SETPOINT_TARGET_FILTER 1

// Setpoint Filter Clamp
#define APPCONF_BALANCE_SETPOINT_FILTER_CLAMP 8

// D term PT1 Filter
#define APPCONF_BALANCE_KD_PT1_FREQUENCY 0

// Control Type
#define APPCONF_PAS_CTRL_TYPE 0

// Sensor Type
#define APPCONF_PAS_SENSOR_TYPE 0

// PAS Max Current
#define APPCONF_PAS_CURRENT_SCALING 0.1

// Pedal RPM Start
#define APPCONF_PAS_PEDAL_RPM_START 10

// Pedal RPM End
#define APPCONF_PAS_PEDAL_RPM_END 180

// Invert Pedal Direction
#define APPCONF_PAS_INVERT_PEDAL_DIRECTION 0

// Sensor Magnets
#define APPCONF_PAS_MAGNETS 24

// Use Filter
#define APPCONF_PAS_USE_FILTER 1

// Positive Ramping Time
#define APPCONF_PAS_RAMP_TIME_POS 0.6

// Negative Ramping Time
#define APPCONF_PAS_RAMP_TIME_NEG 0.3

// Update Rate
#define APPCONF_PAS_UPDATE_RATE_HZ 500

// IMU Type
#define APPCONF_IMU_TYPE 1

// IMU AHRS Mode
#define APPCONF_IMU_AHRS_MODE 0

// Sample Rate
#define APPCONF_IMU_SAMPLE_RATE_HZ 200

// Accelerometer Confidence Decay
#define APPCONF_IMU_ACCEL_CONFIDENCE_DECAY 1

// Mahony KP
#define APPCONF_IMU_MAHONY_KP 0.3

// Mahony KI
#define APPCONF_IMU_MAHONY_KI 0

// Madgwick Beta
#define APPCONF_IMU_MADGWICK_BETA 0.1

// Imu Rotation Roll
#define APPCONF_IMU_ROT_ROLL 0

// Imu Rotation Pitch
#define APPCONF_IMU_ROT_PITCH 0

// Imu Rotation Yaw
#define APPCONF_IMU_ROT_YAW 0

// Accel Offset X
#define APPCONF_IMU_A_OFFSET_0 0

// Accel Offset Y
#define APPCONF_IMU_A_OFFSET_1 0

// Accel Offset Z
#define APPCONF_IMU_A_OFFSET_2 0

// Gyro Offset X
#define APPCONF_IMU_G_OFFSET_0 0

// Gyro Offset Y
#define APPCONF_IMU_G_OFFSET_1 0

// Gyro Offset Z
#define APPCONF_IMU_G_OFFSET_2 0

// Gyro Offset Comp X
#define APPCONF_IMU_G_OFFSET_COMP_FACT_0 0

// Gyro Offset Comp Y
#define APPCONF_IMU_G_OFFSET_COMP_FACT_1 0

// Gyro Offset Comp Z
#define APPCONF_IMU_G_OFFSET_COMP_FACT_2 0

// Gyro Offset Comp Clamp
#define APPCONF_IMU_G_OFFSET_COMP_CLAMP 5

// APP_H_
#endif

