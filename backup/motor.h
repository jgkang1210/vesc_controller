// This file is autogenerated by VESC Tool

#ifndef MOTOR_H_
#define MOTOR_H_

// PWM Mode
#define MCCONF_PWM_MODE 1

// Commutation Mode
#define MCCONF_COMM_MODE 0

// Motor Type
#define MCCONF_DEFAULT_MOTOR_TYPE 2

// Sensor Mode
#define MCCONF_SENSOR_MODE 0

// Motor Current Max
#define MCCONF_L_CURRENT_MAX 35.8288

// Motor Current Max Brake
#define MCCONF_L_CURRENT_MIN -35.8288

// Battery Current Max
#define MCCONF_L_IN_CURRENT_MAX 99

// Battery Current Max Regen
#define MCCONF_L_IN_CURRENT_MIN -60

// Absolute Maximum Current
#define MCCONF_L_MAX_ABS_CURRENT 130

// Max ERPM Reverse
#define MCCONF_L_RPM_MIN -100000

// Max ERPM
#define MCCONF_L_RPM_MAX 100000

// ERPM Limit Start
#define MCCONF_L_RPM_START 0.8

// Max ERPM Full Brake
#define MCCONF_L_CURR_MAX_RPM_FBRAKE 300

// Max ERPM Full Brake Current Control
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC 1500

// Minimum Input Voltage
#define MCCONF_L_MIN_VOLTAGE 8

// Maximum Input Voltage
#define MCCONF_L_MAX_VOLTAGE 57

// Battery Voltage Cutoff Start
#define MCCONF_L_BATTERY_CUT_START 27.2

// Battery Voltage Cutoff End
#define MCCONF_L_BATTERY_CUT_END 24

// Slow ABS Current Limit
#define MCCONF_L_SLOW_ABS_OVERCURRENT 1

// MOSFET Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_FET_START 85

// MOSFET Temp Cutoff End
#define MCCONF_L_LIM_TEMP_FET_END 100

// Motor Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_MOTOR_START 85

// Motor Temp Cutoff End
#define MCCONF_L_LIM_TEMP_MOTOR_END 100

// Acceleration Temperature Decrease
#define MCCONF_L_LIM_TEMP_ACCEL_DEC 0.15

// Minimum Duty Cycle
#define MCCONF_L_MIN_DUTY 0.005

// Maximum Duty Cycle
#define MCCONF_L_MAX_DUTY 0.95

// Maximum Wattage
#define MCCONF_L_WATT_MAX 1.5e+06

// Maximum Braking Wattage
#define MCCONF_L_WATT_MIN -1.5e+06

// Max Current Scale
#define MCCONF_L_CURRENT_MAX_SCALE 1

// Min Current Scale
#define MCCONF_L_CURRENT_MIN_SCALE 1

// Duty Cycle Current Limit Start
#define MCCONF_L_DUTY_START 1

// Minimum ERPM
#define MCCONF_SL_MIN_RPM 150

// Minimum ERPM Integrator
#define MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT 1100

// Max Brake Current at Direction Change
#define MCCONF_SL_MAX_FB_CURR_DIR_CHANGE 10

// Cycle Integrator Limit
#define MCCONF_SL_CYCLE_INT_LIMIT 62

// Phase Advance at BR ERPM
#define MCCONF_SL_PHASE_ADVANCE_AT_BR 0.8

// BR ERPM
#define MCCONF_SL_CYCLE_INT_BR 80000

// BEMF Coupling
#define MCCONF_SL_BEMF_COUPLING_K 600

// Hall Table [0]
#define MCCONF_HALL_TAB_0 -1

// Hall Table [1]
#define MCCONF_HALL_TAB_1 1

// Hall Table [2]
#define MCCONF_HALL_TAB_2 3

// Hall Table [3]
#define MCCONF_HALL_TAB_3 2

// Hall Table [4]
#define MCCONF_HALL_TAB_4 5

// Hall Table [5]
#define MCCONF_HALL_TAB_5 6

// Hall Table [6]
#define MCCONF_HALL_TAB_6 4

// Hall Table [7]
#define MCCONF_HALL_TAB_7 -1

// Sensorless ERPM Hybrid
#define MCCONF_HALL_ERPM 2000

// Current KP
#define MCCONF_FOC_CURRENT_KP 0.0144

// Current KI
#define MCCONF_FOC_CURRENT_KI 49.22

// Switching Frequency
#define MCCONF_FOC_F_SW 25000

// Dead Time Compensation
#define MCCONF_FOC_DT_US 0.12

// Encoder Inverted
#define MCCONF_FOC_ENCODER_INVERTED 1

// Encoder Offset
#define MCCONF_FOC_ENCODER_OFFSET 250.3

// Encoder Ratio
#define MCCONF_FOC_ENCODER_RATIO 20

// Sin/Cos Sine Gain Compensation
#define MCCONF_FOC_ENCODER_SIN_GAIN 1

// Sin/Cos Cosine Gain Compensation
#define MCCONF_FOC_ENCODER_COS_GAIN 1

// Sin/Cos Sine Offset
#define MCCONF_FOC_ENCODER_SIN_OFFSET 1.65

// Sin/Cos Cosine Offset
#define MCCONF_FOC_ENCODER_COS_OFFSET 1.65

// Sin/Cos Filter Constant
#define MCCONF_FOC_ENCODER_SINCOS_FILTER 0.5

// Sensor Mode
#define MCCONF_FOC_SENSOR_MODE 1

// Speed Tracker Kp
#define MCCONF_FOC_PLL_KP 2000

// Speed Tracker Ki
#define MCCONF_FOC_PLL_KI 30000

// Motor Inductance (L)
#define MCCONF_FOC_MOTOR_L 1.442e-05

// Motor Inductance Difference (Ld - Lq)
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF 0

// Motor Resistance (R)
#define MCCONF_FOC_MOTOR_R 0.04922

// Motor Flux Linkage (λ)
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE 0.003448

// Observer Gain (x1M)
#define MCCONF_FOC_OBSERVER_GAIN 8.411e+07

// Observer Gain At Minimum Duty
#define MCCONF_FOC_OBSERVER_GAIN_SLOW 0.05

// Duty Downramp Kp
#define MCCONF_FOC_DUTY_DOWNRAMP_KP 10

// Duty Downramp Ki
#define MCCONF_FOC_DUTY_DOWNRAMP_KI 200

// Openloop ERPM
#define MCCONF_FOC_OPENLOOP_RPM 700

// Openloop ERPM at Min Current
#define MCCONF_FOC_OPENLOOP_RPM_LOW 0

// D Axis Gain Scaling Start
#define MCCONF_FOC_D_GAIN_SCALE_START 0.9

// D Axis Gain Scaling at Max Mod
#define MCCONF_FOC_D_GAIN_SCALE_MAX_MOD 0.2

// Openloop Hysteresis
#define MCCONF_FOC_SL_OPENLOOP_HYST 0.1

// Openloop Lock Time
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK 0

// Openloop Ramp Time
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP 0.1

// Openloop Time
#define MCCONF_FOC_SL_OPENLOOP_TIME 0.05

// Hall Table [0]
#define MCCONF_FOC_HALL_TAB_0 255

// Hall Table [1]
#define MCCONF_FOC_HALL_TAB_1 255

// Hall Table [2]
#define MCCONF_FOC_HALL_TAB_2 255

// Hall Table [3]
#define MCCONF_FOC_HALL_TAB_3 255

// Hall Table [4]
#define MCCONF_FOC_HALL_TAB_4 255

// Hall Table [5]
#define MCCONF_FOC_HALL_TAB_5 255

// Hall Table [6]
#define MCCONF_FOC_HALL_TAB_6 255

// Hall Table [7]
#define MCCONF_FOC_HALL_TAB_7 255

// Hall Interpolation ERPM
#define MCCONF_FOC_HALL_INTERP_ERPM 500

// Sensorless ERPM
#define MCCONF_FOC_SL_ERPM 4000

// Sample in V0 and V7
#define MCCONF_FOC_SAMPLE_V0_V7 0

// High Current Sampling Mode
#define MCCONF_FOC_SAMPLE_HIGH_CURRENT 0

// Stator Saturation Compensation
#define MCCONF_FOC_SAT_COMP 0

// Temp Comp
#define MCCONF_FOC_TEMP_COMP 0

// Temp Comp Base Temp
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP 25

// Current Filter Constant
#define MCCONF_FOC_CURRENT_FILTER_CONST 0.1

// Current Controller Decoupling
#define MCCONF_FOC_CC_DECOUPLING 2

// Observer Type
#define MCCONF_FOC_OBSERVER_TYPE 0

// HFI Start Voltage
#define MCCONF_FOC_HFI_VOLTAGE_START 20

// HFI Run Voltage
#define MCCONF_FOC_HFI_VOLTAGE_RUN 4

// HFI Max Voltage
#define MCCONF_FOC_HFI_VOLTAGE_MAX 10

// Sensorless ERPM HFI
#define MCCONF_FOC_SL_ERPM_HFI 2000

// HFI Start Samples
#define MCCONF_FOC_HFI_START_SAMPLES 65

// HFI Observer Override Time
#define MCCONF_FOC_HFI_OBS_OVR_SEC 0.001

// HFI Samples
#define MCCONF_FOC_HFI_SAMPLES 1

// Buffer Notification Length
#define MCCONF_GPD_BUFFER_NOTIFY_LEFT 200

// Buffer Sampling Interpolation
#define MCCONF_GPD_BUFFER_INTERPOL 0

// Current Filter Constant
#define MCCONF_GPD_CURRENT_FILTER_CONST 0.1

// Current KP
#define MCCONF_GPD_CURRENT_KP 0.03

// Current KI
#define MCCONF_GPD_CURRENT_KI 50

// Speed PID Kp
#define MCCONF_S_PID_KP 0.004

// Speed PID Ki
#define MCCONF_S_PID_KI 0.004

// Speed PID Kd
#define MCCONF_S_PID_KD 0.0001

// Speed PID Kd Filer
#define MCCONF_S_PID_KD_FILTER 0.2

// Minimum ERPM
#define MCCONF_S_PID_MIN_RPM 900

// Allow Braking
#define MCCONF_S_PID_ALLOW_BRAKING 1

// Ramp eRPMs per second
#define MCCONF_S_PID_RAMP_ERPMS_S -1

// Position PID Kp
#define MCCONF_P_PID_KP 0.03

// Position PID Ki
#define MCCONF_P_PID_KI 0

// Position PID Kd
#define MCCONF_P_PID_KD 0.0004

// Position PID Kd Filer
#define MCCONF_P_PID_KD_FILTER 0.2

// Position Angle Division
#define MCCONF_P_PID_ANG_DIV 1

// Startup boost
#define MCCONF_CC_STARTUP_BOOST_DUTY 0.01

// Minimum Current
#define MCCONF_CC_MIN_CURRENT 0.05

// Current Controller Gain
#define MCCONF_CC_GAIN 0.0046

// Current Control Ramp Step Max
#define MCCONF_CC_RAMP_STEP 0.04

// Fault Stop Time
#define MCCONF_M_FAULT_STOP_TIME 500

// Duty Ramp Step Max
#define MCCONF_M_RAMP_STEP 0.02

// Current Backoff Gain
#define MCCONF_M_CURRENT_BACKOFF_GAIN 0.5

// ABI Encoder Counts
#define MCCONF_M_ENCODER_COUNTS 8192

// Sensor Port Mode
#define MCCONF_M_SENSOR_PORT_MODE 2

// Invert Motor Direction
#define MCCONF_M_INVERT_DIRECTION 0

// DRV8301 OC Mode
#define MCCONF_M_DRV8301_OC_MODE 0

// DRV8301 OC Adjustment
#define MCCONF_M_DRV8301_OC_ADJ 16

// Minimum Switching Frequency
#define MCCONF_M_BLDC_F_SW_MIN 3000

// Maximum Switching Frequency
#define MCCONF_M_BLDC_F_SW_MAX 35000

// Switching Frequency
#define MCCONF_M_DC_F_SW 25000

// Beta Value for Motor Thermistor
#define MCCONF_M_NTC_MOTOR_BETA 3380

// Auxiliary Output Mode
#define MCCONF_M_OUT_AUX_MODE 0

// Motor Temperature Sensor Type
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE 0

// Coefficient for PTC Motor Thermistor
#define MCCONF_M_PTC_MOTOR_COEFF 0.61

// Hall Sensor Extra Samples
#define MCCONF_M_HALL_EXTRA_SAMPLES 1

// Motor Poles
#define MCCONF_SI_MOTOR_POLES 40

// Gear Ratio
#define MCCONF_SI_GEAR_RATIO 2.76923

// Wheel Diameter
#define MCCONF_SI_WHEEL_DIAMETER 0.083

// Battery Type
#define MCCONF_SI_BATTERY_TYPE 0

// Battery Cells Series
#define MCCONF_SI_BATTERY_CELLS 8

// Battery Capacity
#define MCCONF_SI_BATTERY_AH 6

// BMS Type
#define MCCONF_BMS_TYPE 1

// Temperature Limit Start
#define MCCONF_BMS_T_LIMIT_START 45

// Temperature Limit End
#define MCCONF_BMS_T_LIMIT_END 65

// SOC Limit Start
#define MCCONF_BMS_SOC_LIMIT_START 0.05

// SOC Limit End
#define MCCONF_BMS_SOC_LIMIT_END 0

// MOTOR_H_
#endif

