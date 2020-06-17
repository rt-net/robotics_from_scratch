/**
 * @file crane_x7_control.h
 * @brief Wrapper functions of DynamixelSDK
 * @author RT Corporation
 * @date 2020/03/21
 */

#ifndef CRANE_X7_CONTROL_H_
#define CRANE_X7_CONTROL_H_

#include <stdint.h>

//// Definition of dynamixel ////

// Data address of dynamixel x
#define OPERATING_MODE_ADDRESS (11)
#define TORQUE_ENABLE_ADDRESS (64)
#define VELOCITY_I_GAIN_ADDRESS (76)
#define VELOCITY_P_GAIN_ADDRESS (78)
#define POSITION_D_GAIN_ADDRESS (80)
#define POSITION_I_GAIN_ADDRESS (82)
#define POSITION_P_GAIN_ADDRESS (84)
#define BUS_WATCHDOG_ADDRESS (98)
#define GOAL_CURRENT_ADDRESS (102)
#define GOAL_VELOCITY_ADDRESS (104)
#define PROFILE_VELOCITY_ADDRESS (112)
#define GOAL_POSITION_ADDRESS (116)
#define PRESENT_CURRENT_ADDRESS (126)
#define PRESENT_VELOCITY_ADDRESS (128)
#define PRESENT_POSITION_ADDRESS (132)
#define PRESENT_VALUE_ADDRESS (126)

// Data length
#define BUS_WATCHDOG_DATA_LENGTH (1)
#define GOAL_POSITION_DATA_LENGTH (4)
#define PRESENT_POSITION_DATA_LENGTH (4)
#define GOAL_VELOCITY_DATA_LENGTH (4)
#define PRESENT_VELOCITY_DATA_LENGTH (4)
#define GOAL_CURRENT_DATA_LENGTH (2)
#define PRESENT_CURRENT_DATA_LENGTH (2)
#define PRESENT_VALUE_DATA_LENGTH (10)
#define PROFILE_VELOCITY_DATA_LENGTH (4)
// Protocol version
#define PROTOCOL_VERSION (2.0)

// Control value
#define TORQUE_ENABLE (1)
#define TORQUE_DISABLE (0)
#define CURRENT_CONTROL_MODE (0)
#define VELOCITY_CONTROL_MODE (1)
#define POSITION_CONTROL_MODE (3)
#define DEFAULT_POSITION_P_GAIN (800)
#define DEFAULT_POSITION_I_GAIN (0)
#define DEFAULT_POSITION_D_GAIN (0)
#define DEFAULT_VELOCITY_P_GAIN (100)
#define DEFAULT_VELOCITY_I_GAIN (1920)
#define PROFILE_VELOCITY (60)
// Serial port setting
#define BAUDRATE (3000000)
#define SERIAL_PORT "/dev/ttyUSB0" // Check the port which crane-x7 is conected

//// Definition of crane-x7 ////
#define XM540_W270_JOINT (1) // only 2nd joint servo motor is XM540_W270 (other XM430_W350)
#ifndef JOINT_NUM
#define JOINT_NUM (8)
#endif
#ifndef PI
#define PI (3.14159265)
#endif

// Unit conversion
#define DXL_VALUE_TO_RADIAN ((2 * PI) / 4096)
#define DXL_VALUE_TO_ANGULARVEL ((0.229 * 2 * PI) / 60)
#define DXL_VALUE_TO_CURRENT (0.00269)
#define TORQUE_CORRECTION_FACTOR (1.3)
#define CURRENT_TO_TORQUE_XM430W350 (1.783 * TORQUE_CORRECTION_FACTOR)
#define CURRENT_TO_TORQUE_XM540W270 (2.409 * TORQUE_CORRECTION_FACTOR)

//// Prototype declaration ////
int initilizeCranex7(uint8_t *);
int setCranex7TorqueEnable(uint8_t);
int setCranex7Angle(double *);
int setCranex7AngularVelocity(double *);
int setCranex7Torque(double *);
int getCranex7JointState(double *, double *, double *);
void brakeCranex7Joint(void);
void closeCranex7Port(void);

#endif