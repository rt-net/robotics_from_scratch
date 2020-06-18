/**
 * @file crane_x7_comm.c
 * @brief Wrapper functions of DynamixelSDK (for communicating with the CRANE-X7)
 * @author RT Corporation
 * @date 2020/03/21
 */
// Copyright 2020 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//// Header files ////
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include "dynamixel_sdk.h"
#include "crane_x7_comm.h"

//// Unique set values of each servo motor ////
static const uint8_t id_array[JOINT_NUM] = {2, 3, 4, 5, 6, 7, 8, 9};                                  // ID (a unique value to identify each servo motor)
static const uint32_t min_angle_array[JOINT_NUM] = {262, 1024, 262, 228, 262, 1024, 148, 1991};       // Min angle (expressed as raw value of the dynamixel motor)
static const uint32_t max_angle_array[JOINT_NUM] = {3834, 3072, 3834, 2048, 3834, 3072, 3928, 3072};  // Max angle (expressed as raw value of the dynamixel motor)
static const uint32_t home_angle_array[JOINT_NUM] = {2048, 1024, 2048, 2048, 2048, 2048, 2048, 2048}; // Values at 0 radian posture (expressed as raw value of the dynamixel motor)

//// Variable for DynamixelSDK ////
static int port_num = 0;                // PortHandler Structs number
static int groupwrite_num = 0;          // Groupbulkwrite Struct number
static int groupread_num = 0;           // Groupbulkread Struct number
static int comm_result = COMM_TX_FAIL;  // Communication result
static uint8_t addparam_result = False; // AddParam result
static uint8_t getdata_result = False;  // GetParam result
static uint8_t dxl_error;               // Dynamixel error

//// Unit convertion functions for dynamixel ////

/**
 * @fn static double rad2dxlvalue(double)
 * @brief Angle unit conversion function from rad to dynamixel value
 * @param[in] rad :angle[rad/s]
 * @return value :angle[dynamixel value]
 */
static double rad2dxlvalue(double rad)
{
  double value = rad / (DXL_VALUE_TO_RADIAN);
  return value;
}

/**
 * @fn static double dxlvalue2rad(double)
 * @brief Angle unit conversion function from dynamixel value to rad
 * @param[in] value :angle[dynamixel value]
 * @return rad :angle[rad/s]
 */
static double dxlvalue2rad(double value)
{
  double rad = value * (DXL_VALUE_TO_RADIAN);
  return rad;
}

/**
 * @fn static double angularvel2dxlvalue(double)
 * @brief Anglular velocity unit conversion function from rad/s to dynamixel value
 * @param[in] angular_velocity :anglular velocity[rad/s]
 * @return value :anglular velocity[dynamixel value]
 */
static double angularvel2dxlvalue(double angular_velocity)
{
  double value = angular_velocity / (DXL_VALUE_TO_ANGULARVEL);
  return value;
}

/**
 * @fn static double dxlvalue2angularvel(double)
 * @brief Anglular velocity unit conversion function from dynamixel value to rad/s
 * @param[in] value :anglular velocity[dynamixel value]
 * @return angular_velocity :anglular velocity[rad/s]
 */
static double dxlvalue2angularvel(double value)
{
  double angular_velocity = value * (DXL_VALUE_TO_ANGULARVEL);
  return angular_velocity;
}

/**
 * @fn static double current2dxlvalue(double)
 * @brief Current unit conversion function from A to dynamixel value
 * @param[in] current :current[A]
 * @return value :current[dynamixel value]
 */
static double current2dxlvalue(double current)
{
  double value = current / (DXL_VALUE_TO_CURRENT);
  return value;
}

/**
 * @fn static double dxlvalue2current(double)
 * @brief Current unit conversion function from dynamixel value to A
 * @param[in] value :current[dynamixel value]
 * @return current :current[A]
 */
static double dxlvalue2current(double value)
{
  double current = value * (DXL_VALUE_TO_CURRENT);
  return current;
}

/**
 * @fn static double current2torqueXM430W350(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM430W350
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
static double current2torqueXM430W350(double current)
{
  double torque = current * (CURRENT_TO_TORQUE_XM430W350);
  return torque;
}

/**
 * @fn static double current2torqueXM540W270(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM540W270
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
static double current2torqueXM540W270(double current)
{
  double torque = current * (CURRENT_TO_TORQUE_XM540W270);
  return torque;
}

/**
 * @fn static double torque2currentXM430W350(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM430W350
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
static double torque2currentXM430W350(double torque)
{
  double current = torque / (CURRENT_TO_TORQUE_XM430W350);
  return current;
}

/**
 * @fn static double torque2currentXM540W270(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM540W270
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
static double torque2currentXM540W270(double torque)
{
  double current = torque / (CURRENT_TO_TORQUE_XM540W270);
  return current;
}

//// Communication functions for CRANE-X7 ////

/**
 * @fn int initilizeCranex7(uint8_t *)
 * @brief Initilizetion function of CRANE-X7
 * @param[in] *operationg_mode An array containing the operating modes of each servo motor.
 * @return Success or failure of initilizetion.
 */
int initilizeCranex7(uint8_t *operating_mode_array)
{
  port_num = portHandler(SERIAL_PORT);                         // Initialize PortHandler Structs
  packetHandler();                                             // Initialize PacketHandler Structs
  groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION); // Initialize PortHandler Structs
  //groupread_num = 0;

  // open serial port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port.\n");
    // set baudrate
    if (setBaudRate(port_num, BAUDRATE))
    {
      printf("Succeeded to change the baudrate.\n");
    }
    else
    {
      printf("Failed to change the baudrate.\n");
      return 1;
    }
  }
  else
  {
    printf("Failed to open the port.\n");
    return 1;
  }
  // Turn off the torque to change operating mode
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], TORQUE_ENABLE_ADDRESS, TORQUE_DISABLE);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
      return 1;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      return 1;
    }
    else
    {
      printf("DXL#%d has been successfully connected.\n", id_array[i]);
    }
  }
  // Set operating mode
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], OPERATING_MODE_ADDRESS, (uint8_t)operating_mode_array[i]);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
      return 1;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      return 1;
    }
    else
    {
      printf("Operationg mode of DXL#%d has been successfully configured.\n", id_array[i]);
    }
  }
  // Set position p gain to the defalut value
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_P_GAIN_ADDRESS, (uint16_t)DEFAULT_POSITION_P_GAIN);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // Set position i gain to the defalut value
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_I_GAIN_ADDRESS, (uint16_t)DEFAULT_POSITION_I_GAIN);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // Set position d gain to the defalut value
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_D_GAIN_ADDRESS, (uint16_t)DEFAULT_POSITION_D_GAIN);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // Set velocity p gain to the defalut value
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], VELOCITY_P_GAIN_ADDRESS, (uint16_t)DEFAULT_VELOCITY_P_GAIN);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // Set velocity i gain to the defalut value
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], VELOCITY_I_GAIN_ADDRESS, (uint16_t)DEFAULT_VELOCITY_I_GAIN);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // Set velocity profile
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], PROFILE_VELOCITY_ADDRESS, (uint32_t)PROFILE_VELOCITY);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  return 0;
}

/**
 * @fn void setCranex7TorqueEnable(uint8_t)
 * @brief Function to enable (or disable) servo motor torque
 * @param[in] torque_enable 1:enable, 0:disable
 * @return Success or failure of enabling.
 */
int setCranex7TorqueEnable(uint8_t torque_enable)
{
  // Set torque enable
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], TORQUE_ENABLE_ADDRESS, torque_enable);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
      return 1;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      return 1;
    }
    else
    {
      if (torque_enable)
      {
        printf("Turn on DXL#%d torque \n", id_array[i]);
      }
      else
      {
        printf("Turn off DXL#%d torque \n", id_array[i]);
      }
    }
  }
  return 0;
}

/**
 * @fn int setCranex7Angle(double)
 * @brief Function to set command angle
 * @param[in] angle_array[] command angle array
 * @return Success or failure.
 */
int setCranex7Angle(double *angle_array)
{
  int32_t goal_position[JOINT_NUM] = {0};

  for (int i = 0; i < JOINT_NUM; i++)
  {
    goal_position[i] = (int32_t)(rad2dxlvalue(angle_array[i])) + home_angle_array[i];
    if ((goal_position[i] > max_angle_array[i]) || (min_angle_array[i]) > goal_position[i])
    {
      printf("Out of angle range : joint %d \n", i + 1);
    }
  }
  // set goal position date to bulk write parameter
  for (int i = 0; i < JOINT_NUM; i++)
  {
    addparam_result = groupBulkWriteAddParam(groupwrite_num, id_array[i], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, goal_position[i], GOAL_POSITION_DATA_LENGTH);
    if (addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] parameter set failed", id_array[i]);
      return 1;
    }
  }
  // transmit goal position data
  groupBulkWriteTxPacket(groupwrite_num);
  if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
  // clear transmittion data
  groupBulkWriteClearParam(groupwrite_num);
  return 0;
}

/**
 * @fn int setCranex7AngularVelocity(double)
 * @brief Function to set command anglular velocity
 * @param[in] angular_velocity_array[] command anglular velocity array
 * @return Success or failure.
 */
int setCranex7AngularVelocity(double *angular_velocity_array)
{
  int32_t goal_velocity[JOINT_NUM] = {0};

  for (int i = 0; i < JOINT_NUM; i++)
  {
    goal_velocity[i] = (int32_t)(angularvel2dxlvalue(angular_velocity_array[i]));
  }
  // set goal velosity data to bulk write parameter
  for (int i = 0; i < JOINT_NUM; i++)
  {
    addparam_result = groupBulkWriteAddParam(groupwrite_num, id_array[i], GOAL_VELOCITY_ADDRESS, GOAL_VELOCITY_DATA_LENGTH, goal_velocity[i], GOAL_VELOCITY_DATA_LENGTH);
    if (addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] parameter set failed", id_array[i]);
      return 1;
    }
  }
  // transmit goal velosity data
  groupBulkWriteTxPacket(groupwrite_num);
  if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
  // clear transmitton data
  groupBulkWriteClearParam(groupwrite_num);
  return 0;
}

/**
 * @fn int setCranex7Torque(double)
 * @brief Function to set command torque
 * @param[in] torque_array[] command torque array
 * @return Success or failure.
 */
int setCranex7Torque(double *torque_array)
{
  int16_t goal_current[JOINT_NUM] = {0};

  // convert torque to currrent
  for (int i = 0; i < JOINT_NUM; i++)
  {
    if (i == XM540_W270_JOINT)
    {
      goal_current[i] = (int16_t)current2dxlvalue(torque2currentXM540W270(torque_array[i]));
    }
    else
    {
      goal_current[i] = (int16_t)current2dxlvalue(torque2currentXM430W350(torque_array[i]));
    }
  }
  // set goal current to bulk write parameter
  for (int i = 0; i < JOINT_NUM; i++)
  {
    addparam_result = groupBulkWriteAddParam(groupwrite_num, id_array[i], GOAL_CURRENT_ADDRESS, GOAL_CURRENT_DATA_LENGTH, goal_current[i], GOAL_CURRENT_DATA_LENGTH);
    if (addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] parameter set failed", id_array[i]);
      return 1;
    }
  }
  // transmit goal current
  groupBulkWriteTxPacket(groupwrite_num);
  if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
  // clear transmittion data
  groupBulkWriteClearParam(groupwrite_num);
  return 0;
}

/**
 * @fn int getCranex7JointState(double *, double *, double *)
 * @brief Function to get joint state
 * @param[out] angle_array[] present angle array
 * @param[out] angular_velocity_array[] present angular velocity array
 * @param[out] torque_array[] present torque array
 * @return Success or failure.
 */
int getCranex7JointState(double *angle_array, double *angular_velocity_array, double *torque_array)
{
  int32_t present_position[JOINT_NUM] = {0};
  int16_t present_velocity[JOINT_NUM] = {0};
  int16_t present_current[JOINT_NUM] = {0};

  groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);
  // set bulk read parameter (present positon, present velosity, present current)
  for (int i = 0; i < JOINT_NUM; i++)
  {
    addparam_result = groupBulkReadAddParam(groupread_num, id_array[i], PRESENT_VALUE_ADDRESS, PRESENT_VALUE_DATA_LENGTH);
    if (addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed", id_array[i]);
      return 1;
    }
  }
  // data request and receive
  groupBulkReadTxRxPacket(groupread_num);
  if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));

  // verification of received data
  for (int i = 0; i < JOINT_NUM; i++)
  {
    getdata_result = groupBulkReadIsAvailable(groupread_num, id_array[i], PRESENT_VALUE_ADDRESS, PRESENT_VALUE_DATA_LENGTH);
    if (getdata_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupBulkRead getdata trq failed", id_array[i]);
      return 1;
    }
  }
  // pick up present position data
  for (int i = 0; i < JOINT_NUM; i++)
  {
    present_position[i] = groupBulkReadGetData(groupread_num, id_array[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
  }
  // pick up present velocity data
  for (int i = 0; i < JOINT_NUM; i++)
  {
    present_velocity[i] = groupBulkReadGetData(groupread_num, id_array[i], PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_DATA_LENGTH);
  }
  // pick up present current data
  for (int i = 0; i < JOINT_NUM; i++)
  {
    present_current[i] = groupBulkReadGetData(groupread_num, id_array[i], PRESENT_CURRENT_ADDRESS, PRESENT_CURRENT_DATA_LENGTH);
  }

  // convert dynamixel value to physical quantity
  for (int i = 0; i < JOINT_NUM; i++)
  {
    angle_array[i] = dxlvalue2rad((double)(present_position[i] - home_angle_array[i]));
    angular_velocity_array[i] = dxlvalue2angularvel((double)present_velocity[i]);
    if (i == XM540_W270_JOINT)
    {
      torque_array[i] = current2torqueXM540W270(dxlvalue2current((double)present_current[i]));
    }
    else
    {
      torque_array[i] = current2torqueXM430W350(dxlvalue2current((double)present_current[i]));
    }
  }
  return 0;
}

/**
 * @fn void closeCranex7Port(void)
 * @brief Close port
 */
void closeCranex7Port(void)
{
  // Close port
  closePort(port_num);
  printf("close com port\n");
}

/**
 * @fn void brakeCranex7Joint(void)
 * @brief Brake joints
 */
void brakeCranex7Joint(void)
{

  //// set position feedback gain to 0 then joints act like braking (if position control mode).
  // set position d gain to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_D_GAIN_ADDRESS, (uint16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // set position i gain to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_I_GAIN_ADDRESS, (uint16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // set position p gain to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], POSITION_P_GAIN_ADDRESS, (uint16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  //// set velocity feedback gain to 0 then joints act like braking (if velocity control mode).
  // set velocity i gain to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], VELOCITY_I_GAIN_ADDRESS, (uint16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  // set velocity p gain to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], VELOCITY_P_GAIN_ADDRESS, (uint16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
  //// set goal current to 0 then joints act like braking (if current control mode).
  // set goal current to 0
  for (int i = 0; i < JOINT_NUM; i++)
  {
    write2ByteTxRx(port_num, PROTOCOL_VERSION, id_array[i], GOAL_CURRENT_ADDRESS, (int16_t)0);
    if ((comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
  }
}
