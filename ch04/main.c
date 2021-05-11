/**
 * @file main.c
 * @brief joint trajectry planning for CRANE-X7
 * @author RT Corporation
 * @date 2019-2021
 * @copyright License: Apache License, Version 2.0
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

#include <stdio.h>
#include <unistd.h>
#include "../common/crane_x7_comm.h"
#include "myCX7_KDL_library.h"
#include "trajectry_planning.h"

#include <time.h> //  制御周期用のヘッダーファイル

#define LOOP_TIME 5000000 //制御ループの時間[ns] (200Hz)

int main()
{
  uint8_t operating_mode[JOINT_NUM] = {POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE};
  VECTOR_3D target_pos = {0};                 //目標位置格納用の変数
  VECTOR_3D target_pos1 = {0.10, 0.20, 0.15}; //目標位置1
  VECTOR_3D target_pos2 = {0.30, 0.20, 0.15}; //目標位置2
  VECTOR_3D target_pos3 = {0.30, -0.20, 0.15}; //目標位置3
  VECTOR_3D target_pos4 = {0.10, -0.20, 0.15};     //目標位置4
  //VECTOR_3D present_pos = {0};                //現在位置格納用の変数
  VECTOR_3D start_pos = {0}; //軌道の初期位置

  double start_angle[JOINT_NUM] = {0};    //初期角度格納用の変数
  double target_angle[JOINT_NUM] = {0};   //目標角度格納用の変数
  double present_angle[JOINT_NUM] = {0};  //現在角度を格納する変数
  double present_angvel[JOINT_NUM] = {0}; //現在速度を格納する変数
  double present_torque[JOINT_NUM] = {0}; //現在トルクを格納する変数

  int state = 0;    //目標位置を切り替えるための変数
  int cnt = 0;      //ループのカウント
  int time_cnt = 0; //ループのカウント

  //軌道を格納する配列
  double trajectry_angle[MAX_LENGTH][JOINT_NUM] = {0};
  double trajectry_angvel[MAX_LENGTH][JOINT_NUM] = {0};
  double trajectry_angacc[MAX_LENGTH][JOINT_NUM] = {0};
  double v_max = 0.6;
  double dt = LOOP_TIME / (1e9); //制御周期
  double T = 2.0;

  //時間管理用変数
  struct timespec start_time = {0}, end_time = {0}, sleep_time = {0}, duration_time = {0};

  printf("Press any key to start (or press q to quit)\n");
  if (getchar() == ('q'))
    return 0;

  // 運動学ライブラリの初期化
  initParam();

  // サーボ関連の設定の初期化
  if (initilizeCranex7(operating_mode))
  {
    return 1;
  }
  // CRANE-X7のトルクON
  setCranex7TorqueEnable(TORQUE_ENABLE);

  target_pos = target_pos1;

  getCranex7JointState(present_angle, present_angvel, present_torque);
  forwardKinematics3Dof(&start_pos, present_angle);

  // main関数のループ
  while (cnt < 9)
  { //10回繰り返したらプログラムを終了する
    cnt++;

    if (inverseKinematics3Dof(start_pos, start_angle))
    {
      break;
    }
    if (inverseKinematics3Dof(target_pos, target_angle))
    {
      break;
    }
    trapezoidalTrajectoryJointSpace(start_angle, target_angle, v_max, T, dt, trajectry_angle, trajectry_angvel, trajectry_angacc);

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    time_cnt = 0;
    do
    {
      ////////////  ↓一定時間処理したい内容をここに書く  ////////////

      setCranex7Angle(trajectry_angle[time_cnt]);

      ////////////  ↑一定時間処理したい内容をここに書く  ////////////
      //周期がLOOP_TIME [ns]となるようにnanosleepで調整する処理
      clock_gettime(CLOCK_MONOTONIC, &end_time);
      if (end_time.tv_nsec < start_time.tv_nsec)
      {
        duration_time.tv_nsec = end_time.tv_nsec + 1000000000 - start_time.tv_nsec;
      }
      else
      {
        duration_time.tv_nsec = end_time.tv_nsec - start_time.tv_nsec;
      }
      //LOOP_TIMEより短かったらnanosleepで時間調整
      if (duration_time.tv_nsec < LOOP_TIME)
      {
        sleep_time.tv_nsec = LOOP_TIME - duration_time.tv_nsec;
        //printf("sleep time %ld.%09ld\n", sleep_time.tv_sec, sleep_time.tv_nsec);
        nanosleep(&sleep_time, NULL);
      }
      start_time.tv_nsec = start_time.tv_nsec + LOOP_TIME;
      if (start_time.tv_nsec > 1000000000)
      {
        start_time.tv_nsec = start_time.tv_nsec - 1000000000;
      }
      time_cnt++;
    } while (time_cnt < (T / dt)); //end do while

    //target_angleを変更
    if (state == 0)
    {
      state = 1;
      start_pos = target_pos;
      target_pos = target_pos2;
    }
    else if (state == 1)
    {
      state = 2;
      start_pos = target_pos;
      target_pos = target_pos3;
    }
    else if (state == 2)
    {
      state = 3;
      start_pos = target_pos;
      target_pos = target_pos4;
    }
    else
    {
      state = 0;
      start_pos = target_pos;
      target_pos = target_pos1;
    }
  } //end main while

  brakeCranex7Joint(); //CRANE X7をブレーキにして終了
  closeCranex7Port();  //シリアルポートを閉じる
  return 0;
}
