/**
 * @file main.c
 * @brief servo test
 * @author RT Corporation
 * @date 2019-2020
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

int main()
{
  uint8_t operating_mode[JOINT_NUM] = {POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE};
  VECTOR_3D target_pos = {0};              //目標位置格納用の変数
  VECTOR_3D target_pos1 = {0.25, 0.15, 0}; //目標位置1
  VECTOR_3D target_pos2 = {0.35, 0.15, 0}; //目標位置2
  VECTOR_3D target_pos3 = {0.35, 0.25, 0}; //目標位置3
  VECTOR_3D target_pos4 = {0.25, 0.25, 0}; //目標位置4
  VECTOR_3D present_pos = {0};             //現在位置格納用の変数

  double target_theta[JOINT_NUM] = {0};    //目標角度格納用の変数
  double present_theta[JOINT_NUM] = {0};   //現在角度を格納する変数
  double present_angvel[JOINT_NUM] = {0};  //現在速度を格納する変数
  double present_current[JOINT_NUM] = {0}; //現在トルクを格納する変数

  int state = 0; //目標位置を切り替えるための変数
  int cnt = 0;   //ループのカウント

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

  // main関数のループ
  while (cnt < 9)
  { //10回繰り返したらプログラムを終了する
    cnt++;

    if (inverseKinematics2Dof(target_pos, target_theta))
    {
      break;
    }
    setCranex7Angle(target_theta);

    sleep(3);

    getCranex7JointState(present_theta, present_angvel, present_current);
    forwardKinematics2Dof(&present_pos, present_theta);
    printf("Target position [x y]:[%lf %lf]\n", target_pos.x, target_pos.y);
    printf("Present position [x y]:[%lf %lf]\n", present_pos.x, present_pos.y);

    //target_angleを変更
    if (state == 0)
    {
      state = 1;
      target_pos = target_pos2;
    }
    else if (state == 1)
    {
      state = 2;
      target_pos = target_pos3;
    }
    else if (state == 2)
    {
      state = 3;
      target_pos = target_pos4;
    }
    else
    {
      state = 0;
      target_pos = target_pos1;
    }
  } //end main while

  brakeCranex7Joint(); //CRANE X7をブレーキにして終了
  closeCranex7Port();  //シリアルポートを閉じる
  return 0;
}
