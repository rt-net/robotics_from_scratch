/**
 * @file main.c
 * @brief servo test
 * @author RT Corporation
 * @date 2020/06/15
 */
#include <stdio.h>
#include <unistd.h>
#include "../common/crane_x7_comm.h"

int main()
{
  uint8_t operating_mode[JOINT_NUM] = {POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE};
  double *target_angle;                                                      //目標角度
  double target_angle1[JOINT_NUM] = {0};                                     //目標角度1
  double target_angle2[JOINT_NUM] = {0, 1.57, 0, 0, 0, 0, 0, 0};             //目標角度2
  double target_angle3[JOINT_NUM] = {0.78, 1.57, 0.78, 0, 0.78, 0, 0.78, 0}; //目標角度3
  double target_angle4[JOINT_NUM] = {0, 1.57, 0, 0.78, 0, 0, 0, 0};          //目標角度4

  int state = 0; //目標位置を切り替えるための変数
  int cnt = 0;   //ループカウント

  printf("Press any key to start (or press q to quit)\n");
  if (getchar() == ('q'))
    return 0;

  // サーボ関連の設定の初期化
  if (initilizeCranex7(operating_mode))
  {
    return 1;
  }
  // CRANE-X7のトルクON
  setCranex7TorqueEnable(TORQUE_ENABLE);

  target_angle = target_angle1;
  // main関数のループ
  while (cnt < 5)
  { //10回繰り返したらプログラムを終了する
    cnt++;
    setCranex7Angle(target_angle);

    sleep(3);

    //target_angleを変更
    if (state == 0)
    {
      state = 1;
      target_angle = target_angle2;
    }
    else if (state == 1)
    {
      state = 2;
      target_angle = target_angle3;
    }
    else if (state == 2)
    {
      state = 3;
      target_angle = target_angle4;
    }
    else
    {
      state = 0;
      target_angle = target_angle1;
    }
  } //end main while

  brakeCranex7Joint(); //CRANE X7をブレーキにして終了
  closeCranex7Port();  //シリアルポートを閉じる
  return 0;
}
