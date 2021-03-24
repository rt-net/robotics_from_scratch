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
#include "../common/arm_parameter.h"
#include "myCX7_KDL_library.h"
#include "trajectry_planning.h"

int main()
{
  //double theta1[JOINT_NUM] = {0.244979, 1.664740, 0.0, -2.071451, 0.0, 0.0, 0.0, 0.0};
  double theta1[JOINT_NUM] = {0};
  double theta2[JOINT_NUM] = {0};
  double v_max = 1.0;
  double T = 1.0;
  double dt = 0.005;

  VECTOR_3D pos1 = {0.2, 0.05, 0.15};
  VECTOR_3D pos2 = {0.15, 0.15, 0.15};

  double trajectry_angle[MAX_LENGTH][JOINT_NUM] = {0};
  double trajectry_angular_vel[MAX_LENGTH][JOINT_NUM] = {0};
  double trajectry_angular_acc[MAX_LENGTH][JOINT_NUM] = {0};

  FILE *fp;
  fp = fopen("test.csv", "w");

  initParam();

  inverseKinematics3Dof(pos1, theta1);
  inverseKinematics3Dof(pos2, theta2);

  trapezoidalTrajectoryJointSpace(theta1, theta2, v_max, T, dt, trajectry_angle, trajectry_angular_vel, trajectry_angular_acc);

  for (int i = 0; i < (T / dt + 1); i++)
  {
    fprintf(fp, "%lf,%lf,%lf,%lf\n", dt * i, trajectry_angle[i][0], trajectry_angular_vel[i][0], trajectry_angular_acc[i][0]);
  }

  fclose(fp);

  return 0;
}