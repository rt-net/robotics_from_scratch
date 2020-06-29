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

int main()
{
  double theta1[JOINT_NUM] = {0.0, 1.789936, 0.0, -2.403867, 0.0, 0.0, 0.0, 0.0};
  double theta2[JOINT_NUM] = {0};

  VECTOR_3D pos1 = {0};
  VECTOR_3D pos2 = {0.25, 0.15, 0};

  initParam();

  forwardKinematics2Dof(&pos1, theta1);
  inverseKinematics2Dof(pos2, theta2);

  printf("Forward kinematics result [θ1 θ2]:[%lf %lf] -> [x y]:[%lf %lf]\n", theta1[1], theta1[3], pos1.x, pos1.y);
  printf("Inverse kinematics result [x y]:[%lf %lf] -> [θ1 θ2]:[%lf %lf]\n", pos2.x, pos2.y, theta2[1], theta2[3]);

  return 0;
}