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
  double theta1[JOINT_NUM] = {0.244979, 1.664740, 0.0, -2.071451, 0.0, 0.0, 0.0, 0.0};
  double theta2[JOINT_NUM] = {0};

  VECTOR_3D pos1 = {0};
  VECTOR_3D pos2 = {0.15, 0.15, 0.15};

  initParam();

  forwardKinematics3Dof(&pos1, theta1);
  inverseKinematics3Dof(pos2, theta2);

  printf("Forward kinematics result [θ1 θ2 θ3]:[%lf %lf %lf] -> [x y z]:[%lf %lf %lf]\n", theta1[0], theta1[1], theta1[3], pos1.x, pos1.y, pos1.z);
  printf("Inverse kinematics result [x y z]:[%lf %lf %lf] -> [θ1 θ2 θ3]:[%lf %lf %lf]\n", pos2.x, pos2.y, pos2.z, theta2[0], theta2[1], theta2[3]);

  return 0;
}