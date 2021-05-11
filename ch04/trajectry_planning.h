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


#ifndef TRAJECTRY_PLANNING_H
#define TRAJECTRY_PLANNING_H

#include "../common/arm_parameter.h"

#define MAX_LENGTH (10000) //軌道の配列要素数

int trapezoidalTrajectoryJointSpace(double *, double *, double, double *, double, double (*)[JOINT_NUM], double (*)[JOINT_NUM], double (*)[JOINT_NUM]);

#endif