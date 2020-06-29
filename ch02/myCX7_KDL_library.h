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

#ifndef MYCX7_KDL_LIB_H
#define MYCX7_KDL_LIB_H

#include "../common/matrix.h"

void initParam(void);

//// Functions related to kinematics ////
int forwardKinematics2Dof(VECTOR_3D *, double *);
int inverseKinematics2Dof(VECTOR_3D, double *);

#endif