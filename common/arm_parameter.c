/**
 * @file arm_parameter.c
 * @brief joint parameters of CRANE-X7
 * @author RT Corporation
 * @date 2019-2020
 * @copyright License: Apache License, Version 2.0
 */
// Copyright 2019 RT Corporation
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

#include "arm_parameter.h"

static const LINK_PARAM base_link =
    //     mass  length  centor of mass                        inertia of tensor
    //   m [kg], l [m], {    xg,        yg,            zg}[m] {           Ixx,            Ixy,           Ixz}, {            Iyx,            Iyy,         Iyz}, {           Izx,         Izy,            Izz}[kgm^2]
    {0.388, 0.041, {-6.734e-3, -0.001e-3, 17.921e-3}, {{{324812.981e-9, 1.195e-9, -47122.898e-9}, {1.195e-9, 482993.163e-9, 19.475e-9}, {-47122.898e-9, 19.475e-9, 585569.734e-9}}}}; //base_link

//
static const LINK_PARAM link_parameter_3dof[LINK_NUM_3DOF] = {
    //mass  length  centor of mass                        inertia of tensor
    //m[kg], l[m], {    xg,        yg,       zg}[m] {           Ixx,            Ixy,           Ixz}, {            Iyx,            Iyy,         Iyz}, {           Izx,         Izy,            Izz}[kgm^2]
    {0.253, 0.064, {0.243e-3, -0.105e-3, 46.95e-3}, {{{178706.106e-9, -73.234e-9, 888.676e-9}, {-73.234e-9, 176299.332e-9, 623.013e-9}, {888.676e-9, 623.013e-9, 89586.08e-9}}}},                   //1_link
    {0.451, 0.250, {122.346e-3, 9.596e-3, 0.097e-3}, {{{273465.146e-9, -127207.638e-9, -5428.565e-9}, {-127207.638e-9, 3498438.202e-9, 1007.993e-9}, {-5428.565e-9, 1007.993e-9, 3545676.16e-9}}}}, //2_link
    {0.712, 0.250, {186.265e-3, 1.924e-3, 0.357e-3}, {{{344745.452e-9, 279720.99e-9, -11963.306e-9}, {279720.99e-9, 5739146.215e-9, 52.649e-9}, {-11963.306e-9, 52.649e-9, 5783803.266e-9}}}},      //3_link
};

static const JOINT_RANGE joint_range[JOINT_NUM] = {
    /*  min [rad], max[rad]  */
    {-2.740, 2.740}, //joint 1
    {0.000, 3.141},  //joint 2
    {-2.740, 2.740}, //joint 3
    {-2.740, 0.000}, //joint 4
    {-2.740, 2.740}, //joint 5
    {-1.570, 1.570}, //joint 6
    {-2.914, 2.914}, //joint 7
    {-0.087, 1.570}, //end efector
};

/**
 * @fn void getLinkParamBase(LINK_PARAM *)
 * @brief get base link physical parameter
 * @param[out] *link_parameter
 */
void getLinkParamBase(LINK_PARAM *link_parameter)
{
        *link_parameter = base_link;
}

/**
 * @fn void getLinkParam2Dof(LINK_PARAM *)
 * @brief get physical parameter of CRANE-X7  when it is assumed to be 2 degrees of freedom arm.
 * @param[out] *link_parameter
 */
void getLinkParam2Dof(LINK_PARAM *link_parameter)
{
        for (int i = 0; i < LINK_NUM_2DOF; i++)
        {
                link_parameter[i] = link_parameter_3dof[i + 1];
        }
}

/**
 * @fn void getLinkParam3Dof(LINK_PARAM *)
 * @brief get physical parameter of CRANE-X7  when it is assumed to be 3 degrees of freedom arm.
 * @param[out] *link_parameter
 */
void getLinkParam3Dof(LINK_PARAM *link_parameter)
{
        int i;
        for (i = 0; i < LINK_NUM_3DOF; i++)
        {
                link_parameter[i] = link_parameter_3dof[i];
        }
}

void getJointRange(JOINT_RANGE *joint_parameter)
{
        int i;
        for (i = 0; i < JOINT_NUM; i++)
        {
                joint_parameter[i] = joint_range[i];
        }
}
