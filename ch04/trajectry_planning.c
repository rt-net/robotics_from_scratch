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


#include <math.h>
#include <stdio.h>

#include "trajectry_planning.h"
#include "../common/matrix.h"

int trapezoidalTrajectoryJointSpace(double *theta_0, double *theta_f, double v_max, double T, double dt, double (*theta_d)[JOINT_NUM], double (*theta_d_dot)[JOINT_NUM], double (*theta_d_ddot)[JOINT_NUM])
{

    // 軌道の計算用の変数
    double t, s, s_dot, s_ddot;
    int n = 0;
    // 速度台形則のパラメータ
    double vM = 0.0;
    double a = 0.0;
    double tb = 0.0;
    double theta_max = 0.0;

    // 軌道の配列の長さを計算し、予め確保されているものより大きい場合はエラーとする
    if ((1 + T / dt) > MAX_LENGTH)
    {
        return 1;
    }
    n = 1 + T / dt;

    // 一番変位の大きい関節の角度を計算する
    for (int i = 0; i < JOINT_NUM; i++)
    {
        if (theta_max < fabs(theta_f[i] - theta_0[i]))
            theta_max = fabs(theta_f[i] - theta_0[i]);
    }

    // 速度台形則のパラメータを計算
    if (theta_max > 0.0)
    {
        vM = v_max / theta_max;
        if (vM > (2.0 / T))
        {
            vM = 2.0 / T;
        }
        else if (vM < (1 / T))
        {
            vM = 1.0 / T;
        }
        tb = T - 1 / vM;
        a = vM / tb;
    }

    // 時系列の軌道の生成
    for (int i = 0; i < n; i++)
    {
        t = i * dt;
        if (t <= tb)
        {
            s = a * t * t / 2;
            s_dot = a * t;
            s_ddot = a;
        }
        else if (t <= (T - tb))
        {
            s = vM * (t - tb / 2);
            s_dot = vM;
            s_ddot = 0;
        }
        else
        {
            s = 1 - (a / 2) * (T - t) * (T - t);
            s_dot = a * (T - t);
            s_ddot = -a;
        }
        for (int j = 0; j < JOINT_NUM; j++)
        {
            theta_d[i][j] = theta_0[j] * (1 - s) + theta_f[j] * s;
            theta_d_dot[i][j] = (theta_f[j] - theta_0[j]) * s_dot;
            theta_d_ddot[i][j] = (theta_f[j] - theta_0[j]) * s_ddot;
        }
    }
    return 0;
}