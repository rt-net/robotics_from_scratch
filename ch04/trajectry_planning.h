#ifndef TRAJECTRY_PLANNING_H
#define TRAJECTRY_PLANNING_H

#include "../common/arm_parameter.h"

#define MAX_LENGTH (10000) //軌道の配列要素数

int trapezoidalTrajectoryJointSpace(double *, double *, double, double, double, double (*)[JOINT_NUM], double (*)[JOINT_NUM], double (*)[JOINT_NUM]);

#endif