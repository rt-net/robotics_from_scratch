/**
 * @file matrix.h
 * @brief bare minimum matrix library
 * @author RT Corporation
 * @date 2019/03/23
 */

#ifndef ARM_PARAMETER_H_
#define ARM_PARAMETER_H_

#include "matrix.h"

#define LINK_NUM_2DOF (2)
#define LINK_NUM_3DOF (3)

#ifndef JOINT_NUM
#define JOINT_NUM (8)
#endif

//// Structure definition ////
/**
 * @struct LINK_PARAM
 * @brief Structure for storing physical parameter of links
 */
typedef struct
{
    double mass;
    double length;            // link length
    VECTOR_3D com;            // centor of mass position (local coordinate)
    MATRIX_3D inertia_tensor; // tensor of inertia (coordinates from the center of mass)
} LINK_PARAM;

/**
 * @struct JOINT_RANGE
 * @brief Structure for storing movable range of joint
 */
typedef struct
{
    double min; // minimum movable range
    double max; // maximum movable range
} JOINT_RANGE;

/**
 * @struct FB_GAIN
 * @brief Structure for storing feedback gain
 */
typedef struct
{
    double Kp; // proportional gain
    double Kd; // differential gain
    //double Ki;              // integral gain
} FB_GAIN;

//// Prototype declaration ////
void getLinkParamBase(LINK_PARAM *);
void getLinkParam2Dof(LINK_PARAM *);
void getLinkParam3Dof(LINK_PARAM *);
void getJointRange(JOINT_RANGE *);

#endif
