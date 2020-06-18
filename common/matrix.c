/**
 * @file matrix.c
 * @brief bare minimum matrix library
 * @author RT Corporation
 * @date 2020/03/22
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

#include "matrix.h"

/**
 * @fn MATRIX_3D mulMatMat3D(MATRIX_3D*, MATRIX_3D*)
 * @brief multiple 3x3 matrix and 3x3 matrix
 * @param[in] MATRIX_3D* 3x3 matrix
 * @param[in] MATRIX_3D* 3x3 matrix
 * @return result of multiplication
 */
MATRIX_3D mulMatMat3D(MATRIX_3D *MatA, MATRIX_3D *MatB)
{
    MATRIX_3D result;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                result.a[i][j] += MatA->a[i][k] * MatB->a[k][j];
            }
        }
    }
    return result;
};

/**
 * @fn VECTOR_3D mulMatVec3D(MATRIX_3D*, VECTOR_3D*)
 * @brief multiple 3x3 matrix and 3 dimentional vector
 * @param[in] MATRIX_3D* 3x3 matrix
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of multiplication
 */
VECTOR_3D mulMatVec3D(MATRIX_3D *MatA, VECTOR_3D *VecA)
{
    VECTOR_3D result;

    result.x = MatA->a[0][0] * VecA->x + MatA->a[0][1] * VecA->y + MatA->a[0][2] * VecA->z;
    result.y = MatA->a[1][0] * VecA->x + MatA->a[1][1] * VecA->y + MatA->a[1][2] * VecA->z;
    result.z = MatA->a[2][0] * VecA->x + MatA->a[2][1] * VecA->y + MatA->a[2][2] * VecA->z;

    return result;
};

/**
 * @fn MATRIX_3D mulScoMat3D(double, MATRIX_3D*)
 * @brief multiple scolar value and 3x3 matrix
 * @param[in] double scolar value
 * @param[in] MATRIX_3D* 3x3 matrix
 * @return result of multiplication
 */
MATRIX_3D mulScoMat3D(double c, MATRIX_3D *MatA)
{
    MATRIX_3D result;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = c * MatA->a[i][j];
        }
    }

    return result;
};

/**
 * @fn VECTOR_3D mulScoVec3D(double, VECTOR_3D *)
 * @brief multiple scolar value and 3 dimentional vector
 * @param[in] double scolar value
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of multiplication
 */
VECTOR_3D mulScoVec3D(double c, VECTOR_3D *VecA)
{
    VECTOR_3D result;

    result.x = c * VecA->x;
    result.y = c * VecA->y;
    result.z = c * VecA->z;

    return result;
};

/**
 * @fn VECTOR_3D crsVecVec3D(VECTOR_3D *, VECTOR_3D *)
 * @brief cross product of 3 dimentional vector and 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of cross product
 */
VECTOR_3D crsVecVec3D(VECTOR_3D *VecA, VECTOR_3D *VecB)
{
    VECTOR_3D result;

    result.x = VecA->y * VecB->z - VecA->z * VecB->y;
    result.y = VecA->z * VecB->x - VecA->x * VecB->z;
    result.z = VecA->x * VecB->y - VecA->y * VecB->x;

    return result;
};

/**
 * @fn MATRIX_3D sumMatMat3D(MATRIX_3D *, MATRIX_3D *)
 * @brief summation of 3x3 matrix and 3x3 matrix
 * @param[in] MATRIX_3D* 3x3 matrix
 * @param[in] MATRIX_3D* 3x3 matrix
 * @return result of summation
 */
MATRIX_3D sumMatMat3D(MATRIX_3D *MatA, MATRIX_3D *MatB)
{
    MATRIX_3D result;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = MatA->a[i][j] + MatB->a[i][j];
        }
    }

    return result;
};

/**
 * @fn VECTOR_3D sumVecVec3D(VECTOR_3D *, VECTOR_3D *)
 * @brief summation of 3 dimentional vector and 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of summation
 */
VECTOR_3D sumVecVec3D(VECTOR_3D *VecA, VECTOR_3D *VecB)
{
    VECTOR_3D result;

    result.x = VecA->x + VecB->x;
    result.y = VecA->y + VecB->y;
    result.z = VecA->z + VecB->z;

    return result;
};

/**
 * @fn VECTOR_3D subVecVec3D(VECTOR_3D *, VECTOR_3D *)
 * @brief subtraction of 3 dimentional vector and 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of subtraction
 */
VECTOR_3D subVecVec3D(VECTOR_3D *VecA, VECTOR_3D *VecB)
{
    VECTOR_3D result;

    result.x = VecA->x - VecB->x;
    result.y = VecA->y - VecB->y;
    result.z = VecA->z - VecB->z;

    return result;
};

/**
 * @fn MATRIX_3D transeposeMat3D(MATRIX_3D *)
 * @brief transpose 3x3 matrix
 * @param[in] VECTOR_3D* 3 dimentional vector
 * @return result of transpose
 */
MATRIX_3D transeposeMat3D(MATRIX_3D *MatA)
{
    MATRIX_3D result;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result.a[i][j] = MatA->a[j][i];
        }
    }

    return result;
};

/**
 * @fn int inverseMat3D(MATRIX_3D *, MATRIX_3D *)
 * @brief summation of 3x3 matrix and 3x3 matrix
 * @param[in] MATRIX_3D* 3x3 matrix
 * @param[out] MATRIX_3D* inversed matrix
 * @return Success or failure
 */
int inverseMat3D(MATRIX_3D *MatA, MATRIX_3D *InvA)
{
    double detMatA = MatA->a[0][0] * MatA->a[1][1] * MatA->a[2][2] + MatA->a[0][1] * MatA->a[1][2] * MatA->a[2][0] + MatA->a[0][2] * MatA->a[1][0] * MatA->a[2][1] - MatA->a[0][2] * MatA->a[1][1] * MatA->a[2][0] - MatA->a[0][1] * MatA->a[1][0] * MatA->a[2][2] - MatA->a[0][0] * MatA->a[1][2] * MatA->a[2][1];
    if (detMatA == 0)
        return 0;

    InvA->a[0][0] = (MatA->a[1][1] * MatA->a[2][2] - MatA->a[1][2] * MatA->a[2][1]) / detMatA;
    InvA->a[0][1] = -(MatA->a[0][1] * MatA->a[2][2] - MatA->a[0][2] * MatA->a[2][1]) / detMatA;
    InvA->a[0][2] = (MatA->a[0][1] * MatA->a[1][2] - MatA->a[0][2] * MatA->a[1][1]) / detMatA;
    InvA->a[1][0] = -(MatA->a[1][0] * MatA->a[2][2] - MatA->a[1][2] * MatA->a[2][1]) / detMatA;
    InvA->a[1][1] = (MatA->a[0][0] * MatA->a[2][2] - MatA->a[0][2] * MatA->a[2][0]) / detMatA;
    InvA->a[1][2] = -(MatA->a[0][0] * MatA->a[1][2] - MatA->a[0][2] * MatA->a[1][0]) / detMatA;
    InvA->a[2][0] = (MatA->a[1][0] * MatA->a[2][1] - MatA->a[1][1] * MatA->a[2][0]) / detMatA;
    InvA->a[2][1] = -(MatA->a[0][0] * MatA->a[2][1] - MatA->a[0][1] * MatA->a[2][0]) / detMatA;
    InvA->a[2][2] = (MatA->a[0][0] * MatA->a[1][1] - MatA->a[0][1] * MatA->a[1][0]) / detMatA;

    return 1;
}

/**
 * @fn MATRIX_4D mulMatMat4D(MATRIX_4D *, MATRIX_4D *)
 * @brief multiple 4x4 matrix and 4x4 matrix
 * @param[in] MATRIX_4D* 4x4 matrix
 * @param[in] MATRIX_4D* 4x4 matrix
 * @return result of multiplication
 */
MATRIX_4D mulMatMat4D(MATRIX_4D *MatA, MATRIX_4D *MatB)
{
    MATRIX_4D result;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result.a[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                result.a[i][j] += MatA->a[i][k] * MatB->a[k][j];
            }
        }
    }
    return result;
};

/**
 * @fn VECTOR_4D mulMatVec4D(MATRIX_4D *, VECTOR_4D *)
 * @brief multiple 4x4 matrix and 4 dimentional vector
 * @param[in] MATRIX_4D* 4x4 matrix
 * @param[in] VECTOR_4D* 4 dimentional vector
 * @return result of multiplication
 */
VECTOR_4D mulMatVec4D(MATRIX_4D *MatA, VECTOR_4D *VecA)
{
    VECTOR_4D result;

    for (int i = 0; i < 4; i++)
    {
        result.x[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            result.x[i] += MatA->a[i][j] * VecA->x[j];
        }
    }
    return result;
};
