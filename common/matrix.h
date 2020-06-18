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

/**
 * @file matrix.h
 * @brief bare minimum matrix library
 * @author RT Corporation
 * @date 2019/03/22
 */

#ifndef MATRIX_H_
#define MATRIX_H_

//// Structure definition ////
/**
 * @struct VECTOR_3D
 * @brief 3 dimentional vector
 */
typedef struct
{
    double x;
    double y;
    double z;
} VECTOR_3D;

/**
 * @struct MATRIX_3D
 * @brief 3 raws x 3 columns matrix
 */
typedef struct
{
    double a[3][3];
} MATRIX_3D;

/**
 * @struct VECTOR_4D
 * @brief 4 dimentional vector
 */
typedef struct
{
    double x[4];
} VECTOR_4D;

/**
 * @struct MATRIX_4D
 * @brief 4 raws x 4 columns matrix
 */
typedef struct
{
    double a[4][4];
} MATRIX_4D;

//// Prototype declaration ////
MATRIX_3D mulMatMat3D(MATRIX_3D *, MATRIX_3D *);
VECTOR_3D mulMatVec3D(MATRIX_3D *, VECTOR_3D *);
MATRIX_3D mulScoMat3D(double, MATRIX_3D *);
VECTOR_3D mulScoVec3D(double, VECTOR_3D *);
VECTOR_3D crsVecVec3D(VECTOR_3D *, VECTOR_3D *);
MATRIX_3D sumMatMat3D(MATRIX_3D *, MATRIX_3D *);
VECTOR_3D sumVecVec3D(VECTOR_3D *, VECTOR_3D *);
VECTOR_3D subVecVec3D(VECTOR_3D *, VECTOR_3D *);
MATRIX_3D transeposeMat3D(MATRIX_3D *);
int inverseMat3D(MATRIX_3D *, MATRIX_3D *);
MATRIX_4D mulMatMat4D(MATRIX_4D *, MATRIX_4D *);
VECTOR_4D mulMatVec4D(MATRIX_4D *, VECTOR_4D *);

#endif
