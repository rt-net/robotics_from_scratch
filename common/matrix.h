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