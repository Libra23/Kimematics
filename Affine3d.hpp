#ifndef _MATH_UTILITY_H
#define _MATH_UTILITY_H

#include <BasicLinearAlgebra.h>

typedef BLA::Matrix<3> Vector3d;
typedef BLA::Matrix<3,3> Matrix3d;
typedef BLA::Matrix<4,4> Matrix4d;

class Affine3d {
    public:
    Affine3d();
    static Affine3d Identity();
    Vector3d& translation();
    Vector3d translation() const;
    Matrix3d& linear();
    Matrix3d linear() const;
    Matrix3d rotation() const;
    Affine3d operator*(Affine3d affine3d);
    Affine3d Inverse();
    Matrix4d ToMatrix() const;
    private:
    Vector3d transition_;
    Matrix3d linear_;
};

#endif