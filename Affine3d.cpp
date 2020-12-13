#include "Affine3d.hpp"

Affine3d::Affine3d() {
    this->transition_.Fill(0.0);
    this->linear_ << 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;
}

Affine3d Affine3d::Identity() {
    Affine3d trans;
    trans.transition_.Fill(0.0);
    trans.linear_ << 1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0;
    return trans;
}

Vector3d& Affine3d::translation() {
    return this->transition_;
}

Vector3d Affine3d::translation() const {
    return this->transition_;
}

Matrix3d& Affine3d::linear() {
    return this->linear_;
}

Matrix3d Affine3d::linear() const {
    return this->linear_;
}

Matrix3d Affine3d::rotation() const {
    return this->linear_ / this->linear_.Det();
}

Affine3d Affine3d::Inverse() {
    Affine3d ret;
    ret.transition_ = - this->linear_.Inverse() * this->transition_;
    ret.linear_ = this->linear_.Inverse();
    return ret;
}

Matrix4d Affine3d::ToMatrix() const {
    Matrix4d mat;
    for (int i = 0; i < mat.Rows; i++) {
        for (int j = 0; j < mat.Cols; j++) {
            if (i < this->linear_.Rows && j < this->linear_.Cols) {
                mat(i, j) = this->linear_(i, j);
            } else if (i == this->linear_.Rows && j == this->linear_.Cols) {
                mat(i, j) = 1.0;
            } else if (i == this->linear_.Rows && j < this->linear_.Cols) {
                mat(i, j) = 0.0;
            } else if (i < this->linear_.Rows && j == this->linear_.Cols) {
                mat(i, j) = this->transition_(i);
            }
            
        }
    }
    return mat;
}

Affine3d Affine3d::operator*(Affine3d affine3d) {
    Affine3d ret;
    ret.transition_ = this->linear_ * affine3d.transition_ + this->transition_;
    ret.linear_ = this->linear_ * affine3d.linear_;
    return ret;
}