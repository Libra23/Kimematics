#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#include "Const.hpp"
#include <array>

#if defined(ESP32)
#include "Affine3d.hpp"
using namespace BLA;
typedef Matrix<NUM_JOINT> Joint;
typedef Matrix<DOF> Twist;
typedef Matrix<DOF, NUM_JOINT> Jacobian;
typedef Matrix<DOF, DOF> MatrixDoF;
#else
#include "Eigen/Dense"
using namespace Eigen;
typedef Matrix<double, NUM_JOINT, 1> Joint;
typedef Matrix<double, DOF, 1> Twist;
typedef Matrix<double, DOF, NUM_JOINT> Jacobian;
typedef Matrix<double, DOF, DOF> MatrixDoF;

#endif

enum JointType {
    ROTATE,
    SLIDE,
    FIXED
};

struct KinematicModel {
    std::array<std::array<double, XYZ>, NUM_JOINT + 1> xyz;
    std::array<std::array<double, XYZ>, NUM_JOINT + 1> axis;
    std::array<JointType, NUM_JOINT + 1> type;
};

class Kinematic {
    public:
    Kinematic();
    void Config(const KinematicModel& model, int num_ik_max = 20);
    void Forward(const Joint& q, const Affine3d& base_trans, Affine3d& tip_trans);
    bool Inverse(const Affine3d& tip_trans, const Affine3d& base_trans, const Joint& init_q, Joint& q);
    double GetControllability() const;
    double GetStablility() const;
    void SetConstant(double w, double k);

    private:
    KinematicModel model_;
    int num_ik_max_;
    double w_, k_;
    double w_0_, k_0_;
    Affine3d CvtModelToTrans(const std::array<double, 3>& xyz, const std::array<double, 3>& axis, JointType type, double q);
    Jacobian GetJacobian(const Joint& q, const Affine3d& base_trans, const Affine3d& tip_trans);
    Jacobian SingularityLowSensitiveInverse(const Jacobian& jacobian);
};

Matrix3d MatrixFromRpy(const Vector3d& rpy);
Vector3d Rpy(const Matrix3d& m);
Twist Differentiate(const Affine3d& trans, const Affine3d& trans_pre, double dt);
MatrixDoF IdentityDoF();
#endif
