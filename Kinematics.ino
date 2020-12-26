#include "Kinematic.hpp"

Kinematic kinematic_;
double t = 0.0;

void PrintAffine3d(const Affine3d& trans);
void PrintVector3d(const Vector3d& vec);

void setup() {
  // put your setup code here, to run once:
    KinematicModel model;
    model.xyz = {{
                {{27.5, 47.63, 0.0}},   // Yaw
                {{33.25, 0.0, 0.0}},    // Pitch1
                {{60.0, 0.0, 0.0}},     // Pitch2
                {{120.0, 0.0, 0.0}},    // Tip
                }};
    model.axis = {{
                 {{0.0, 0.0, 1.0}},      // Yaw
                 {{0.0, 1.0, 0.0}},      // Pitch1
                 {{0.0, 1.0, 0.0}},      // Pitch2
                 {{0.0, 0.0, 0.0}},      // Tip
                 }};
    model.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    kinematic_.Config(model);
    kinematic_.SetConstant(1500.0, 0.1);

    Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Check FK");
  Joint q;
  q << 45.0, 0.0, 0.01;
  q *= DEG_TO_RAD;
  Affine3d tip_trans;
  kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
  PrintVector3d(tip_trans.translation());
  PrintVector3d(q * RAD_TO_DEG);
  
  Serial.println("Check IK");
  Joint q_pre = q;
  const Affine3d tip_trans_pre = tip_trans;
  tip_trans.translation() = tip_trans_pre.translation() + Vector3d(0.0, 0.0, 0.05);
  bool ik_ret = kinematic_.Inverse(tip_trans, Affine3d::Identity(), q_pre, q);
  kinematic_.Forward(q, Affine3d::Identity(), tip_trans);
  Serial.print("IK = ");Serial.println(ik_ret);
  PrintVector3d(tip_trans.translation());
  PrintVector3d(q * RAD_TO_DEG);
  
  Serial.println("End");
  t += 0.001; // 1 ms
}

void PrintAffine3d(const Affine3d& trans) {
    Serial.println("Transition");
    Vector3d transition = trans.translation();
    for (int i = 0; i < transition.Rows; i++) {
        Serial.print(transition(i)); Serial.print(" ");
    }
    Serial.println();
    Serial.println("Rotation");
    Matrix3d rotation = trans.rotation();
    for (int i = 0;i < rotation.Rows; i++) {
        for (int j = 0; j < rotation.Cols; j++) {
            Serial.print(rotation(i, j)); Serial.print(" ");
        }
        Serial.println();
    }
}

void PrintVector3d(const Vector3d& vec) {
    Serial.println("Vector");
    for (int i = 0; i < vec.Rows; i++) {
        Serial.print(vec(i)); Serial.print(" ");
    }
    Serial.println();
}