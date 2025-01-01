#pragma once
#include <Eigen/Dense>

class Kinematics {
public:
    static Eigen::Matrix4d forwardKinematics(const Robot& robot);
    static bool inverseKinematics(Robot& robot, const Eigen::Matrix4d& targetPose,
                                std::vector<double>& solution);
};
