#include <vector>
#include "../headers/robot.h"
#include "../Eigen/Dense"


class Controller {
protected:
    Robot& robot;
    double dt;  // Control loop time step

public:
    Controller(Robot& robot, double dt);
    virtual ~Controller() = default;
    virtual void update() = 0;
};

// JointSpaceController.hpp
class JointSpaceController : public Controller {
private:
    std::vector<double> kp;  // Proportional gains
    std::vector<double> ki;  // Integral gains
    std::vector<double> kd;  // Derivative gains
    std::vector<double> targetPositions;
    std::vector<double> integralError;

public:
    JointSpaceController(Robot& robot, double dt);
    
    void setGains(const std::vector<double>& kp,
                 const std::vector<double>& ki,
                 const std::vector<double>& kd);
    void setTargetPositions(const std::vector<double>& targets);
    void update() override;
};


class CartesianController : public Controller {
private:
    Eigen::Matrix4d targetPose;
    Eigen::Matrix<double, 6, 6> kp;  // Proportional gains
    Eigen::Matrix<double, 6, 6> ki;  // Integral gains
    Eigen::Matrix<double, 6, 6> kd;  // Derivative gains
    Eigen::Matrix<double, 6, 1> integralError;

public:
    CartesianController(Robot& robot, double dt);
    
    void setGains(const Eigen::Matrix<double, 6, 6>& kp,
                 const Eigen::Matrix<double, 6, 6>& ki,
                 const Eigen::Matrix<double, 6, 6>& kd);
    void setTargetPose(const Eigen::Matrix4d& target);
    void update() override;
};
