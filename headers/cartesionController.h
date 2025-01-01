
class Controller;

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
