#include "../headers/Controller.h"
#include <stdexcept>
#include <format>

Controller::Controller(Robot& robot, double dt)
    : robot(robot)
    , dt(dt)
    , isEnabled(false)
    , lastUpdateTime(0.0)
    , safetyLimits{
        .maxPositionError = 0.5,    // 0.5 rad or meters
        .maxVelocity = 2.0,         // 2 rad/s or m/s
        .maxAcceleration = 5.0,     // 5 rad/s² or m/s²
        .maxJerk = 10.0             // 10 rad/s³ or m/s³
    } {
    
    if (dt <= 0) {
        throw std::invalid_argument("Controller time step must be positive");
    }
}

void Controller::enable() {
    isEnabled = true;
    lastUpdateTime = getCurrentTime();
    resetInternalState();
}

void Controller::disable() {
    isEnabled = false;
    resetInternalState();
}

bool Controller::isControllerEnabled() const {
    return isEnabled;
}

void Controller::setSafetyLimits(const SafetyLimits& limits) {
    if (limits.maxPositionError <= 0 ||
        limits.maxVelocity <= 0 ||
        limits.maxAcceleration <= 0 ||
        limits.maxJerk <= 0) {
        throw std::invalid_argument("Safety limits must be positive");
    }
    safetyLimits = limits;
}

// JointSpaceController.cpp
JointSpaceController::JointSpaceController(Robot& robot, double dt)
    : Controller(robot, dt)
    , targetPositions(robot.getNumJoints(), 0.0)
    , targetVelocities(robot.getNumJoints(), 0.0)
    , kp(robot.getNumJoints(), 10.0)  // Default gains
    , ki(robot.getNumJoints(), 0.0)
    , kd(robot.getNumJoints(), 1.0)
    , integralError(robot.getNumJoints(), 0.0)
    , lastError(robot.getNumJoints(), 0.0)
    , maxIntegralError(robot.getNumJoints(), 1.0)  // Anti-windup limit {

void JointSpaceController::setGains(const std::vector<double>& newKp,
                                  const std::vector<double>& newKi,
                                  const std::vector<double>& newKd) {
    if (newKp.size() != robot.getNumJoints() ||
        newKi.size() != robot.getNumJoints() ||
        newKd.size() != robot.getNumJoints()) {
        throw std::invalid_argument(
            std::format("Gain vectors must have size {} (number of joints)",
                robot.getNumJoints())
        );
    }

    // Validate gains
    for (size_t i = 0; i < robot.getNumJoints(); ++i) {
        if (newKp[i] < 0 || newKi[i] < 0 || newKd[i] < 0) {
            throw std::invalid_argument(
                std::format("Gains for joint {} must be non-negative", i)
            );
        }
    }

    kp = newKp;
    ki = newKi;
    kd = newKd;
    
    // Reset integral terms when gains change
    std::fill(integralError.begin(), integralError.end(), 0.0);
}

void JointSpaceController::setTargetPositions(const std::vector<double>& targets) {
    if (targets.size() != robot.getNumJoints()) {
        throw std::invalid_argument(
            std::format("Target position vector must have size {} (number of joints)",
                robot.getNumJoints())
        );
    }

    // Validate targets against joint limits
    for (size_t i = 0; i < robot.getNumJoints(); ++i) {
        auto joint = robot.getJoint(i);
        auto [minPos, maxPos] = joint->getPositionLimits();
        if (targets[i] < minPos || targets[i] > maxPos) {
            throw std::invalid_argument(
                std::format("Target position {} for joint {} exceeds limits [{}, {}]",
                    targets[i], i, minPos, maxPos)
            );
        }
    }

    targetPositions = targets;
}

void JointSpaceController::setTargetVelocities(const std::vector<double>& targets) {
    if (targets.size() != robot.getNumJoints()) {
        throw std::invalid_argument(
            std::format("Target velocity vector must have size {} (number of joints)",
                robot.getNumJoints())
        );
    }

    // Validate targets against velocity limits
    for (size_t i = 0; i < robot.getNumJoints(); ++i) {
        auto joint = robot.getJoint(i);
        double velLimit = joint->getVelocityLimit();
        if (std::abs(targets[i]) > velLimit) {
            throw std::invalid_argument(
                std::format("Target velocity {} for joint {} exceeds limit {}",
                    targets[i], i, velLimit)
            );
        }
    }

    targetVelocities = targets;
}

void JointSpaceController::update() {
    if (!isEnabled) {
        return;
    }

    double currentTime = getCurrentTime();
    double elapsedTime = currentTime - lastUpdateTime;
    
    if (elapsedTime < dt * 0.5) {
        return;  // Too soon to update
    }

    std::vector<double> currentPositions = robot.getJointPositions();
    std::vector<double> currentVelocities = robot.getJointVelocities();
    std::vector<double> commands(robot.getNumJoints());

    for (size_t i = 0; i < robot.getNumJoints(); ++i) {
        // Compute errors
        double posError = targetPositions[i] - currentPositions[i];
        double velError = targetVelocities[i] - currentVelocities[i];

        // Safety checks
        if (std::abs(posError) > safetyLimits.maxPositionError) {
            disable();
            throw std::runtime_error(
                std::format("Position error {} for joint {} exceeds safety limit {}",
                    posError, i, safetyLimits.maxPositionError)
            );
        }

        // Update integral term with anti-windup
        integralError[i] += posError * elapsedTime;
        integralError[i] = std::clamp(integralError[i],
                                    -maxIntegralError[i],
                                    maxIntegralError[i]);

        // Compute derivative term
        double derivativeError = (posError - lastError[i]) / elapsedTime;
        lastError[i] = posError;

        // PID control law
        commands[i] = kp[i] * posError +
                     ki[i] * integralError[i] +
                     kd[i] * derivativeError;

        // Velocity feedforward
        commands[i] += targetVelocities[i];
    }

    // Apply commands to robot
    try {
        robot.setJointCommands(commands);
    } catch (const std::exception& e) {
        disable();
        throw;
    }

    lastUpdateTime = currentTime;
}

void JointSpaceController::resetInternalState() {
    std::fill(integralError.begin(), integralError.end(), 0.0);
    std::fill(lastError.begin(), lastError.end(), 0.0);
}

// CartesianController.cpp
CartesianController::CartesianController(Robot& robot, double dt)
    : Controller(robot, dt)
    , targetPose(Eigen::Matrix4d::Identity())
    , targetTwist(Eigen::Vector6d::Zero())
    , kp(Eigen::Matrix6d::Identity() * 10.0)  // Default gains
    , ki(Eigen::Matrix6d::Zero())
    , kd(Eigen::Matrix6d::Identity() * 1.0)
    , integralError(Eigen::Vector6d::Zero())
    , lastError(Eigen::Vector6d::Zero())
    , maxIntegralError(Eigen::Vector6d::Ones()) {
}

void CartesianController::setGains(const Eigen::Matrix6d& newKp,
                                 const Eigen::Matrix6d& newKi,
                                 const Eigen::Matrix6d& newKd) {
    // Validate positive semi-definiteness
    Eigen::LLT<Eigen::Matrix6d> lltKp(newKp);
    Eigen::LLT<Eigen::Matrix6d> lltKi(newKi);
    Eigen::LLT<Eigen::Matrix6d> lltKd(newKd);

    if (lltKp.info() == Eigen::NumericalIssue ||
        lltKi.info() == Eigen::NumericalIssue ||
        lltKd.info() == Eigen::NumericalIssue) {
        throw std::invalid_argument("Gain matrices must be positive semi-definite");
    }

    kp = newKp;
    ki = newKi;
    kd = newKd;
    
    // Reset integral term when gains change
    integralError.setZero();
}

void CartesianController::setTargetPose(const Eigen::Matrix4d& target) {
    // Validate rotation part
    Eigen::Matrix3d R = target.block<3,3>(0,0);
    if (!R.isApprox(R * R.transpose(), 1e-10)) {
        throw std::invalid_argument("Target pose rotation matrix must be orthogonal");
    }

    targetPose = target;
}

void CartesianController::setTargetTwist(const Eigen::Vector6d& twist) {
    // Validate against velocity limits
    if (twist.norm() > safetyLimits.maxVelocity) {
        throw std::invalid_argument(
            std::format("Target twist magnitude {} exceeds limit {}",
                twist.norm(), safetyLimits.maxVelocity)
        );
    }

    targetTwist = twist;
}

void CartesianController::update() {
    if (!isEnabled) {
        return;
    }

    double currentTime = getCurrentTime();
    double elapsedTime = currentTime - lastUpdateTime;
    
    if (elapsedTime < dt * 0.5) {
        return;  // Too soon to update
    }

    // Get current end-effector pose and twist
    Eigen::Matrix4d currentPose = robot.getEndEffectorPose();
    Eigen::Vector6d currentTwist = robot.getEndEffectorTwist();

    // Compute pose error (logarithmic map for SO(3))
    Eigen::Vector6d poseError = computePoseError(targetPose, currentPose);
    Eigen::Vector6d twistError = targetTwist - currentTwist;

    // Safety checks
    if (poseError.norm() > safetyLimits.maxPositionError) {
        disable();
        throw std::runtime_error(
            std::format("Pose error magnitude {} exceeds safety limit {}",
                poseError.norm(), safetyLimits.maxPositionError)
        );
    }

    // Update integral term with anti-windup
    integralError += poseError * elapsedTime;
    for (int i = 0; i < 6; ++i) {
        integralError[i] = std::clamp(integralError[i],
                                    -maxIntegralError[i],
                                    maxIntegralError[i]);
    }

    // Compute derivative term
    Eigen::Vector6d derivativeError = (poseError - lastError) / elapsedTime;
    lastError = poseError;

    // Compute desired spatial acceleration
    Eigen::Vector6d command = kp * poseError +
                             ki * integralError +
                             kd * derivativeError;

    // Add feedforward term
    command += targetTwist;

    // Convert to joint commands using inverse dynamics
    try {
        robot.computeAndApplyInverseDynamics(command);
    } catch (const std::exception& e) {
        disable();
        throw;
    }

    lastUpdateTime = currentTime;
}

void CartesianController::resetInternalState() {
    integralError.setZero();
    lastError.setZero();
}

Eigen::Vector6d CartesianController::computePoseError(
    const Eigen::Matrix4d& target,
    const Eigen::Matrix4d& current) {
    
    Eigen::Vector6d error;

    // Position error
    error.tail<3>() = target.block<3,1>(0,3) - current.block<3,1>(0,3);

    // Orientation error (logarithmic map)
    Eigen::Matrix3d R_error = target.block<3,3>(0,0) * current.block<3,3>(0,0).transpose();
    error.head<3>() = rotationMatrixToAxisAngle(R_error);

    return error;
}

Eigen::Vector3d CartesianController::rotationMatrixToAxisAngle(
    const Eigen::Matrix3d& R) {
    
    Eigen::AngleAxisd angleAxis(R);
    return angleAxis.angle() * angleAxis.axis();
}
