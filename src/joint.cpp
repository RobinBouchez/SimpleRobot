#include "../headers/joint.h"
#include <stdexcept>
#include <format>
#include <cmath>

Joint::Joint(const std::string& name, Type type)
    : Component(name)
    , type(type)
    , position(0.0)
    , velocity(0.0)
    , effort(0.0)
    , positionLimit{-M_PI, M_PI}  // Default limits for revolute joints
    , velocityLimit(M_PI)         // Default velocity limit of 180 deg/s
    , effortLimit(100.0)          // Default torque/force limit
    , parentLink(nullptr)
    , childLink(nullptr) {
    
    // Adjust limits based on joint type
    if (type == Type::PRISMATIC) {
        positionLimit[0] = -1.0;  // Default -1m
        positionLimit[1] = 1.0;   // Default 1m
        velocityLimit = 1.0;      // Default 1m/s
    }
}

void Joint::setPosition(double pos) {
    // Check position limits
    if (type != Type::FIXED) {
        if (pos < positionLimit[0] || pos > positionLimit[1]) {
            throw std::runtime_error(
                std::format("Joint position {} exceeds limits [{}, {}] for joint '{}'",
                    pos, positionLimit[0], positionLimit[1], name)
            );
        }
        position = pos;
    }
}

double Joint::getPosition() const {
    return position;
}

void Joint::setVelocity(double vel) {
    // Check velocity limits
    if (type != Type::FIXED) {
        if (std::abs(vel) > velocityLimit) {
            throw std::runtime_error(
                std::format("Joint velocity {} exceeds limit {} for joint '{}'",
                    vel, velocityLimit, name)
            );
        }
        velocity = vel;
    }
}

double Joint::getVelocity() const {
    return velocity;
}

void Joint::setEffort(double eff) {
    // Check effort limits
    if (type != Type::FIXED) {
        if (std::abs(eff) > effortLimit) {
            throw std::runtime_error(
                std::format("Joint effort {} exceeds limit {} for joint '{}'",
                    eff, effortLimit, name)
            );
        }
        effort = eff;
    }
}

double Joint::getEffort() const {
    return effort;
}

void Joint::setPositionLimits(double min, double max) {
    if (min >= max) {
        throw std::invalid_argument(
            std::format("Invalid position limits [{}, {}] for joint '{}': min must be less than max",
                min, max, name)
        );
    }
    positionLimit[0] = min;
    positionLimit[1] = max;
    
    // Clamp current position to new limits
    if (position < min) position = min;
    if (position > max) position = max;
}

void Joint::setVelocityLimit(double limit) {
    if (limit <= 0) {
        throw std::invalid_argument(
            std::format("Invalid velocity limit {} for joint '{}': must be positive",
                limit, name)
        );
    }
    velocityLimit = limit;
    
    // Clamp current velocity to new limits
    if (std::abs(velocity) > limit) {
        velocity = std::copysign(limit, velocity);
    }
}

void Joint::setEffortLimit(double limit) {
    if (limit <= 0) {
        throw std::invalid_argument(
            std::format("Invalid effort limit {} for joint '{}': must be positive",
                limit, name)
        );
    }
    effortLimit = limit;
    
    // Clamp current effort to new limits
    if (std::abs(effort) > limit) {
        effort = std::copysign(limit, effort);
    }
}


Joint::Type Joint::getType() const {
    return type;
}

void Joint::setParentLink(std::shared_ptr<Link> parent) {
    parentLink = parent;
}

void Joint::setChildLink(std::shared_ptr<Link> child) {
    childLink = child;
}

std::shared_ptr<Link> Joint::getParentLink() const {
    return parentLink;
}

std::shared_ptr<Link> Joint::getChildLink() const {
    return childLink;
}

// Derived class implementations

RevoluteJoint::RevoluteJoint(const std::string& name)
    : Joint(name, Type::REVOLUTE)
    , axis(Eigen::Vector3d::UnitZ()) {  // Default rotation around Z axis
}

void RevoluteJoint::setAxis(const Eigen::Vector3d& axis) {
    if (axis.isZero(1e-10)) {
        throw std::invalid_argument("Rotation axis cannot be zero vector");
    }
    this->axis = axis.normalized();
}

Eigen::Matrix4d RevoluteJoint::getTransformationMatrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // Create rotation matrix using Rodrigues' formula
    double c = std::cos(position);
    double s = std::sin(position);
    double t = 1.0 - c;
    
    double x = axis.x();
    double y = axis.y();
    double z = axis.z();
    
    T(0,0) = t*x*x + c;    T(0,1) = t*x*y - z*s;  T(0,2) = t*x*z + y*s;
    T(1,0) = t*x*y + z*s;  T(1,1) = t*y*y + c;    T(1,2) = t*y*z - x*s;
    T(2,0) = t*x*z - y*s;  T(2,1) = t*y*z + x*s;  T(2,2) = t*z*z + c;
    
    return T;
}

PrismaticJoint::PrismaticJoint(const std::string& name)
    : Joint(name, Type::PRISMATIC)
    , axis(Eigen::Vector3d::UnitX()) {  // Default translation along X axis
}

void PrismaticJoint::setAxis(const Eigen::Vector3d& axis) {
    if (axis.isZero(1e-10)) {
        throw std::invalid_argument("Translation axis cannot be zero vector");
    }
    this->axis = axis.normalized();
}

Eigen::Matrix4d PrismaticJoint::getTransformationMatrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // Translation along the axis
    T.block<3,1>(0,3) = axis * position;
    
    return T;
}
