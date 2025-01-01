// Link.cpp
#include "../headers/Link.h"
#include <stdexcept>
#include <format>

Link::Link(const std::string& name)
    : Component(name)
    , mass(0.0)
    , centerOfMass(Eigen::Vector3d::Zero())
    , inertia(Eigen::Matrix3d::Identity())
    , transformToParent(Eigen::Matrix4d::Identity())
    , worldTransform(Eigen::Matrix4d::Identity())
    , parentJoint(nullptr)
    , childJoints()
  //  , visualMesh(nullptr)
//    , collisionMesh(nullptr)
{
}

void Link::setMass(double m) {
    if (m < 0) {
        throw std::invalid_argument(
            std::format("Invalid mass {} for link '{}': mass must be non-negative",
                m, name)
        );
    }
    mass = m;
}

double Link::getMass() const {
    return mass;
}

void Link::setCenterOfMass(const Eigen::Vector3d& com) {
    centerOfMass = com;
}

const Eigen::Vector3d& Link::getCenterOfMass() const {
    return centerOfMass;
}

void Link::setInertia(const Eigen::Matrix3d& I) {
    // Verify that the inertia matrix is symmetric
    if (!I.isApprox(I.transpose(), 1e-10)) {
        throw std::invalid_argument(
            std::format("Invalid inertia matrix for link '{}': matrix must be symmetric",
                name)
        );
    }
    
    // Check positive definiteness
    Eigen::LLT<Eigen::Matrix3d> llt(I);
    if (llt.info() == Eigen::NumericalIssue) {
        throw std::invalid_argument(
            std::format("Invalid inertia matrix for link '{}': matrix must be positive definite",
                name)
        );
    }
    
    inertia = I;
}

const Eigen::Matrix3d& Link::getInertia() const {
    return inertia;
}

void Link::setParentJoint(std::shared_ptr<Joint> joint) {
    parentJoint = joint;
    if (joint) {
        joint->setChildLink(this);
    }
}

std::shared_ptr<Joint> Link::getParentJoint() const {
    return parentJoint;
}

void Link::setTransformToParent(const Eigen::Matrix4d& transform) {
    // Verify that the rotation part is orthogonal
    Eigen::Matrix3d rotation = transform.block<3,3>(0,0);
    if (!rotation.isApprox(rotation * rotation.transpose(), 1e-10)) {
        throw std::invalid_argument(
            std::format("Invalid transform for link '{}': rotation matrix must be orthogonal",
                name)
        );
    }
    
    transformToParent = transform;
    updateWorldTransform();
}

const Eigen::Matrix4d& Link::getTransformToParent() const {
    return transformToParent;
}

void Link::setWorldTransform(const Eigen::Matrix4d& transform) {
    worldTransform = transform;
}

const Eigen::Matrix4d& Link::getWorldTransform() const {
    return worldTransform;
}

void Link::resetWorldTransform() {
    worldTransform = Eigen::Matrix4d::Identity();
}

void Link::updateWorldTransform() {
    if (!parentJoint) {
        // This is the base link
        worldTransform = transformToParent;
        return;
    }
    
    // Get parent link's transform
    auto parentLink = parentJoint->getParentLink();
    if (!parentLink) {
        worldTransform = transformToParent;
        return;
    }
    
    // Compute world transform as a chain of transformations
    worldTransform = parentLink->getWorldTransform() *
                    parentJoint->getTransformationMatrix() *
                    transformToParent;
}

Eigen::Vector3d Link::getWorldPosition() const {
    return worldTransform.block<3,1>(0,3);
}

Eigen::Matrix3d Link::getWorldRotation() const {
    return worldTransform.block<3,3>(0,0);
}

Eigen::Vector3d Link::getWorldCenterOfMass() const {
    return (worldTransform * Eigen::Vector4d(centerOfMass.x(),
                                           centerOfMass.y(),
                                           centerOfMass.z(),
                                           1.0)).head<3>();
}

void Link::addChildJoint(std::shared_ptr<Joint> joint) {
    if (!joint) {
        throw std::invalid_argument("Cannot add null joint as child");
    }
    
    // Verify this joint isn't already a child
    auto it = std::find(childJoints.begin(), childJoints.end(), joint);
    if (it != childJoints.end()) {
        throw std::runtime_error(
            std::format("Joint '{}' is already a child of link '{}'",
                joint->getName(), name)
        );
    }
    
    childJoints.push_back(joint);
    joint->setParentLink(this);
}

const std::vector<std::shared_ptr<Joint>>& Link::getChildJoints() const {
    return childJoints;
}

//void Link::setVisualMesh(std::shared_ptr<Mesh> mesh) {
//    visualMesh = mesh;
//}
//
//void Link::setCollisionMesh(std::shared_ptr<Mesh> mesh) {
//    collisionMesh = mesh;
//}
//
//std::shared_ptr<Mesh> Link::getVisualMesh() const {
//    return visualMesh;
//}
//
//std::shared_ptr<Mesh> Link::getCollisionMesh() const {
//    return collisionMesh;
//}

//// Utility methods for spatial inertia calculations
//Eigen::Matrix6d Link::getSpatialInertia() const {
//    Eigen::Matrix6d M = Eigen::Matrix6d::Zero();
//    
//    // Top-left block: rotational inertia
//    M.block<3,3>(0,0) = inertia;
//    
//    // Create skew-symmetric matrix for center of mass
//    Eigen::Matrix3d comSkew;
//    comSkew << 0, -centerOfMass.z(), centerOfMass.y(),
//               centerOfMass.z(), 0, -centerOfMass.x(),
//               -centerOfMass.y(), centerOfMass.x(), 0;
//    
//    // Off-diagonal blocks
//    M.block<3,3>(0,3) = mass * comSkew;
//    M.block<3,3>(3,0) = -mass * comSkew;
//    
//    // Bottom-right block: mass matrix
//    M.block<3,3>(3,3) = mass * Eigen::Matrix3d::Identity();
//    
//    return M;
//}

//bool Link::isInCollision(const Link& other) const {
//    if (!collisionMesh || !other.collisionMesh) {
//        return false;  // No collision checking possible without meshes
//    }
//    
//    // Transform both meshes to world coordinates
//    Eigen::Matrix4d thisTransform = getWorldTransform();
//    Eigen::Matrix4d otherTransform = other.getWorldTransform();
//    
//    // Delegate to mesh collision checking (implementation depends on mesh representation)
//    return collisionMesh->checkCollision(*other.collisionMesh,
//                                       thisTransform,
//                                       otherTransform);
//}
