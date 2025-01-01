#pragma once
#include "../Eigen/dense"

#include "component.h"
#include "joint.h"


class Link: public Component {
private:
    std::string name;
    double mass;
    Eigen::Vector3d centerOfMass;
    Eigen::Matrix3d inertia;
    std::shared_ptr<Joint> parentJoint;
    Eigen::Matrix4d worldTransform;
    Eigen::Matrix4d transformToParent;
    std::vector<std::shared_ptr<Joint>> childJoints;
    //std::shared_ptr<Mesh> visualMesh;
    //std::shared_ptr<Mesh> collisionMesh;

public:
    Link(const std::string& name);
    
    double getMass() const;
    const Eigen::Vector3d& getCenterOfMass() const;
    const Eigen::Matrix3d& getInertia() const;
    std::shared_ptr<Joint> getParentJoint() const;
    const Eigen::Matrix4d& getTransformToParent() const;
    const Eigen::Matrix4d& getWorldTransform() const;
    Eigen::Vector3d getWorldCenterOfMass() const;
    const std::vector<std::shared_ptr<Joint>>& getChildJoints() const;
    
    
    Eigen::Vector3d getWorldPosition() const;
    Eigen::Matrix3d getWorldRotation() const;
    void setWorldTransform(const Eigen::Matrix4d& transform);
    void resetWorldTransform() ;
    void updateWorldTransform();

    void setMass(double mass);
    void setCenterOfMass(const Eigen::Vector3d& com);
    void setInertia(const Eigen::Matrix3d& inertia);
    void setParentJoint(std::shared_ptr<Joint> joint);
    void addChildJoint(std::shared_ptr<Joint> joint);
    void setTransformToParent(const Eigen::Matrix4d& transform);
    

};
