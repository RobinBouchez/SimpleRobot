#pragma once
#include "link.h"
#include "joint.h"


class Robot {
private:
    std::string name;
    std::vector<std::shared_ptr<Link>> links;
    std::vector<std::shared_ptr<Joint>> joints;
    std::shared_ptr<Link> baseLink;
    
    void updateKinematicTree();

public:
    Robot(const std::string& name);
    
    void addLink(std::shared_ptr<Link> link);
    void addJoint(std::shared_ptr<Joint> joint);
    void setBaseLink(std::shared_ptr<Link> base);
    
    // Kinematics
    Eigen::Matrix4d getEndEffectorPose() const;
    Eigen::MatrixXd getJacobian() const;
    bool setJointPositions(const std::vector<double>& positions);
    
    // State
    std::vector<double> getJointPositions() const;
    std::vector<double> getJointVelocities() const;
};
