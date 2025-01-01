#pragma once
//#include "constants.h"
#include "component.h"
#include "link.h"
#include "../Eigen/Dense"

//
//class Body;
//
//class Joint: public Compoment {
//public:
//    Joint(std::string fmt, float radius);
//    
//    void update();
//    
//    void connect(Body* b);
//    
//    float getRadius() const;
//    
//private:
//    float radius = 10.0f;
//    Body* connectdeBody;
//};
//


class Joint: public Component {
public:
    enum class Type {
        REVOLUTE,
        PRISMATIC,
        FIXED
    };

    Joint(const std::string& name, Type type);
    virtual ~Joint() = default;

    // Getters and setters
    virtual void setPosition(double pos);
    virtual double getPosition() const;
    virtual void setVelocity(double vel);
    virtual double getVelocity() const;
    virtual void setEffort(double eff);
    virtual double getEffort() const;
    
    // Joint limits
    virtual void setPositionLimits(double min, double max);
    virtual void setVelocityLimit(double limit);
    virtual void setEffortLimit(double limit);
    
    // Type-specific operations
    virtual Eigen::Matrix4d getTransformationMatrix() const = 0;
    
    Type getType() const;
    void setParentLink(std::shared_ptr<Link> parent);
    void setChildLink(std::shared_ptr<Link> child);
    std::shared_ptr<Link> getParentLink() const;
    std::shared_ptr<Link> getChildLink() const;
    
protected:
    Type type;
    double position;    // Current joint position
    double velocity;    // Current joint velocity
    double effort;      // Current joint effort/torque
    double positionLimit[2];  // Min/Max position limits
    double velocityLimit;     // Maximum velocity
    double effortLimit;       // Maximum effort/torque
    std::shared_ptr<Link> parentLink;
    std::shared_ptr<Link> childLink;


};

// RevoluteJoint.hpp
class RevoluteJoint : public Joint {
private:
    Eigen::Vector3d axis;    // Rotation axis

public:
    RevoluteJoint(const std::string& name);
    void setAxis(const Eigen::Vector3d& axis);
    Eigen::Matrix4d getTransformationMatrix() const override;
};

// PrismaticJoint.hpp
class PrismaticJoint : public Joint {
private:
    Eigen::Vector3d axis;    // Translation axis

public:
    PrismaticJoint(const std::string& name);
    void setAxis(const Eigen::Vector3d& axis);
    Eigen::Matrix4d getTransformationMatrix() const override;
};

// Link.hpp
