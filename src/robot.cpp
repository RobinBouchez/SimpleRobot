#include "../headers/robot.h"

#include <stdexcept>
#include <algorithm>
#include <format>

Robot::Robot(const std::string& name) :name(name)
{
    
}

void Robot::addLink(std::shared_ptr<Link> link) {
    if (!link) {
        throw std::invalid_argument("Cannot add null link to robot");
    }

    // Check if link with same name already exists
    auto existingLink = std::find_if(links.begin(), links.end(),
        [&link](const std::shared_ptr<Link>& existing) {
            return existing->getName() == link->getName();
        });

    if (existingLink != links.end()) {
        throw std::runtime_error(
            std::format("Link with name '{}' already exists in robot", link->getName())
        );
    }

    // If this is the first link and no base link is set, make it the base link
    if (links.empty() && !baseLink) {
        baseLink = link;
    }

    links.push_back(link);
}

void Robot::addJoint(std::shared_ptr<Joint> joint) {
    if (!joint) {
        throw std::invalid_argument("Cannot add null joint to robot");
    }

    // Check if joint with same name already exists
    auto existingJoint = std::find_if(joints.begin(), joints.end(),
        [&joint](const std::shared_ptr<Joint>& existing) {
            return existing->getName() == joint->getName();
        });

    if (existingJoint != joints.end()) {
        throw std::runtime_error(
            std::format("Joint with name '{}' already exists in robot", joint->getName())
        );
    }

    // Verify that the joint's parent and child links exist in the robot
    auto parentLink = joint->getParentLink();
    auto childLink = joint->getChildLink();

    if (parentLink) {
        auto foundParent = std::find_if(links.begin(), links.end(),
            [&parentLink](const std::shared_ptr<Link>& link) {
                return link->getName() == parentLink->getName();
            });

        if (foundParent == links.end()) {
            throw std::runtime_error(
                std::format("Parent link '{}' for joint '{}' not found in robot",
                    parentLink->getName(), joint->getName())
            );
        }
    }

    if (childLink) {
        auto foundChild = std::find_if(links.begin(), links.end(),
            [&childLink](const std::shared_ptr<Link>& link) {
                return link->getName() == childLink->getName();
            });

        if (foundChild == links.end()) {
            throw std::runtime_error(
                std::format("Child link '{}' for joint '{}' not found in robot",
                    childLink->getName(), joint->getName())
            );
        }
    }

    joints.push_back(joint);

    // Update the kinematic tree
    updateKinematicTree();
}

void Robot::setBaseLink(std::shared_ptr<Link> base) {
    if (!base) {
        throw std::invalid_argument("Cannot set null base link");
    }

    // Check if the link exists in the robot
    auto foundLink = std::find_if(links.begin(), links.end(),
        [&base](const std::shared_ptr<Link>& link) {
            return link->getName() == base->getName();
        });

    if (foundLink == links.end()) {
        throw std::runtime_error(
            std::format("Cannot set base link '{}': link not found in robot",
                base->getName())
        );
    }

    // Check if the proposed base link has any parent joints
    for (const auto& joint : joints) {
        if (joint->getChildLink() && joint->getChildLink()->getName() == base->getName()) {
            throw std::runtime_error(
                std::format("Cannot set link '{}' as base: link has parent joint '{}'",
                    base->getName(), joint->getName())
            );
        }
    }

    baseLink = base;

    // Update the kinematic tree starting from the new base
    updateKinematicTree();
}

// Private helper method to update the kinematic tree
void Robot::updateKinematicTree() {
    if (!baseLink) {
        return;
    }

    // Reset all transforms
    for (auto& link : links) {
        link->resetWorldTransform();
    }

    // Start from base link and propagate transforms through the tree
    std::function<void(std::shared_ptr<Link>, const Eigen::Matrix4d&)> propagateTransforms;
    propagateTransforms = [this, &propagateTransforms]
        (std::shared_ptr<Link> currentLink, const Eigen::Matrix4d& parentTransform) {
        // Set world transform for current link
        Eigen::Matrix4d currentTransform = parentTransform * currentLink->getTransformToParent();
        currentLink->setWorldTransform(currentTransform);

        // Find all joints that have this link as parent
        for (const auto& joint : joints) {
            if (joint->getParentLink() && joint->getParentLink()->getName() == currentLink->getName()) {
                auto childLink = joint->getChildLink();
                if (childLink) {
                    // Recursively propagate transforms to children
                    Eigen::Matrix4d jointTransform = currentTransform * joint->getTransformationMatrix();
                    propagateTransforms(childLink, jointTransform);
                }
            }
        }
    };

    // Start propagation from base link
    propagateTransforms(baseLink, Eigen::Matrix4d::Identity());
}
