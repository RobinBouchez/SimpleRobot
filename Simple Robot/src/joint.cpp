#pragma once
#include "joint.h"
#include "body.h"


Joint::Joint(std::string fmt, float radius):
Compoment(fmt, radius, radius) {}

void Joint::update() {
    ImGui::Begin(name.c_str());
    
    ImGui::BeginDisabled(fixed || isChild);
    ImGui::SliderFloat("x", &x, 0.0f, SCREEN_WIDTH / 2);
    ImGui::SliderFloat("y", &y, 0.0f, SCREEN_HEIGHT);
    ImGui::EndDisabled();
    
    ImGui::SliderFloat("Radius", &radius, 10.0f, 100.0f);
    ImGui::SliderFloat("Angle", &angle, 0.0f, EIGEN_PI * 2);
    ImGui::Spacing();
    
    
    
    
//    Eigen::Vector2f pivot(100, 100); // Rotate around this point
//    Eigen::Vector2f center(x, y);
//    
//    Eigen::Matrix2f rotation;
//    rotation << std::cos(angle), -std::sin(angle),
//    std::sin(angle),  std::cos(angle);
//    
//    
//    Eigen::Vector2f translatedCenter = center - pivot;
//    
//    Eigen::Vector2f rotatedCenter = rotation * translatedCenter;
//    
//    Eigen::Vector2f finalCenter = rotatedCenter + pivot;
//    
//    ImGui::Text("Pivot x %f", pivot.x());
//    ImGui::Text("Pivot y %f", pivot.y());
//    
//    
//    setPosition(finalCenter.x(), finalCenter.y());
//    
    
    if(connectdeBody != nullptr) {
        Eigen::Matrix<float, 2, 4> rectangle;
        rectangle << connectdeBody->getX() -  connectdeBody->getWidth()  / 2, connectdeBody->getX() +  connectdeBody->getWidth()  / 2, connectdeBody->getX() +  connectdeBody->getWidth()  / 2, connectdeBody->getX() - connectdeBody->getWidth() / 2,
        connectdeBody->getY() - connectdeBody->getHeight() / 2, connectdeBody->getY() -  connectdeBody->getHeight()  / 2, connectdeBody->getY() + connectdeBody->getHeight() / 2, connectdeBody->getY() + connectdeBody->getHeight() / 2;
        
        
        Eigen::Matrix2f rotation;
        rotation << std::cos(angle), -std::sin(angle),
        std::sin(angle),  std::cos(angle);
        
        setPivot(Eigen::Vector2f(x, y));
        
        Eigen::Matrix<float, 2, 4> translatedRectangle = rectangle.colwise() - pivot;
        
        Eigen::Matrix<float, 2, 4> rotatedRectangle = rotation * translatedRectangle;
        
        Eigen::Matrix<float, 2, 4> finalRectangle = rotatedRectangle.colwise() + pivot;
        
        ImGui::Text("Pivot x %f", pivot.x());
        ImGui::Text("Pivot y %f", pivot.y());
        
        
        connectdeBody->setShape(ImVec2(finalRectangle(0, 0), finalRectangle(1, 0)),
                       ImVec2(finalRectangle(0, 1), finalRectangle(1, 1)),
                       ImVec2(finalRectangle(0, 2), finalRectangle(1, 2)),
                       ImVec2(finalRectangle(0, 3), finalRectangle(1, 3)));
        
        connectdeBody->setPosition(x, y + connectdeBody->getHeight()  / 2);
        
    }
    ImGui::End();
//    
//    for (Compoment* child: children){
//        child->setPosition(this->shape.bottomRight.x + abs(this->shape.bottomLeft.x - this->shape.bottomRight.x) / 2,
//                           this->shape.bottomLeft.y + abs(this->shape.bottomLeft.y - this->shape.bottomRight.y) / 2);
//        child->setPivot(Eigen::Vector2f(x, y));
//    }
    
}

void Joint::connect(Body* b) {
    connectdeBody = b;
    
}


float Joint::getRadius() const { return radius; }
