#pragma once
#include "component.h"

Compoment::Compoment(std::string fmt, float w, float h):
    width(w), height(h), name(fmt),  x(0), y(0)
    {
        color = IM_COL32(rand() % 255, rand() % 255, rand() % 255, 255);
    }
    
    
    int Compoment::getWidth() const { return width; }
    int Compoment::getHeight() const { return height; }
    float Compoment::getX() const { return x; }
    float Compoment::getY() const { return y; }
    float Compoment::getAngle() const { return angle; }
    Shape Compoment::getShape() {
        return shape;
    }
    
    void Compoment::setPosition(float newX, float newY) {
        x = newX;
        y = newY;
    }
    
    void Compoment::setAngle(float newAngle) { angle = newAngle; }
    void Compoment::setShape(ImVec2 newtopLeft, ImVec2 newtopRight, ImVec2 newbottomRight, ImVec2 newbottomLeft) {
        shape.topLeft = newtopLeft;
        shape.topRight = newtopRight;
        shape.bottomLeft = newbottomLeft;
        shape.bottomRight = newbottomRight;
    }
    
    std::string Compoment::getName() const { return name; }
    
    ImU32 Compoment::getColor() const { return color; }

    void Compoment::setPivot(Eigen::Vector2f newPivot) { pivot = newPivot; }
