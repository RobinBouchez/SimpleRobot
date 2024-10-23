#pragma once

#include <string>
#include <vector>
#include <stdio.h>

#include "../imgui/imgui.h"
#include "../imgui/backends/imgui_impl_sdl2.h"
#include "../imgui/backends/imgui_impl_opengl2.h"

#include "../Eigen/Dense"

struct Shape{
    ImVec2 topLeft, topRight, bottomLeft, bottomRight;
};


class Compoment {
public:
    Compoment(std::string fmt, float w, float h);
    
    
    int getWidth() const;
    int getHeight() const;
    float getX() const;
    float getY() const;
    float getAngle() const;
    Shape getShape();
    
    bool fixed = false;
    bool isChild = false;
    
    
    void setPosition(float newX, float newY);
    void setAngle(float newAngle);
    void setShape(ImVec2 newtopLeft, ImVec2 newtopRight, ImVec2 newbottomRight, ImVec2 newbottomLeft);
    
    std::string getName() const;
    
    ImU32 getColor() const;
    float angle;
    
    Eigen::Vector2f pivot;
    
    void setPivot(Eigen::Vector2f newPivot);
    
    std::vector<Compoment*> children;
    
protected:
    Shape shape;
    ImU32 color;
    std::string name;
    float width;
    float height;
    float x;
    float y;
};
