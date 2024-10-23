#pragma once
#include "component.h"
#include "constants.h"


class Body;

class Joint: public Compoment {
public:
    Joint(std::string fmt, float radius);
    
    void update();
    
    void connect(Body* b);
    
    float getRadius() const;
    
private:
    float radius = 10.0f;
    Body* connectdeBody;
};
