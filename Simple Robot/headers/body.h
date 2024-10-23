#pragma once

#include <string>
#include "constants.h"
#include "component.h"

class Joint;

class Body: public Compoment {
public:
    Body(std::string fmt, float w, float h);
    
    void update();
    
    Joint* connectedJoint;
};


