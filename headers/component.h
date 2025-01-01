#pragma once
#include <string>

class Component {
private:
    std::string name;

public:
    Component(const std::string& name);
    
    const std::string& getName() const;
};
