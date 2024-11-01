#pragma once

#include <vector>


class Body;

class SerialChain {
public:
    SerialChain();
    
    
private:
    Tree chain;
    Body* First;
    Body* last;
    
}
