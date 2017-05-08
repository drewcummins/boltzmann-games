//
//  Force.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#ifndef Force_hpp
#define Force_hpp

#include <stdio.h>

class Force {
public:
    virtual void exert(float dt) = 0;
};

#endif /* Force_hpp */
