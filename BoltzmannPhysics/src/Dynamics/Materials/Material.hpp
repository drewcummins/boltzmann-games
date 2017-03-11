//
//  Material.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/11/17.
//
//

#ifndef Material_hpp
#define Material_hpp

#include <stdio.h>

namespace bltz {
    typedef struct Material {
        float density;
        float friction;
        float bounciness;
    } Material;
}

#endif /* Material_hpp */
