//
//  Constants.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/18/17.
//
//

#ifndef Constants_hpp
#define Constants_hpp

#include <stdio.h>

namespace bltz {
    const float UNITS_TO_METERS = 0.2;
    const float METERS_TO_UNITS = 5.0;
    inline float U2M(float units) { return units * UNITS_TO_METERS; }
    inline float M2U(float meters) { return meters * METERS_TO_UNITS; }
}




#endif /* Constants_hpp */
