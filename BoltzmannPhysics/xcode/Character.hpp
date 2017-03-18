//
//  Character.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/15/17.
//
//

#ifndef Character_hpp
#define Character_hpp

#include <stdio.h>
#include "RigidBody.hpp"
#include "Constraints.h"

using namespace ci;
using namespace std;

namespace bltz {
    class Character {
    public:
        Body torso, pelvis, luleg, ruleg, llleg, rlleg;
        Hinge back, lhip, rhip, lknee, rknee;
        void setup(float height, vec3 pelvisX);
        vector<Body> getBones();
        vector<Constraint> getJoints();
    };
}

#endif /* Character_hpp */
