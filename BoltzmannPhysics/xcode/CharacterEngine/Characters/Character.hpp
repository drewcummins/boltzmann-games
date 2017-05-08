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
#include "Muscle.hpp"
#include "Brain.hpp"

using namespace ci;
using namespace std;

namespace bltz {
    class Character {
    public:
        
        enum MUSCLE {
            BACKLAT = 0,
            BACK = 1,
            LHIPLAT = 2,
            LHIP = 3,
            LKNEE = 4,
            RHIPLAT = 5,
            RHIP = 6,
            RKNEE = 7
        };
        
        uint islandId;
        Body torso, pelvis, luleg, ruleg, llleg, rlleg, torsoLat, lhipLat, rhipLat;
        Hinge back, lhip, rhip, lknee, rknee, backLat, lhipLatJoint, rhipLatJoint;
        shared_ptr<Brain> brain;
        virtual void setup(float height, vec3 pelvisX);
        virtual void update(float dt);
        vector<Body> getBones();
        vector<Constraint> getJoints();
        vector<shared_ptr<MotorMuscle>> muscles;
        float sy;
        int ticks;
    };
}

#endif /* Character_hpp */
