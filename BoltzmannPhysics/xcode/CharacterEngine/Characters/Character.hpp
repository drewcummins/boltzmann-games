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
            LHIP = 2,
            LHIPLAT = 3,
            LKNEE = 4,
            RHIP = 5,
            RHIPLAT = 6,
            RKNEE = 7
        };
        
        static shared_ptr<Character> create();
        
        float s;
        uint islandId;
        Element lfootElem, rfootElem;
        Body torso, pelvis, luleg, ruleg, llleg, rlleg, torsoLat, lhipLat, rhipLat;
        Hinge back, lhip, rhip, lknee, rknee, backLat, lhipLatJoint, rhipLatJoint;
        shared_ptr<Brain> brain;
        virtual void setup(float height, vec3 pelvisX);
        virtual void update(float dt);
        vec3 com();
        virtual vector<Body> getBones();
        virtual vector<Constraint> getJoints();
        vector<shared_ptr<MotorMuscle>> muscles;
        vec3 leftAnkle();
        vec3 rightAnkle();
        float sy;
        int ticks;
    };
}

#endif /* Character_hpp */
