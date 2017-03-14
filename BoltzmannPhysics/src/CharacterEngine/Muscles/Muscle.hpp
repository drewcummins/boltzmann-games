//
//  Muscle.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#ifndef Muscle_hpp
#define Muscle_hpp

#include <stdio.h>
#include "RigidBody.hpp"
#include "Force.hpp"

using namespace std;

namespace bltz {
    class BaseMuscle {
    public:
        virtual void update(float dt) = 0;
        virtual float getMuscleLength(vec3 &x1p, vec3 &x2p);
        Body bone1, bone2;
        vec3 x1, x2; // position of muscle attachment to bone
    protected:
        BaseMuscle(Body b1, vec3 x1, Body b2, vec3 x2);
        virtual void actuate() = 0;
    };
    
    typedef shared_ptr<BaseMuscle> Muscle;
    
    class SimpleMuscle : public BaseMuscle {
    public:
        static Muscle create(Body b1, vec3 x1, Body b2, vec3 x2, float k);
        virtual void update(float dt);
        float restLength;
        float targetLength;
    protected:
        SimpleMuscle(Body b1, vec3 x1, Body b2, vec3 x2, float k);
        float stiffness;
        virtual void actuate();
    };
    
    class SineMuscle : public SimpleMuscle {
    public:
        static Muscle create(Body b1, vec3 x1, Body b2, vec3 x2, float k, float sineCounter=0);
        float sineCounter;
        virtual void update(float dt);
    protected:
        SineMuscle(Body b1, vec3 x1, Body b2, vec3 x2, float k, float sineCounter=0);
    };
    
}



#endif /* Muscle_hpp */
