//
//  BallAndSocketJoint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#ifndef BallAndSocketJoint_hpp
#define BallAndSocketJoint_hpp

#include <stdio.h>
#include "BaseConstraint.hpp"

namespace bltz {

    class BallAndSocketJoint : public BaseConstraint {
    public:
        static Constraint create(Body b1, vec3 r1, Body b2);
        virtual void solve(float dt);
        virtual void prepare(float dt);
        vec3 r1, r2;
    protected:
        BallAndSocketJoint(Body b1, vec3 r1, Body b2);
        C3DOF eqn;
        vec3 r1R, r2R, d;
    };
    
}

#endif /* BallAndSocketJoint_hpp */
