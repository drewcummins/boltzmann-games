//
//  DistanceConstraint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef DistanceConstraint_hpp
#define DistanceConstraint_hpp

#include <stdio.h>
#include "BaseConstraint.hpp"

namespace bltz {

    class DistanceConstraint : public BaseConstraint {
    public:
        static Constraint create(Body b1, vec3 r1, Body b2, vec3 r2);
        virtual void solve(float dt);
        virtual void prepare(float dt);
        virtual void render();
        float distance;
        vec3 r1, r2;
    protected:
        DistanceConstraint(Body b1, vec3 r1, Body b2, vec3 r2);
        C1DOF eqn;
        vec3 r1R, r2R, d;
    };
    
}

#endif /* DistanceConstraint_hpp */
