//
//  GroundConstraint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef GroundConstraint_hpp
#define GroundConstraint_hpp

#include <stdio.h>
#include "BaseConstraint.hpp"

namespace bltz {

    class GroundConstraint : public BaseConstraint {
    public:
        static Constraint create(Body b1);
        vec3 p; // position to fix to
        virtual void solve(float dt);
        virtual void prepare(float dt);
    protected:
        C3DOF eqn;
        GroundConstraint(Body b1);
    };
        
}

#endif /* GroundConstraint_hpp */
