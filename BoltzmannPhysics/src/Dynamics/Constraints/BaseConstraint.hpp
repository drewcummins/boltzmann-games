//
//  BaseConstraint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef BaseConstraint_hpp
#define BaseConstraint_hpp

#include <stdio.h>
#include "RigidBody.hpp"

namespace bltz {

    template <class T>
    struct Jacobian {
        T L1;
        T L2;
        T A1;
        T A2;
    };

    template <class T1, class T2, class T3>
    struct ConstraintEquation {
        Jacobian<T1> J;
        T2 K;
        T3 bias;
        T3 lambda;
    };

    typedef ConstraintEquation<vec3, float, float> C1DOF;
    typedef ConstraintEquation<vec3, mat2, vec2> C2DOF;
    typedef ConstraintEquation<mat3, mat3, vec3> C3DOF;

    class BaseConstraint {
    public:
        uint id;
        Body b1, b2;
        float beta = 0.2f;
        virtual void solve(float dt) {};
        virtual void prepare(float dt) {};
        virtual void render() {};
    protected:
        BaseConstraint(Body b1, Body b2);
    };
    
    typedef shared_ptr<BaseConstraint> Constraint;
    
}

#endif /* BaseConstraint_hpp */
