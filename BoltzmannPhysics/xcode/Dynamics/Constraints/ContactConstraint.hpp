//
//  ContactConstraint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#ifndef ContactConstraint_hpp
#define ContactConstraint_hpp

#include <stdio.h>
#include "BaseConstraint.hpp"
#include "CollisionGeometry.hpp"

namespace bltz {

    class ContactConstraint : public BaseConstraint {
    public:
        static Constraint create(Contact contact);
        virtual void solve(float dt);
        virtual void prepare(float dt);
    protected:
        ContactConstraint(Contact contact);
        Contact contact;
        vector<C1DOF> n;
        vector<C1DOF> u1;
        vector<C1DOF> u2;
        vector<C1DOF> eqns;
        float nLambda = 0.f;
        void buildNonpenetrationConstraint(float dt, int i);
        void buildFrictionConstraint(float dt, int i, vec3 u, vector<C1DOF> &eqns);
        void solveNonpenetrationConstraint(float dt, int i);
        void solveFrictionConstraint(float dt, int i, vec3 u, vector<C1DOF> &eqns);
    };
    
}

#endif /* ContactConstraint_hpp */
