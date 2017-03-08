//
//  GroundConstraint.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include "GroundConstraint.hpp"

using namespace bltz;

Constraint GroundConstraint::create(Body b1) {
    Constraint ground(new GroundConstraint(b1));
    return ground;
}

GroundConstraint::GroundConstraint(Body b1) : BaseConstraint(b1, NULL) {
    p = vec3(b1->x);
}

void GroundConstraint::prepare(float dt) {
    eqn.J.L1 = mat3();
    eqn.K = inverse(mat3() * b1->invM);
    eqn.bias = (b1->x - p) * (beta/dt);
}

void GroundConstraint::solve(float dt) {
    vec3 Cdot = eqn.J.L1 * b1->v;
    vec3 lambda = -eqn.K * (Cdot - eqn.bias);
    b1->v += eqn.J.L1 * lambda * b1->invM;
}
