//
//  DistanceConstraint.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include "DistanceConstraint.hpp"

using namespace bltz;


DistanceConstraint::DistanceConstraint(Body b1, vec3 r1, Body b2, vec3 r2) : BaseConstraint(b1, b2), r1(r1), r2(r2) {
    vec3 r1ws = b1->x + b1->R * r1;
    vec3 r2ws = b2->x + b2->R * r2;
    
    distance = length(r2ws - r1ws);
}

void DistanceConstraint::prepare(float dt) {
    // attachment points rotated to be in world space
    r1R = b1->R * r1;
    r2R = b2->R * r2;
    
    // axis between these points
    d = b2->x + r2R - b1->x - r1R;
    
    // C    = x1 + r1 - x2 - r2 = 0
    // C'   = v1 . -d - ω1 x r1 x d - v2 . d + ω2 x r2 x d = 0
    
    // bias = beta * C
    eqn.bias = (distance - length(d)) * (beta/dt);
    
    // body 1 linear Jacobian
    eqn.J.L1 = -d;
    // body 1 angular Jacobian
    eqn.J.A1 = -cross(r1R, d);
    
    eqn.K  = dot(eqn.J.L1 * b1->invM, eqn.J.L1) + dot(eqn.J.A1 * b1->invIWorld, eqn.J.A1);
    
    if (!b2->isGround) {
        // body 2 linear Jacobian
        eqn.J.L2 = d;
        // body 2 angular Jacobian
        eqn.J.A2 = cross(r2R, d);
        
        eqn.K += dot(eqn.J.L2 * b2->invM, eqn.J.L2) + dot(eqn.J.A2 * b2->invIWorld, eqn.J.A2);
    }
    
    eqn.K = 1 / eqn.K;
}

void DistanceConstraint::solve(float dt) {
    float Cdot = dot(eqn.J.L1, b1->v) + dot(eqn.J.A1, b1->omega);
    
    if (!b2->isGround) {
        Cdot += dot(eqn.J.L2, b2->v) + dot(eqn.J.A2, b2->omega);
    }
    
    // this is the impulse "amount"
    float lambda = -eqn.K * (Cdot - eqn.bias);
    
    b1->v += b1->invM * eqn.J.L1 * lambda;
    b1->omega += b1->invIWorld * eqn.J.A1 * lambda;
    
    if (!b2->isGround) {
        b2->v += b2->invM * eqn.J.L2 * lambda;
        b2->omega += b2->invIWorld * eqn.J.A2 * lambda;
    }
}

void DistanceConstraint::render() {
    gl::color(0.5, 0.5, 1.0);
    gl::lineWidth(5);
    vec3 r1ws = b1->x + b1->R * r1;
    vec3 r2ws = b2->x + b2->R * r2;
    gl::drawLine(r1ws, r2ws);
}



