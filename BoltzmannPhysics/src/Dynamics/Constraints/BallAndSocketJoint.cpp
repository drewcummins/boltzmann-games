//
//  BallAndSocketJoint.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "BallAndSocketJoint.hpp"
#include "Utils.hpp"

using namespace bltz;

Constraint BallAndSocketJoint::create(Body b1, vec3 r1, Body b2) {
    Constraint joint(new BallAndSocketJoint(b1, r1, b2));
    return joint;
}

BallAndSocketJoint::BallAndSocketJoint(Body b1, vec3 r1, Body b2) : BaseConstraint(b1, b2), r1(r1) {
    this->r1 += b1->xModel; // move to center of mass frame
    r2 = (b1->com + b1->R * this->r1 - b2->com) * inverse(b2->R);
    eqn.lambda = vec3(0,0,0);
}

void BallAndSocketJoint::prepare(float dt) {
    r1R = b1->R * r1;
    r2R = b2->R * r2;
    
    eqn.bias = (b2->com + r2R - b1->com - r1R) * (beta/dt);
    
    eqn.J.L1 = mat3();
    eqn.J.A1 = Utils::skew(r1R);
    
    eqn.K = b1->invM * mat3() + eqn.J.A1 * b1->invIWorld * -eqn.J.A1;
    
    if (!b2->isGround) {
        eqn.J.L2 = mat3();
        eqn.J.A2 = Utils::skew(r2R);
        eqn.K += b2->invM * mat3() + eqn.J.A2 * b2->invIWorld * -eqn.J.A2;
    }
    
    eqn.K = inverse(eqn.K);
}


void BallAndSocketJoint::solve(float dt) {
    vec3 Cdot = -eqn.J.L1 * b1->v - eqn.J.A1 * b1->omega;
    if (!b2->isGround) {
        Cdot += eqn.J.L2 * b2->v + eqn.J.A2 * b2->omega;
    }
    
    vec3 lambda = -eqn.K * (Cdot + eqn.bias);
    eqn.lambda += lambda;
    
    b1->v -= b1->invM * lambda;
    b1->omega += b1->invIWorld * eqn.J.A1 * lambda;
    
    if (!b2->isGround) {
        b2->v += b2->invM * lambda;
        b2->omega -= b2->invIWorld * eqn.J.A2 * lambda;
    }
}

void BallAndSocketJoint::render() {
    gl::color(0.2, 0.2, 1.0);
    gl::lineWidth(0.05);
    
    r1R = b1->R * r1;
    
    gl::pushModelMatrix();
    gl::translate(b1->com + r1R);
    gl::drawSphere(vec3(), 0.015f);
    gl::popModelMatrix();
    
    r2R = b2->R * r2;
    
    gl::pushModelMatrix();
    gl::translate(b2->com + r2R);
    gl::drawSphere(vec3(), 0.015f);
    gl::popModelMatrix();
}

