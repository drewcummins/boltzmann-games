//
//  HingeJoint.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "HingeJoint.hpp"
#include "Utils.hpp"

using namespace bltz;

Constraint HingeJoint::create(Body b1, vec3 r1, vec3 axis, Body b2) {
    Constraint hinge(new HingeJoint(b1, r1, axis, b2));
    return hinge;
}

HingeJoint::HingeJoint(Body b1, vec3 r1, vec3 axis, Body b2) : BallAndSocketJoint(b1, r1, b2) {
    a1 = axis;
    a2 = inverse(b2->R) * (b1->R * a1);
    hasLimits = false;
}

void HingeJoint::setLimits(float minTheta, float maxTheta) {
    this->minTheta = minTheta;
    this->maxTheta = maxTheta;
    q0 = b2->q * inverse(b1->q);
    q0 = normalize(q0);
    hasLimits = true;
}

void HingeJoint::prepare(float dt) {
    BallAndSocketJoint::prepare(dt);
    
    vec3 a1ws = b1->R * a1;
    vec3 a2ws = b2->R * a2;
    
    vec3 b2ws = vec3(-a2ws.y, a2ws.z, a2ws.x);
    vec3 c2ws = cross(a2ws, b2ws);
    b2ws = cross(c2ws, a2ws);
    
    // bastardizing the equation because the linear element is zero
    rqn.J.A1 = cross(b2ws, a1ws);
    rqn.J.A2 = cross(c2ws, a1ws);
    
    float a = dot(rqn.J.A1, b1->invIWorld * rqn.J.A1);
    float b = dot(rqn.J.A1, b1->invIWorld * rqn.J.A2);
    float c = dot(rqn.J.A2, b1->invIWorld * rqn.J.A1);
    float d = dot(rqn.J.A2, b1->invIWorld * rqn.J.A2);
    
    if (!b2->isGround) {
        a += dot(rqn.J.A1, b2->invIWorld * rqn.J.A1);
        b += dot(rqn.J.A1, b2->invIWorld * rqn.J.A2);
        c += dot(rqn.J.A2, b2->invIWorld * rqn.J.A1);
        d += dot(rqn.J.A2, b2->invIWorld * rqn.J.A2);
    }
    
    rqn.K = inverse(mat2(a,b,c,d));
    
    rqn.bias = vec2(dot(a1ws, b2ws), dot(a1ws, c2ws)) * (beta/dt);
    
    if (hasLimits) {
        prepareLimits(dt);
    }
}

void HingeJoint::solve(float dt) {
    BallAndSocketJoint::solve(dt);
    
    vec2 Cdot = vec2(dot(-rqn.J.A1, b1->omega), dot(-rqn.J.A2, b1->omega));
    if (!b2->isGround) {
        Cdot += vec2(dot(rqn.J.A1, b2->omega), dot(rqn.J.A2, b2->omega));
    }
    
    rqn.lambda = -rqn.K * (Cdot + rqn.bias);
    
    b1->omega += -rqn.J.A1 * b1->invIWorld * rqn.lambda[0] - rqn.J.A2 * b1->invIWorld * rqn.lambda[1];
    if (!b2->isGround) {
        b2->omega += rqn.J.A1 * b2->invIWorld * rqn.lambda[0] + rqn.J.A2 * b2->invIWorld * rqn.lambda[1];
    }
    
    if (hasLimits) {
        if (lim1.bias <= 0) {
            solveLimit(lim1, dt);
        } else if (lim2.bias <= 0) {
            solveLimit(lim2, dt);
        }
    }
}


void HingeJoint::prepareLimits(float dt) {
    quat qt = normalize(b2->q * inverse(b1->q));
    quat dq = normalize(qt * inverse(q0));
    vec3 v = vec3(dq.x, dq.y, dq.z);
    vec3 a1ws = b1->R * a1;
    
    float halfcos = dq.w;
    float halfsin = sqrt(dot(v,v));
    float theta;
    
    if (dot(v, a1ws) > 0) {
        theta = 2 * atan2(halfsin, halfcos);
    } else {
        theta = 2 * atan2(halfsin, -halfcos);
    }
    
    theta = fmod(theta, 2*glm::pi<float>());
    
    if (theta < -glm::pi<float>()) {
        theta = theta + 2*glm::pi<float>();
    } else if (theta > glm::pi<float>()) {
        theta = theta - 2*glm::pi<float>();
    }
    
    lim1.J.A1 = -a1ws;
    lim2.J.A1 =  a1ws;
    lim1.K = dot(lim2.J.A1, b1->invIWorld * lim2.J.A1);
    lim1.bias = (theta - minTheta) * (beta/dt);
    lim2.bias = (maxTheta - theta) * (beta/dt);
    
    if (!b2->isGround) {
        lim1.J.A2 =  a1ws;
        lim2.J.A2 = -a1ws;
        lim1.K += dot(lim1.J.A2, b2->invIWorld * lim1.J.A2);
    }
    
    lim2.K = lim1.K;
}

void HingeJoint::solveLimit(C1DOF limit, float dt) {
    float Cdot = dot(limit.J.A1, b1->omega);
    if (!b2->isGround) {
        Cdot += dot(limit.J.A2, b2->omega);
    }
    limit.lambda = -limit.K * (Cdot + limit.bias);
    b1->omega += limit.J.A1 * b1->invIWorld * limit.lambda;
    if (!b2->isGround) {
        b2->omega += limit.J.A2 * b2->invIWorld * limit.lambda;
    }
}

