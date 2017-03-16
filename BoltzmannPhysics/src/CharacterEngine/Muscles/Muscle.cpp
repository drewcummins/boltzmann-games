//
//  Muscle.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#include "Muscle.hpp"

using namespace bltz;

BaseMuscle::BaseMuscle(Body b1, vec3 x1, Body b2, vec3 x2) : bone1(b1), bone2(b2) {
    this->x1 = b1->getPositionInCOMSpace(x1);
    this->x2 = b2->getPositionInCOMSpace(x2);
}

float BaseMuscle::getMuscleLength(vec3 &x1p, vec3 &x2p) {
    x1p = bone1->com + bone1->R * x1;
    x2p = bone2->com + bone2->R * x2;
    return length(x2p - x1p);
}



Muscle SimpleMuscle::create(Body b1, vec3 x1, Body b2, vec3 x2, float k) {
    return Muscle(new SimpleMuscle(b1, x1, b2, x2, k));
}

SimpleMuscle::SimpleMuscle(Body b1, vec3 x1, Body b2, vec3 x2, float k) : BaseMuscle(b1, x1, b2, x2), stiffness(k) {}

void SimpleMuscle::actuate() {
    vec3 x1ws, x2ws;
    float len = getMuscleLength(x1ws, x2ws);
    float dl = targetLength - len;
    vec3 dx = x2ws - x1ws;
    dx = normalize(dx);
    bone1->addForceAtPoint(x1ws, dx * dl * stiffness);
    bone2->addForceAtPoint(x2ws,-dx * dl * stiffness);
}

void SimpleMuscle::update(float dt) {
    
}
