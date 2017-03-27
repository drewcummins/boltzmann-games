//
//  Muscle.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#include "Muscle.hpp"
#include "Utils.hpp"

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



shared_ptr<SimpleMuscle> SimpleMuscle::create(Body b1, vec3 x1, Body b2, vec3 x2, float k) {
    return shared_ptr<SimpleMuscle>(new SimpleMuscle(b1, x1, b2, x2, k));
}

SimpleMuscle::SimpleMuscle(Body b1, vec3 x1, Body b2, vec3 x2, float k) : BaseMuscle(b1, x1, b2, x2), stiffness(k) {
    vec3 u1, u2;
    restLength = getMuscleLength(u1, u2);
}

void SimpleMuscle::actuate() {
    vec3 x1ws, x2ws;
    float len = getMuscleLength(x1ws, x2ws);
    cout << " - " << len << endl;
    float dl = targetLength - len;
    vec3 dx = x2ws - x1ws;
    dx = normalize(dx);
    vec3 force = dx * dl * stiffness;
    
    bone1->addForce(force);
    bone1->addTorque(cross(force, x1));
    
    bone2->addForce(-force);
    bone2->addTorque(cross(-force, x2));
    
//    bone1->addForceAtPoint(x1ws, dx * dl * stiffness);
//    bone2->addForceAtPoint(x2ws,-dx * dl * stiffness);
}

void SimpleMuscle::update(float dt) {
    actuate();
}



shared_ptr<SineMuscle> SineMuscle::create(Body b1, vec3 x1, Body b2, vec3 x2, float k, float sineCounter) {
    return shared_ptr<SineMuscle>(new SineMuscle(b1, x1, b2, x2, k));
}

SineMuscle::SineMuscle(Body b1, vec3 x1, Body b2, vec3 x2, float k, float sineCounter) : SimpleMuscle(b1, x1, b2, x2, k) {}

void SineMuscle::update(float dt) {
    sineCounter += dt*2;
    float t = sin(sineCounter);
    targetLength = restLength - 0.2 + t * restLength * 0.2;
    SimpleMuscle::update(dt);
}


shared_ptr<MotorMuscle> MotorMuscle::create(shared_ptr<HingeJoint> joint) {
    return shared_ptr<MotorMuscle>(new MotorMuscle(joint));
}

MotorMuscle::MotorMuscle(shared_ptr<HingeJoint> joint) {
    this->joint = joint;
    t = 0.5;
}

void MotorMuscle::update(float dt) {
    float thetaRange = joint->maxTheta - joint->minTheta;
    float target = joint->minTheta + thetaRange * t;;
    float current = joint->cacheTheta();
    float speed = (target - current)/(dt*5.f);
//    cout << speed << endl;
    joint->setMotor(speed);
}



