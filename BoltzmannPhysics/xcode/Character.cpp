//
//  Character.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/15/17.
//
//

#include "Character.hpp"
#include "Constants.hpp"

using namespace bltz;

void Character::setup(float height, vec3 pelvisX) {
    
    float s = M2U(height * 0.3);
    
    auto leg = Box::create(vec3(s/3,s,s/3));
    auto foot = bltz::Sphere::create(s/4);
    
    auto pelvisGeom = Box::create(vec3(s*0.75,s/3,s/3));
    auto torsoGeom = Box::create(vec3(s/2,s,s/2));
    
    Material animal;
    animal.density = 1.f;
    animal.friction = 0.96;
    animal.bounciness = 0.f;
    
    luleg = RigidBody::create();
    llleg = RigidBody::create();
    ruleg = RigidBody::create();
    rlleg = RigidBody::create();
    pelvis = RigidBody::create();
    torso = RigidBody::create();
    
    torso->addElement(torsoGeom, animal);
    pelvis->addElement(pelvisGeom, animal);
//    pelvis->isGround = true;
    
//    Muscle abs = SineMuscle::create(torso, vec3(0,s/2,0), pelvis, vec3(0,0,0), 30.f);
//    muscles.push_back(abs);
    
    luleg->addElement(leg, animal);
    ruleg->addElement(leg, animal);
    
    llleg->addElement(leg, animal);
    llleg->addElement(foot, animal, vec3(0,-s/2,0));
    
    rlleg->addElement(leg, animal);
    rlleg->addElement(foot, animal, vec3(0,-s/2,0));
    
    pelvis->setPosition(pelvisX);
    torso->setPosition(pelvisX + vec3(0,s/2 + s/4,0));
    
    luleg->setPosition(pelvisX - vec3(s/2,s/2,0));
    llleg->setPosition(luleg->x - vec3(0,s,0));
    
    ruleg->setPosition(pelvisX + vec3(s/2,-s/2,0));
    rlleg->setPosition(ruleg->x - vec3(0,s,0));
    
//    Muscle lham = SineMuscle::create(luleg, vec3(0,s/2,-s/3), llleg, vec3(0,-s/3,-s/3), 19.f);
//    muscles.push_back(lham);
    
//    Muscle lquad = SineMuscle::create(pelvis, vec3(-s/2,s/6,s/3), luleg, vec3(0,-s/2,s/3), 10.f);
//    muscles.push_back(lquad);
    
    lknee = HingeJoint::create(luleg, vec3(0,-s/2,0), vec3(1,0,0), llleg);
    lknee->setLimits(0, glm::pi<float>()*0.5);
    
    rknee = HingeJoint::create(ruleg, vec3(0,-s/2,0), vec3(1,0,0), rlleg);
    rknee->setLimits(0, glm::pi<float>()*0.5);
    
    lhip = HingeJoint::create(pelvis, vec3(-s*0.75/2,0,0), vec3(1,0,0), luleg);
    lhip->setLimits(-glm::pi<float>()*0.5, glm::pi<float>()*0.25);
    
    rhip = HingeJoint::create(pelvis, vec3(s*0.75/2,0,0), vec3(1,0,0), ruleg);
    rhip->setLimits(-glm::pi<float>()*0.5, glm::pi<float>()*0.25);
    
    back = HingeJoint::create(torso, vec3(0,-s/2,0), vec3(1,0,0), pelvis);
    back->setLimits(-glm::pi<float>()*0.15, glm::pi<float>()*0.15);
    
    auto motor = MotorMuscle::create(lknee);
    motors.push_back(motor);
    
    motor = MotorMuscle::create(lhip);
    motors.push_back(motor);
    
    motor = MotorMuscle::create(rhip);
    motors.push_back(motor);
    
    motor = MotorMuscle::create(rknee);
    motors.push_back(motor);
    
    motor = MotorMuscle::create(back);
    motors.push_back(motor);
    
}

vector<Body> Character::getBones() {
    return {torso, pelvis, luleg, ruleg, llleg, rlleg};
}

vector<Constraint> Character::getJoints() {
    return {lhip, rhip, lknee, rknee, back};
}

void Character::update(float dt) {
    for (auto &muscle : muscles) {
        muscle->update(dt);
    }
    
    for (auto &motor : motors) {
        motor->update(dt);
    }
}




