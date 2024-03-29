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

shared_ptr<Character> Character::create() {
    return shared_ptr<Character>(new Character);
}

void Character::setup(float height, vec3 pelvisX) {
    
    s = M2U(height * 0.3);
    
    auto leg = Box::create(vec3(s/3,s,s/3));
    auto foot = bltz::Sphere::create(s/4);
    
    auto pelvisGeom = Box::create(vec3(s*0.75,s/3,s/3));
    auto torsoGeom = Box::create(vec3(s/2,s,s/2));
    auto torsoLatGeom = bltz::Sphere::create(s/4);
    
    Material animal;
    animal.density = 3.f;
    animal.friction = 0.96;
    animal.bounciness = 0.f;
    
    luleg = RigidBody::create();
    llleg = RigidBody::create();
    ruleg = RigidBody::create();
    rlleg = RigidBody::create();
    pelvis = RigidBody::create();
    torso = RigidBody::create();
    torsoLat = RigidBody::create();
    lhipLat = RigidBody::create();
    rhipLat = RigidBody::create();
    
    Material lt = {1.f,0.96,0.f};
    
    torso->addElement(torsoGeom, lt);
    torsoLat->addElement(torsoLatGeom, lt);
    lhipLat->addElement(torsoLatGeom, lt);
    rhipLat->addElement(torsoLatGeom, lt);
    
    pelvis->addElement(pelvisGeom, animal);
//    pelvis->isGround = true;
    
//    Muscle abs = SineMuscle::create(torso, vec3(0,s/2,0), pelvis, vec3(0,0,0), 30.f);
//    muscles.push_back(abs);
    
    luleg->addElement(leg, animal);
    ruleg->addElement(leg, animal);
    
    llleg->addElement(leg, animal);
    lfootElem = llleg->addElement(foot, animal, vec3(0,-s/2,0));
    
    rlleg->addElement(leg, animal);
    rfootElem = rlleg->addElement(foot, animal, vec3(0,-s/2,0));
    
    pelvis->setPosition(pelvisX);
    torso->setPosition(pelvisX + vec3(0,s/2 + s/4,0));
    torsoLat->setPosition(pelvisX + vec3(0,s/2,0));
    
    luleg->setPosition(pelvisX + vec3(s/2,-s/2,0));
    llleg->setPosition(luleg->x + vec3(0,-s,0));
    
    ruleg->setPosition(pelvisX + vec3(-s/2,-s/2,0));
    rlleg->setPosition(ruleg->x + vec3(0,-s,0));
    
    rhipLat->setPosition(pelvisX + vec3(-s/2,0,0));
    lhipLat->setPosition(pelvisX + vec3(s/2,0,0));
    
    
    
    
    
    lknee = HingeJoint::create(luleg, vec3(0,-s/2,0), vec3(1,0,0), llleg);
    lknee->setLimits(0, glm::pi<float>()*0.75);
    
    rknee = HingeJoint::create(ruleg, vec3(0,-s/2,0), vec3(1,0,0), rlleg);
    rknee->setLimits(0, glm::pi<float>()*0.75);
    
    
    
    lhipLatJoint = HingeJoint::create(lhipLat, vec3(0,0,0), vec3(0,0,1), pelvis);
    lhipLatJoint->setLimits(-glm::pi<float>()*0.2, glm::pi<float>()*0.2);
    
    rhipLatJoint = HingeJoint::create(rhipLat, vec3(), vec3(0,0,1), pelvis);
    rhipLatJoint->setLimits(-glm::pi<float>()*0.2, glm::pi<float>()*0.2);
    
    
    
    lhip = HingeJoint::create(luleg, vec3(0,s/2,0), vec3(1,0,0), lhipLat);
    lhip->setLimits(-glm::pi<float>()*0.25, glm::pi<float>()*0.5);
    
    rhip = HingeJoint::create(ruleg, vec3(0,s/2,0), vec3(-1,0,0), rhipLat);
    rhip->setLimits(-glm::pi<float>()*0.5, glm::pi<float>()*0.25);
    
    
    
    backLat = HingeJoint::create(torsoLat, vec3(0,0,0), vec3(0,0,1), pelvis);
    backLat->setLimits(-glm::pi<float>()*0.1, glm::pi<float>()*0.1);
    
    
    back = HingeJoint::create(torso, torsoLat->x - torso->x, vec3(1,0,0), torsoLat);
    back->setLimits(-glm::pi<float>()*0.35, glm::pi<float>()*0.35);
    
    
    muscles.resize(8);
    
    auto motor = MotorMuscle::create(rhip);
    muscles[RHIP] = motor;
    
    motor = MotorMuscle::create(rknee);
    muscles[RKNEE] = motor;
    
    motor = MotorMuscle::create(back);
    muscles[BACK] = motor;
    
    motor = MotorMuscle::create(lhip);
    muscles[LHIP] = motor;
    
    motor = MotorMuscle::create(lknee);
    muscles[LKNEE] = motor;
    
    motor = MotorMuscle::create(lhipLatJoint);
    muscles[LHIPLAT] = motor;
    
    motor = MotorMuscle::create(rhipLatJoint);
    muscles[RHIPLAT] = motor;
    
    motor = MotorMuscle::create(backLat);
    muscles[BACKLAT] = motor;
    
    sy = pelvis->com.z;
    ticks = 0;
    
}

vec3 Character::com() {
    vec3 COM = vec3();
    float mass = 0.f;
    for (auto &bone : getBones()) {
        COM += bone->com * bone->m;
        mass += bone->m;
    }
    return COM/mass;
}

vec3 Character::leftAnkle() {
    return luleg->com + luleg->R * (lfootElem.x + luleg->xModel);
}

vec3 Character::rightAnkle() {
    return ruleg->com + ruleg->R * (rfootElem.x + ruleg->xModel);
}

vector<Body> Character::getBones() {
    return {torso, pelvis, luleg, ruleg, llleg, rlleg, torsoLat, lhipLat, rhipLat};
}

vector<Constraint> Character::getJoints() {
    return {lhip, rhip, lknee, rknee, back, backLat, lhipLatJoint, rhipLatJoint};
}

void Character::update(float dt) {
    if (brain) {
        brain->update(dt);
    }
    
//    cout << pelvis->com.y << endl;
    for (int i = 0; i < muscles.size(); i++) {
        muscles[i]->t = Utils::clamp(brain->network[i]->output, 0.0, 1.0);
        muscles[i]->update(dt);
    }
}




