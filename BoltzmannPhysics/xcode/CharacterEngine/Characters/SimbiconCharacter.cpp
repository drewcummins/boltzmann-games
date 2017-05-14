//
//  SimbiconCharacter.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 5/7/17.
//
//

#include "SimbiconCharacter.hpp"
#include "Constants.hpp"

using namespace bltz;

shared_ptr<SimbiconCharacter> SimbiconCharacter::create() {
    return shared_ptr<SimbiconCharacter>(new SimbiconCharacter);
}

void SimbiconCharacter::setup(float height, vec3 pelvisX) {
    Character::setup(height, pelvisX + vec3(0,-0.3,0));
    
    
    Body ground = RigidBody::create();
    ground->isGround = true;
    hj = HingeJoint::create(pelvis, vec3(0,0,0), vec3(1,0,0), ground);
    
    
    FState dummy1 = TemporalState::create(0.3f);
    dummy1->leftRight = false;
    dummy1->Theta = {0,0,0,0,0,0,0,0};
    dummy1->Theta[RHIP] = -3.14/2.f;
    dummy1->Theta[RKNEE] = 3.14/2.f;
    
    FState dummy2 = TemporalState::create(0.3f);
    dummy2->leftRight = true;
    dummy2->Theta = {0,0,0,0,0,0,0,0};
    dummy2->Theta[LHIP] = -3.14/2.f;
    dummy2->Theta[LKNEE] = 3.14/2.f;
    
    fsm.states.push_back(dummy1);
    fsm.states.push_back(dummy2);
    return;
    
    FState leftStand = TemporalState::create(0.3f);
    leftStand->leftRight = false;
    leftStand->Theta = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
//    leftStand->Theta[RHIP] = -3.14/2.f;
//    leftStand->Theta[RKNEE] = 3.14/4.f;
    
    fsm.states.push_back(leftStand);
    
    FState rightStrike = StrikeState::create(rlleg);
    rightStrike->leftRight = false;
    rightStrike->Theta = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
//    rightStrike->Theta[RHIP] = 3.14/4.f;
    
    fsm.states.push_back(rightStrike);
    
    FState rightStand = TemporalState::create(0.3f);
    rightStand->leftRight = true;
    rightStand->Theta = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
//    rightStand->Theta[LHIP] = -3.14/2.f;
//    rightStand->Theta[LKNEE] = 3.14/4.f;
    
    fsm.states.push_back(rightStand);
    
    FState leftStrike = StrikeState::create(llleg);
    leftStrike->leftRight = true;
    leftStrike->Theta = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
//    leftStrike->Theta[LHIP] = 3.14/4.f;
    
    fsm.states.push_back(leftStrike);
    
}

vector<Constraint> SimbiconCharacter::getJoints() {
    return {lhip, rhip, lknee, rknee, back, backLat, lhipLatJoint, rhipLatJoint, hj};
}

void SimbiconCharacter::update(float dt) {
    fsm.step(dt);
    
    FState state = fsm.states[fsm.current];
    
    vector<float> Theta(state->Theta);
    
    int swing, stance;
    float swingTheta, swingLatTheta;
    vec3 swingAnkle, stanceAnkle;
    
    if (state->leftRight) {
        swing = LHIP;
        stance = RHIP;
        swingTheta = lhip->cacheTheta();
        swingLatTheta = lhipLatJoint->cacheTheta();
        swingAnkle = leftAnkle();
        stanceAnkle = rightAnkle();
    } else {
        swing = RHIP;
        stance = LHIP;
        swingTheta = rhip->cacheTheta();
        swingLatTheta = rhipLatJoint->cacheTheta();
        swingAnkle = rightAnkle();
        stanceAnkle = leftAnkle();
    }
    
    vec3 COM = com();
    
    vec3 coronal = pelvis->R * vec3(1,0,0);
    vec3 sagittal = pelvis->R * vec3(0,0,1);
    
    coronal.y = sagittal.y = 0;
    
    float dCoronal = dot(coronal, COM - stanceAnkle);
    float vCoronal = dot(coronal, pelvis->v);
    float dSagittal = dot(sagittal, COM - stanceAnkle);
    float vSagittal = dot(sagittal, pelvis->v);
    
    const float cd = 0.5f;
    const float cv = 0.2f;
    
//    Theta[swing]    += cd * dSagittal + cv * vSagittal;
//    Theta[swing+1]  += cd * dCoronal  + cv * vCoronal;
    
    vec3 up = vec3(0,1,0);
    vec3 torsoUpInWorldSpace = inverse(torso->R) * up;
//    vec3 r1ws = torso->com + torso->R * back->r1;
//    vec3 a1ws = torso->R * back->a1;
//    vector<vec2> torsoPlane = Utils::projectPointsOntoPlane({torsoUpInWorldSpace, up}, r1ws, a1ws);
    
    vec3 upInTorsoSpace = inverse(torso->R) * up;
    
    // up in the points to be transformed is actually just the torso up vector
    vector<vec2> torsoPlane = Utils::projectPointsOntoPlane({upInTorsoSpace, up}, back->r1, back->a1);
    vec2 u1 = normalize(torsoPlane[0]);
    vec2 u2 = normalize(torsoPlane[1]);
    
    float dthetaTorso = glm::acos(dot(u1, u2));
    if (Utils::isClockwise({u1, u2, vec2()})) {
        dthetaTorso = -dthetaTorso;
    }
    
    vec3 upInTorsoLatSpace = torsoLat->R * up;
    
    torsoPlane = Utils::projectPointsOntoPlane({upInTorsoLatSpace, up}, backLat->r1, backLat->a1);
    u1 = normalize(torsoPlane[0]);
    u2 = normalize(torsoPlane[1]);
    float dthetaTorsoLat = acos(dot(u1, u2));
    
//    cout << back->cacheTheta() << endl;
    cout << dthetaTorso << endl;
    
//    // set torso orientation to up in world coordinates:
    
    Theta[BACK] = back->cacheTheta() + dthetaTorso;
//    Theta[BACKLAT] = backLat->cacheTheta() - dthetaTorsoLat;
//////
//////    // find stance hip torque:
//////    // -back - swing hip for coronal and sagittal
//    Theta[stance] = -dthetaTorso - (Theta[swing] - swingTheta);
//    Theta[stance+1] = -dthetaTorsoLat - (Theta[swing+1] - swingLatTheta);
    
    for (int i = 0; i < muscles.size(); i++) {
        if (i == BACK || i == BACKLAT) {
//            continue;
        }
        muscles[i]->mapThetaToT(Theta[i]);
        muscles[i]->update(dt);
    }
}







