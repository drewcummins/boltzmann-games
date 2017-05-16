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
    Character::setup(height, pelvisX + vec3(0,0.3,0));
    
    
    Body ground = RigidBody::create();
    ground->isGround = true;
    hj = HingeJoint::create(pelvis, vec3(0,0,0), vec3(0,1,0), ground);
    
    
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
}

vector<Constraint> SimbiconCharacter::getJoints() {
    return {lhip, rhip, lknee, rknee, back, backLat, lhipLatJoint, rhipLatJoint, hj};
}

//float SimbiconCharacter::getTargetInJointSpace(float thetaWorld, float thetaJoint, Body body, vec3 jointAxis, bool flip) {
////    quat q(1,0,0,0);
//    quat q = glm::rotate(quat(), thetaWorld, jointAxis);
//    vec3 x = orthogonal(jointAxis);
//    vec3 xInWorldSpace = q * x;
//    vec3 xInJointSpace = inverse(body->R) * xInWorldSpace;
////    if (dot(xInJointSpace, jointAxis) > 0) {
////        flip = true;
////    } else {
////        flip = false;
////    }
//    vector<vec2> projection = Utils::projectPointsOntoPlane({xInJointSpace, x}, vec3(), jointAxis);
//    vec2 u1 = normalize(projection[0]);
//    vec2 u2 = normalize(projection[1]);
//    float dtheta = glm::acos(dot(u1, u2));
////    if (fabs(dtheta) > glm::pi<float>()*0.5) {
////        dtheta -= glm::pi<float>();
////    }
//    if (Utils::isClockwise({u1, u2, vec2()})) {
//        dtheta = -dtheta;
//    }
//    
//    cout << dtheta << endl;
////    if (flip) {
////        dtheta = -dtheta;
////    }
//    return thetaJoint + dtheta;
//}

float SimbiconCharacter::getTargetInJointSpace(float thetaWorld, float thetaJoint, Body body, vec3 jointAxis, bool flip) {
    vec3 x = Utils::orthogonal(jointAxis);
    vec3 targetInWorldSpace = glm::rotate(x, thetaWorld, jointAxis);
    vec3 currentInJointSpace = glm::rotate(x, thetaJoint, jointAxis);
    vec3 currentInWorldSpace = body->R * currentInJointSpace;
    cout << targetInWorldSpace << " - " << currentInWorldSpace << endl;
//    vec3 xInJointSpace = body->R * glm::rotate(x, thetaWorld, jointAxis);
//    vec3 xInWorldSpace = body->R * x;
    vector<vec2> projection = Utils::projectPointsOntoPlane({targetInWorldSpace, currentInWorldSpace}, vec3(12,2,10), jointAxis);
    vec2 u1 = normalize(projection[0]);
    vec2 u2 = normalize(projection[1]);
    
    cout << u1 << " - " << u2 << " = " << dot(u1, u2) << endl;
    float dtheta = glm::acos(Utils::clamp(dot(u1, u2), 0.f, 1.f));
    if (Utils::isClockwise({u1, u2, vec2()})) {
//        cout << "OKAY" << endl;
        dtheta = -dtheta;
    }
    cout << "DTHETA: " << dtheta << endl;
//    cout << dtheta << endl;
    return thetaJoint + dtheta;
}








void SimbiconCharacter::update(float dt) {
    FState state = fsm.step(dt);
    
    vector<float> Theta(state->Theta);
    
    int swing, stance;
    float swingTheta, swingLatTheta;
    vec3 swingAnkle, stanceAnkle;
    Hinge swingHip, swingHipLat;
    
    if (state->leftRight) {
        swing = LHIP;
        stance = RHIP;
        swingTheta = lhip->cacheTheta();
        swingLatTheta = lhipLatJoint->cacheTheta();
        swingAnkle = leftAnkle();
        stanceAnkle = rightAnkle();
        swingHip = rhip;
        swingHipLat = rhipLatJoint;
    } else {
        swing = RHIP;
        stance = LHIP;
        swingTheta = rhip->cacheTheta();
        swingLatTheta = rhipLatJoint->cacheTheta();
        swingAnkle = rightAnkle();
        stanceAnkle = leftAnkle();
        swingHip = lhip;
        swingHipLat = lhipLatJoint;
    }
    
    vec3 COM = com();
    
    vec3 coronal = pelvis->R * vec3(1,0,0);
    vec3 sagittal = pelvis->R * vec3(0,0,1);
    
//    coronal.y = sagittal.y = 0;
    
    float dCoronal = dot(coronal, COM - stanceAnkle);
    float vCoronal = dot(coronal, pelvis->v);
    float dSagittal = dot(sagittal, COM - stanceAnkle);
    float vSagittal = dot(sagittal, pelvis->v);
    
    const float cd = 0.5f;
    const float cv = 0.2f;
    
    
    Theta[swing]    += cd * dSagittal + cv * vSagittal;
    Theta[swing+1]  += cd * dCoronal  + cv * vCoronal;
    
    vec3 up = vec3(0,1,0);
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
    if (Utils::isClockwise({u1, u2, vec2()})) {
        dthetaTorsoLat = -dthetaTorsoLat;
    }
    
//    // set torso orientation to up in world coordinates:
//    Theta[BACK] = back->cacheTheta() + dthetaTorso;
//    Theta[BACKLAT] = backLat->cacheTheta() + dthetaTorsoLat;
    
    cout << "BEFORE: " << Theta[BACK] << ", " << back->cacheTheta() << ", " << endl;;
    Theta[BACK] = getTargetInJointSpace(Theta[BACK], back->cacheTheta(), torso, back->a1);
    cout << "AFTER: " << Theta[BACK] << endl << endl;
//    Theta[BACKLAT] = getTargetInJointSpace(Theta[BACKLAT], backLat->cacheTheta(), torsoLat, backLat->a1);
    
//    cout << (back->cacheTheta() + dthetaTorso) << " - " << Theta[BACK] << endl;
//////
//////    // find stance hip torque:
//////    // -back - swing hip for coronal and sagittal
    Theta[stance] += -dthetaTorso - (Theta[swing] - swingTheta);
    Theta[stance+1] += -dthetaTorsoLat - (Theta[swing+1] - swingLatTheta);
    
    for (int i = 0; i < muscles.size(); i++) {
        muscles[i]->mapThetaToT(Theta[i]);
        muscles[i]->update(dt);
    }
}







