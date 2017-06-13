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
    Character::setup(height, pelvisX + vec3(0,0,0));
    
    
    Body ground = RigidBody::create();
    ground->isGround = true;
    hj = HingeJoint::create(pelvis, vec3(0,0,0), normalize(vec3(0,1,0)), ground);
//    pelvis->isGround = true;
    
    
    FState dummy1 = TemporalState::create(0.3f);
    dummy1->leftRight = false;
    dummy1->Theta = {0,0,0,0,0,0,0,0};
    dummy1->Theta[RHIP] = 3.14/2.f;
//    dummy1->Theta[RKNEE] = 3.14/2.f;
//    dummy1->Theta[RKNEE] = 3.14/2.f;
    
//    FState strikeRight = StrikeState::create(rlleg);
//    strikeRight->leftRight = false;
//    strikeRight->Theta = {0,0,0,0,0,0,0,0};
//    strikeRight->Theta[RHIP] = 3.14/4.f;
//
    FState dummy2 = TemporalState::create(0.3f);
    dummy2->leftRight = true;
    dummy2->Theta = {0,0,0,0,0,0,0,0};
//    dummy2->Theta[LHIP] = -3.14/2.f;
//    dummy2->Theta[LKNEE] = 3.14/2.f;
//
//    FState strikeLeft = StrikeState::create(llleg);
//    strikeLeft->leftRight = true;
//    strikeLeft->Theta = {0,0,0,0,0,0,0,0};
//    strikeLeft->Theta[LHIP] = 3.14/4.f;
    
    fsm.states.push_back(dummy1);
//    fsm.states.push_back(strikeRight);
    fsm.states.push_back(dummy2);
//    fsm.states.push_back(strikeLeft);
}

vector<Constraint> SimbiconCharacter::getJoints() {
//    return {lhip, rhip, lknee, rknee, back, backLat, lhipLatJoint, rhipLatJoint};
    return {lhip, rhip, lknee, rknee, back, backLat, lhipLatJoint, rhipLatJoint, hj};
}

float gtijs(float thetaWorld, Hinge joint) {
    // First, we need a reference vector orthogonal to the joint axis
    vec3 reference = normalize(cross(orthogonal(joint->a1), joint->a1));
    
    // Rotate this vector about the joint axis to the global target (<thetaWorld>):
    vec3 targetWorld = glm::rotate(reference, glm::degrees(thetaWorld), joint->a1);
    
    // Transform target into joint space
    vec3 targetJoint = inverse(joint->b1->R) * targetWorld;
    
    // Now we need to find the angle between these 2 vectors on the joint axis plane (in world space)
    vector<vec2> plane = Utils::projectPointsOntoPlane({targetJoint, reference}, vec3(), joint->a1);
    vec2 u1 = normalize(plane[0]);
    vec2 u2 = normalize(plane[1]);
    
    // The cosine of the angle between these 2 vectors is given by the dot product
    // Theta is then arccos(u1.u2), so we clamp rounding errors to [-1,1]
    float cosineDTheta = Utils::clamp(dot(u1, u2), -1.f, 1.f);
    float dTheta = glm::acos(cosineDTheta);
    
    // If the points are in clockwise order, we need to change the sign of theta
    // so that we are calculating the angle in the correct direction
    if (Utils::isClockwise({u1, u2, vec2()})) {
        dTheta = -dTheta;
    }
    cout << dTheta << endl;
    return joint->cacheTheta() + dTheta;
}

float SimbiconCharacter::getTargetInJointSpace(float thetaWorld, float thetaJoint, Body body, vec3 jointAxis, bool flip) {
    
    vec3 oj = orthogonal(jointAxis);
    vec3 target = glm::rotate(oj, glm::degrees(thetaWorld), jointAxis);
//    target = normalize(cross(target, jointAxis));
//    vec3 targetInJointSpace = inverse(body->R) * target;
    vec3 targetInJointSpace = glm::rotate(oj, glm::degrees(thetaJoint), jointAxis);
    
    vector<vec2> plane = Utils::projectPointsOntoPlane({target, targetInJointSpace}, vec3(0), jointAxis);
    vec2 u1 = normalize(plane[0]);
    vec2 u2 = normalize(plane[1]);
    
    float du = Utils::clamp(dot(u1, u2), -1.f, 1.f);
    
    float dtheta = glm::acos(du);
    if (Utils::isClockwise({u1, u2, vec2()})) {
        dtheta = -dtheta;
    }
    
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
        swingHip = lhip;
        swingHipLat = lhipLatJoint;
    } else {
        swing = RHIP;
        stance = LHIP;
        swingTheta = rhip->cacheTheta();
        swingLatTheta = rhipLatJoint->cacheTheta();
        swingAnkle = rightAnkle();
        stanceAnkle = leftAnkle();
        swingHip = rhip;
        swingHipLat = rhipLatJoint;
    }
    
    vec3 COM = pelvis->com; // com();
    
    vec3 coronal = pelvis->R * vec3(1,0,0);
    vec3 sagittal = pelvis->R * vec3(0,0,1);
    
//    coronal.y = sagittal.y = 0;
    
    float dCoronal = dot(coronal, COM - stanceAnkle);
    float vCoronal = dot(coronal, pelvis->v);
    float dSagittal = dot(sagittal, COM - stanceAnkle);
    float vSagittal = dot(sagittal, pelvis->v);
    
    const float cd = 0.5f;
    const float cv = 0.2f;
    
//    Theta[swing] = gtijs(Theta[swing], swingHip);
//    Theta[swing] = getTargetInJointSpace(Theta[swing], swingHip->cacheTheta(), swingHip->b2, -swingHip->a2);
//    cout << state->Theta[swing] << " - " << swingHip->cacheTheta() << " - " << Theta[swing] << endl;
//    Theta[swing+1] = getTargetInJointSpace(Theta[swing+1], swingHipLat->cacheTheta(), swingHipLat->b1, swingHipLat->a1);
//
//    Theta[swing]    += cd * dSagittal + cv * vSagittal;
//    Theta[swing+1]  += cd * dCoronal  + cv * vCoronal;

//    Theta[BACK] = gtijs(Theta[BACK], back);
//    Theta[BACKLAT] = gtijs(Theta[BACKLAT], backLat);
    
//    Theta[BACK] = getTargetInJointSpace(Theta[BACK], back->cacheTheta(), torso, back->a1);
//    Theta[BACKLAT] = getTargetInJointSpace(Theta[BACKLAT], backLat->cacheTheta(), torsoLat, -backLat->a1);
    
//    cout << (back->cacheTheta() + dthetaTorso) << " - " << Theta[BACK] << endl;
//////
//////    // find stance hip torque:
//////    // -back - swing hip for coronal and sagittal
//    Theta[stance] += -(Theta[BACK] - state->Theta[BACK]) - (Theta[swing] - swingTheta);
//    Theta[stance+1] += -(Theta[BACKLAT] - state->Theta[BACKLAT]) - (Theta[swing+1] - swingLatTheta);
    
    for (int i = 0; i < muscles.size(); i++) {
        muscles[i]->mapThetaToT(Theta[i]);
        muscles[i]->update(dt);
    }
}







