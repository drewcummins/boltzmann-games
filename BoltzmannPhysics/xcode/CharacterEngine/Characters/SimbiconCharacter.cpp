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

void SimbiconCharacter::setup(float height, vec3 pelvisX) {
    Character::setup(height, pelvisX);
    
    FState leftStand = TemporalState::create(0.3f);
    leftStand->leftRight = false;
    leftStand->Theta = {0.f, 0.f, 0.f, 3.14/4.0, 0.f, -3.14/4.0};
    
    fsm.states.push_back(leftStand);
    
    FState rightStrike = StrikeState::create(rlleg);
    rightStrike->leftRight = false;
    rightStrike->Theta = {-3.14/4.0, 0.f, 0.f, 0.f, 0.f, 0.f};
    
    fsm.states.push_back(rightStrike);
    
    FState rightStand = TemporalState::create(0.3f);
    rightStand->leftRight = true;
    rightStand->Theta = {3.14/4.0, 0.f, -3.14/4.0, 0.f, 0.f, 0.f};
    
    fsm.states.push_back(rightStand);
    
    FState leftStrike = StrikeState::create(llleg);
    leftStrike->leftRight = true;
    leftStrike->Theta = {0.f, 0.f, 0.f, -3.14/4.0, 0.f, 0.f};
    
    fsm.states.push_back(leftStrike);
}

void SimbiconCharacter::update(float dt) {
    fsm.step(dt);
    
    FState state = fsm.states[fsm.current];
    
    vector<float> Theta(state->Theta);
    
    int swing, stance;
    vec3 swingAnkle, stanceAnkle;
    
    if (state->leftRight) {
        swing = LHIP;
        stance = RHIP;
        swingAnkle = leftAnkle();
        stanceAnkle = rightAnkle();
    } else {
        swing = RHIP;
        stance = LHIP;
        swingAnkle = rightAnkle();
        stanceAnkle = leftAnkle();
    }
    
    vec3 COM = pelvis->com;
    
    vec3 coronal = pelvis->R * vec3(1,0,0);
    vec3 sagittal = pelvis->R * vec3(0,0,1);
    
    float dCoronal = sqrt(dot(coronal, COM - stanceAnkle));
    float vCoronal = sqrt(dot(coronal, pelvis->v));
    float dSagittal = sqrt(dot(sagittal, COM - stanceAnkle));
    float vSagittal = sqrt(dot(sagittal, pelvis->v));
    
    const float cd = 1.f;
    const float cv = 1.f;
    
    Theta[swing]    += cd * dSagittal + cv * vSagittal;
    Theta[swing+1]  += cd * dCoronal  + cv * vCoronal;
    
    // set torso orientation to up in world coordinates:
    // 
    
    // find stance hip torque:
    // -back - swing hip for coronal and sagittal
    
    for (int i = 0; i < muscles.size(); i++) {
        muscles[i]->mapThetaToT(Theta[i]);
        muscles[i]->update(dt);
    }
}
