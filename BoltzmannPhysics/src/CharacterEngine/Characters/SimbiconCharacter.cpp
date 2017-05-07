//
//  SimbiconCharacter.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 5/7/17.
//
//

#include "SimbiconCharacter.hpp"

using namespace bltz;

void SimbiconCharacter::setup(float height, vec3 pelvisX) {
    Character::setup(height, pelvisX);
    
    FState leftStand = TemporalState::create(0.3f);
    leftStand->Theta = {0.f, 0.f, 0.f, 3.14/4.0, 0.f, -3.14/4.0};
    
    fsm.states.push_back(leftStand);
    
    FState rightStrike = StrikeState::create(rlleg);
    rightStrike->Theta = {-3.14/4.0, 0.f, 0.f, 0.f, 0.f, 0.f};
    
    fsm.states.push_back(rightStrike);
    
    FState rightStand = TemporalState::create(0.3f);
    rightStand->Theta = {3.14/4.0, 0.f, -3.14/4.0, 0.f, 0.f, 0.f};
    
    fsm.states.push_back(rightStand);
    
    FState leftStrike = StrikeState::create(llleg);
    leftStrike->Theta = {0.f, 0.f, 0.f, -3.14/4.0, 0.f, 0.f};
    
    fsm.states.push_back(leftStrike);
}

void SimbiconCharacter::update(float dt) {
    fsm.step(dt);
    vector<float> Theta = fsm.states[fsm.current]->Theta;
    
    for (int i = 0; i < muscles.size(); i++) {
        muscles[i]->mapThetaToT(Theta[i]);
        muscles[i]->update(dt);
    }
}
