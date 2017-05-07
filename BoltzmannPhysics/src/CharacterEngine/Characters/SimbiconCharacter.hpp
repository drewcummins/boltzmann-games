//
//  SimbiconCharacter.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 5/7/17.
//
//

#ifndef SimbiconCharacter_hpp
#define SimbiconCharacter_hpp

#include <stdio.h>
#include "RigidBody.hpp"
#include "Constraints.h"
#include "Muscle.hpp"
#include "Brain.hpp"
#include "Character.hpp"

using namespace ci;
using namespace std;

namespace bltz {
    
    class FiniteState {
    public:
        
        float time;
        virtual void transitionIn() {
            time = 0.f;
        }
        virtual void transitionOut() {
            //
        }
        virtual void step(float dt) {
            time += dt;
        }
        virtual bool isComplete() = 0;
        vector<float> Theta;
    };
    
    typedef shared_ptr<FiniteState> FState;
    
    class TemporalState : public FiniteState {
    public:
        static FState create(float duration) {
            shared_ptr<TemporalState> state(new TemporalState());
            state->duration = duration;
            return state;
        }
        float duration;
        virtual bool isComplete() {
            return time >= duration;
        }
    };
    
    class StrikeState : public FiniteState {
    public:
        static FState create(Body body) {
            shared_ptr<StrikeState> state(new StrikeState());
            state->body = body;
            return state;
        }
        Body body;
        virtual bool isComplete() {
            return body->didCollideWithGround;
        }
    };
    
    class FSM {
    public:
        vector<FState> states;
        int current;
        void step(float dt) {
            FState state = states[current];
            state->step(dt);
            if (state->isComplete()) {
                current = (current+1) % states.size();
                state->transitionOut();
                state = states[current];
                state->transitionIn();
            }
        }
    };
    
    class SimbiconCharacter : public Character {
    public:
        FSM fsm;
        virtual void setup(float height, vec3 pelvisX);
        virtual void update(float dt);
    };
}

#endif /* SimbiconCharacter_hpp */
