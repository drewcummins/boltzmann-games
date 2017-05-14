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
        bool leftRight;
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
        static FState create(float duration, int size=8) {
            shared_ptr<TemporalState> state(new TemporalState());
            state->duration = duration;
            state->Theta.reserve(size);
            return state;
        }
        float duration;
        virtual bool isComplete() {
            return time >= duration;
        }
    };
    
    class StrikeState : public FiniteState {
    public:
        static FState create(Body body, int size=8) {
            shared_ptr<StrikeState> state(new StrikeState());
            state->body = body;
            state->Theta.reserve(size);
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
        int current = 0;
        FState step(float dt) {
            FState state = states[current];
            state->step(dt);
            if (state->isComplete()) {
                cout << current << "-> ";
                current = (current+1) % states.size();
                cout << current << endl;
                state->transitionOut();
                state = states[current];
                state->transitionIn();
            }
            return state;
        }
    };
    
    class SimbiconCharacter : public bltz::Character {
    public:
        static shared_ptr<SimbiconCharacter> create();
        FSM fsm;
        Hinge hj;
        virtual void setup(float height, vec3 pelvisX);
        virtual void update(float dt);
        virtual vector<Constraint> getJoints();
    };
}

#endif /* SimbiconCharacter_hpp */
