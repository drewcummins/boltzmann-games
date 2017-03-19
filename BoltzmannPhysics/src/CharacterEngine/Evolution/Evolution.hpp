//
//  Evolution.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#ifndef Evolution_hpp
#define Evolution_hpp

#include <stdio.h>
#include "EvolutionModel.hpp"
#include "Character.hpp"

namespace bltz {
    using namespace ci;
    using namespace std;
    
    typedef struct Member {
        Character  character;
        shared_ptr<Objective> objective;
    } Member;
    
    class Genome {
    public:
        Genome(vector<Gene> genes);
        void mutate(float rate);
        Genome *crossover(Genome *mate, float rate);
        vector<Gene> genes;
    };
    
    class Evolution {
    public:
        Evolution(int populationSize);
        void runSimulation(int numGenerations);
        void select();
        int populationSize;
        vector<Member> generation;
        void step(float dt);
        void next();
    };
    
    class TorsoUpObjective : public Objective {
    public:
        TorsoUpObjective(Character character);
        Character character;
        quat Q;
        virtual void update();
        virtual float fitness();
        float qdot;
    };
}

#endif /* Evolution_hpp */
