//
//  EvolutionModel.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#ifndef EvolutionModel_hpp
#define EvolutionModel_hpp

#include <stdio.h>

using namespace std;

namespace bltz {
    typedef struct Gene {
        float value;
        float sigma;
    } Gene;
    
    class Evolvable {
    public:
        virtual vector<Gene> toGenome() = 0;
        virtual void fromGenome(vector<Gene> genome) = 0;
    };
    
    class Objective {
    public:
        virtual void update() = 0;
        virtual float fitness() = 0;
    };
}

#endif /* EvolutionModel_hpp */
