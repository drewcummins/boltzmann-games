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
#include "Utils.hpp"

using namespace std;

namespace bltz {
    
    
    
    // the most horrific thing i've ever done right here
    class Gene {
    public:
        
        Gene() {}
        
        Gene(float fvalue, float sigma, float mean, float min, float max) : fvalue(fvalue), sigma(sigma), mean(mean), min(min), max(max) {}
        
        Gene(uint64_t bvalue, uint numBits) : bvalue(bvalue), numBits(numBits) {}
        
        float fvalue;
        float sigma;
        float mean;
        float min;
        float max;
        
        uint64_t bvalue;
        uint numBits;
        
        virtual float floatValue() {
            return fvalue;
        }
        virtual uint64_t binaryValue()  {
            return bvalue;
        }
        virtual void finit() {
            fvalue = Utils::clamp(mean + Utils::rand.nextGaussian() * sigma, min, max);
        }
        
        virtual void fmutate(float rate) {
            if (Utils::rand.nextFloat() < rate) {
                fvalue = Utils::clamp(fvalue + Utils::rand.nextGaussian() * sigma, min, max);
            }
        }
        
        virtual void binit() {
            bvalue = 0;
        }
        
        virtual void init() {
            binit();
            finit();
        }
        
        virtual void bmutate(float rate) {
            for (int i = 0; i < numBits; i++) {
                if (Utils::rand.nextFloat() < rate) {
                    bvalue ^= (uint64_t) 1 << static_cast<uint64_t>(i);
                }
            }
        }
    };
    
    
    class Evolvable {
    public:
        virtual vector<Gene> toGenome() = 0;
        virtual void fromGenome(vector<Gene> genome) = 0;
    };
    
    class Objective {
    public:
        virtual void update() = 0;
        virtual float fitness() = 0;
        bool isDead = false;
    };
}

#endif /* EvolutionModel_hpp */
