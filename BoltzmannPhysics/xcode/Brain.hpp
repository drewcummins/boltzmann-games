//
//  Brain.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#ifndef Brain_hpp
#define Brain_hpp

#include <stdio.h>
#include <vector>
#include <iostream>
#include "EvolutionModel.hpp"

namespace bltz {
    
    using namespace std;
    
    class AbstractNeuron {
    public:
        virtual void update(float input, float dt) = 0;
        virtual vector<Gene> toGenes() = 0;
        virtual void fromGenes(vector<Gene> genes) = 0;
        float output;
    };
    
    typedef shared_ptr<AbstractNeuron> Neuron;
    
    class StandardNeuron : public AbstractNeuron {
    public:
        static Neuron create(float tau, float bias);
        float A, dA;
        float tau;
        float bias;
        virtual void update(float input, float dt);
        virtual vector<Gene> toGenes();
        virtual void fromGenes(vector<Gene> genes);
    protected:
        StandardNeuron(float tau, float bias);
    };
    
    class Brain : public Evolvable {
    public:
        Brain(int n);
        vector<Neuron> network;
        vector<vector<float> > W;
        void update(float dt);
        virtual vector<Gene> toGenome();
        virtual void fromGenome(vector<Gene> genome);
    };
}

#endif /* Brain_hpp */
