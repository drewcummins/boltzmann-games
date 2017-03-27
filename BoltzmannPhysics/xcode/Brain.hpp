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
    
    
    
    typedef struct MatsuokaNeuron {
        float u, v, y;
        uint64_t mask = 0; //std::numeric_limits<uint64_t>::max();
        vector<float> W;
        vector<Gene> toGenes();
        void fromGenes(vector<Gene> genes);
        bool hasSynapse(int idx);
        void connect(int idx);
        void disconnect(int idx);
    } MatsuokaNeuron;
    
    class MatsuokaCPG : public AbstractNeuron {
    public:
        static shared_ptr<MatsuokaCPG> create();
        int index;
        MatsuokaNeuron m1, m2;
        float tauu=0.025, tauv=0.3, beta=2.5, u0=1;
        virtual void update(float input, float dt);
        virtual void update(MatsuokaNeuron &m, float input, float dt);
        virtual vector<Gene> toGenes();
        virtual void fromGenes(vector<Gene> genes);
    protected:
        MatsuokaCPG();
    };
    
    
    
    
    
    class Brain : public Evolvable {
    public:
        Brain();
        Brain(int n);
        vector<Neuron> network;
        vector<vector<float> > W;
        virtual void update(float dt);
        virtual vector<Gene> toGenome();
        virtual void fromGenome(vector<Gene> genome);
    };
    
    class MatsuokaNetwork : public Brain {
    public:
        MatsuokaNetwork(int n);
//        vector<shared_ptr<MatsuokaCPG>> network;
        virtual void update(float dt);
        virtual vector<Gene> toGenome();
        virtual void fromGenome(vector<Gene> genome);
    };
}

#endif /* Brain_hpp */
