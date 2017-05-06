//
//  Brain.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#include "Brain.hpp"
#include "Utils.hpp"

using namespace bltz;
using namespace std;

Neuron StandardNeuron::create(float tau, float bias) {
    return Neuron(new StandardNeuron(tau, bias));
}

StandardNeuron::StandardNeuron(float tau, float bias) : tau(tau), bias(bias) {
    A = 0; //((double)arc4random() / (RAND_MAX)) * 10 - 5;
    dA = 0; //((double)arc4random() / (RAND_MAX)) * 5 - 3;
}

void StandardNeuron::update(float input, float dt) {
    dA = (input-A)/tau;
    A += dA * dt;
    output = 1.f/(1+exp(bias - A));
}

vector<Gene> StandardNeuron::toGenes() {
    vector<Gene> genes(2);
    Gene tauGene = {tau, 1.5f, 2.75f, 0.5, 5.0};
    Gene biasGene = {bias, 3.0f, 0.f, -4.0, 4.0};
    genes[0] = tauGene;
    genes[1] = biasGene;
    return genes;
}

void StandardNeuron::fromGenes(vector<Gene> genes) {
    tau = genes[0].floatValue();
    bias = genes[1].floatValue();
}



Brain::Brain(int n) {
    for (int i = 0; i < n; i++) {
        float tau = Utils::clamp(2.75+Utils::rand.nextGaussian()*1.5, 0.5, 5.0);
        auto neuron = StandardNeuron::create(tau, Utils::rand.nextGaussian()*3.0);
        network.push_back(neuron);
        vector<float> Wi(n);    
        W.push_back(Wi);
        for (int j = 0; j < n; j++) {
            W[i][j] = Utils::rand.nextGaussian()*15.0;
            W[i][j] = Utils::clamp(W[i][j], -16.f, 16.f);
        }
    }
}

void Brain::update(float dt) {
    vector<float> output;
    for (auto neuron : network) {
        output.push_back(neuron->output);
    }
    for (int i = 0; i < network.size(); i++) {
        auto neuron = network[i];
        float input = 0;
        for (int j = 0; j < network.size(); j++) {
            if (i == j) {
                continue;
            }
            input += W[i][j] * output[j];
        }
        neuron->update(input, dt);
    }
}

vector<Gene> Brain::toGenome() {
    int neuronSize = network[0]->toGenes().size();
    int n = network.size();
    
    vector<Gene> genome;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Gene weight = { W[i][j], 2.f, 0.f, -16, 16 };
            genome.push_back(weight);
        }
    }
    
    for (int i = 0; i < n; i++) {
        vector<Gene> ng = network[i]->toGenes();
        for (int j = 0; j < neuronSize; j++) {
            genome.push_back(ng[j]);
        }
    }
    
    return genome;
}

void Brain::fromGenome(vector<Gene> genome) {
    int neuronSize = network[0]->toGenes().size();
    int n = network.size();
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            W[i][j] = genome[i*n+j].floatValue();
        }
        vector<Gene> gene;
        for (int j = 0; j < neuronSize; j++) {
            gene.push_back(genome[n*n+i*neuronSize+j]);
        }
        network[i]->fromGenes(gene);
    }
}

Brain::Brain() {}

shared_ptr<MatsuokaCPG> MatsuokaCPG::create() {
    return shared_ptr<MatsuokaCPG>(new MatsuokaCPG());
}

MatsuokaCPG::MatsuokaCPG(){}

void MatsuokaCPG::update(float input, float dt) {}


void MatsuokaNeuron::fromGenes(vector<Gene> genes) {
    W.clear();
    for (int i = 0; i < genes.size()-1; i++) {
        W.push_back(genes[i].floatValue());
    }
    mask = genes[genes.size()-1].binaryValue();
    u = v = y = 0;
}

vector<Gene> MatsuokaNeuron::toGenes() {
    vector<Gene> genes;
    for (int i = 0; i < W.size(); i++) {
        Gene gene = {W[i], 0.75, 0, -3, 3};
        genes.push_back(gene);
    }
    genes.push_back({mask, (uint) W.size()});
    return genes;
}

bool MatsuokaNeuron::hasSynapse(int idx) {
    return (mask & ((uint64_t)1 << static_cast<uint64_t>(idx))) > 0;
}

void MatsuokaNeuron::connect(int idx) {
    mask |= ((uint64_t)1 << static_cast<uint64_t>(idx));
}

void MatsuokaNeuron::disconnect(int idx) {
    mask &= ~((uint64_t)1 << static_cast<uint64_t>(idx));
}


vector<Gene> MatsuokaCPG::toGenes() {
    Gene u0Gene = {u0, 0.2, 1, 0.1, 2.5};
    Gene betaGene = {beta, 0.5, 2.5, 0.1, 5};
    Gene tauuGene = {tauu, 0.1, 0.2, 0.025, 1.7};
    Gene tauvGene = {tauv, 0.1, 0.5, 0.3, 1.7};
    vector<Gene> genes = {u0Gene, betaGene, tauuGene, tauvGene};
    vector<Gene> m1Genes = m1.toGenes();
    vector<Gene> m2Genes = m2.toGenes();
    genes.reserve(m1Genes.size() + m2Genes.size() + 4);
    genes.insert(genes.end(), m1Genes.begin(), m1Genes.end());
    genes.insert(genes.end(), m2Genes.begin(), m2Genes.end());
    return genes;
}

void MatsuokaCPG::fromGenes(vector<Gene> genes) {
    u0 = genes[0].floatValue();
    beta = genes[1].floatValue();
    tauu = genes[2].floatValue();
    tauv = genes[3].floatValue();
    
    int weightSize = (genes.size() - 4)/2;
    m1.fromGenes(vector<Gene>(genes.begin()+4, genes.begin()+4+weightSize));
    m2.fromGenes(vector<Gene>(genes.begin()+4+weightSize, genes.end()));
    
//    m1.u = 1;
}


void MatsuokaCPG::update(MatsuokaNeuron &m, float input, float dt) {
    
    float du = (-m.u - beta * m.v + input + u0) / tauu;
    float dv = (-m.v + m.y) / tauv;
    
    m.u += du * dt;
    m.v += dv * dt;
    
    m.y = max(0.f, m.u);
}


MatsuokaNetwork::MatsuokaNetwork(int n) : Brain() {
    for (int i = 0; i < n; i++) {
        auto neuron = MatsuokaCPG::create();
        for (int j = 0; j < n*2; j++) {
            neuron->m1.W.push_back(-2);
            neuron->m2.W.push_back(-2);
        }
        // make internal neurons always connected to start
        neuron->m2.connect(i*2);
        neuron->m1.connect(i*2+1);
//        if (i > 0) {
//            neuron->m1.connect(i*2-1);
//            neuron->m1.connect(i*2-2);
//            neuron->m2.connect(i*2-1);
//            neuron->m2.connect(i*2-2);
//        }
        network.push_back(neuron);
    }
    
//    for (int i = 0; i < n; i++) {
//        shared_ptr<MatsuokaCPG> neuron = static_pointer_cast<MatsuokaCPG>(network[i]);
//        for (int j = 0; j < n; j++) {
//            if (i != j) {
//                neuron->m1.connect(i*2);
//                neuron->m1.connect(i*2+1);
//                neuron->m2.connect(i*2);
//                neuron->m2.connect(i*2+1);
//            }
//        }
//    }
    
    vector<Gene> genes = toGenome();
    
    for (auto &gene : genes) {
        gene.init();
    }
    
    fromGenome(genes);
}

void MatsuokaNetwork::update(float dt) {
    vector<float> y;
    for (auto &neuron : network) {
        shared_ptr<MatsuokaCPG> matsuoka = static_pointer_cast<MatsuokaCPG>(neuron);
        y.push_back(matsuoka->m1.y);
        y.push_back(matsuoka->m2.y);
    }
    
    for (int i = 0; i < network.size(); i++) {
        auto cpg = static_pointer_cast<MatsuokaCPG>(network[i]);
        float in1 = 0.f, in2 = 0.f;
        
        for (int j = 0; j < y.size(); j++) {
            if (cpg->m1.hasSynapse(j)) {
                if (false) { //j == i*2+1) {
                    in1 += -2 * y[j];
                } else {
                    in1 += cpg->m1.W[j] * y[j];
                }
                
            }
            
            if (cpg->m2.hasSynapse(j)) {
                if (false) { //j == i*2) {
                    in1 += -2 * y[j];
                } else {
                    in2 += cpg->m2.W[j] * y[j];
                }
            }
            
        }
        
        cpg->update(cpg->m1, in1, dt);
        cpg->update(cpg->m2, in2, dt);
        
        cpg->output = cpg->m1.y - cpg->m2.y;
    }
}

vector<Gene> MatsuokaNetwork::toGenome() {
    vector<Gene> genome;
    for (auto &cpg : network) {
        vector<Gene> genes = cpg->toGenes();
        for (auto &gene : genes) {
            genome.push_back(gene);
        }
    }
    return genome;
}

void MatsuokaNetwork::fromGenome(vector<Gene> genes) {
    network.clear();
    // N = number of genes
    // n = number of network nodes
    // sizeof(node) in genes is: 2 x 2n (weights) + 2 (synapse maps) + 4 (CPG parameters)
    // this gives us N = 4n^2 + 6n
    //
    // solution is n = 1/4 (√[4N + 9] - 3)
    
    int n = (sqrt(4*genes.size() + 9) - 3)/4;
    int neuronSize = genes.size()/n;
    for (int i = 0; i < n; i++) {
        auto neuron = MatsuokaCPG::create();
        neuron->fromGenes(vector<Gene>(genes.begin()+i*neuronSize, genes.begin()+i*neuronSize+neuronSize));
        network.push_back(neuron);
    }
}







