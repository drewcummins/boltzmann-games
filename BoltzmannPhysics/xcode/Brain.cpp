//
//  Brain.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#include "Brain.hpp"

using namespace bltz;

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
    output = 1.f/(1+exp(A - bias));
}

vector<Gene> StandardNeuron::toGenes() {
    vector<Gene> genes(2);
    Gene tauGene = {tau, 1.f};
    Gene biasGene = {bias, 1.2f};
    genes[0] = tauGene;
    genes[1] = biasGene;
    return genes;
}

void StandardNeuron::fromGenes(vector<Gene> genes) {
    tau = genes[0].value;
    bias = genes[1].value;
}


Brain::Brain(int n) {
    for (int i = 0; i < n; i++) {
        auto neuron = StandardNeuron::create(0.5+((double)arc4random() / (RAND_MAX)) * 4.5, -4 + ((double)arc4random() / (RAND_MAX)) * 8);
        network.push_back(neuron);
        vector<float> Wi(n);
        W.push_back(Wi);
        for (int j = 0; j < n; j++) {
            W[i][j] = -15 + ((double)arc4random() / (RAND_MAX)) * 30;
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
            input += W[i][j] * output[j];
        }
        neuron->update(input, dt);
    }
}

vector<Gene> Brain::toGenome() {
    int neuronSize = network[0]->toGenes().size();
    int n = network.size();
    vector<Gene> genome(n*(neuronSize+n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Gene weight = { W[i][j], 2.f };
            genome[i*n+j] = weight;
        }
        vector<Gene> ng = network[i]->toGenes();
        for (int j = 0; j < neuronSize; j++) {
            genome[n*n+n*j+i] = ng[j];
        }
    }
    return genome;
}

void Brain::fromGenome(vector<Gene> genome) {
    int neuronSize = network[0]->toGenes().size();
    int n = (sqrt(4*genome.size() + neuronSize*neuronSize) - neuronSize)/2;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            W[i][j] = genome[i*n+j].value;
        }
        vector<Gene> gene;
        for (int j = 0; j < neuronSize; j++) {
            gene.push_back(genome[j]);
        }
        network[i]->fromGenes(gene);
    }
}
