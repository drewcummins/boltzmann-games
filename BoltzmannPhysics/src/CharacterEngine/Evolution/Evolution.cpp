//
//  Evolution.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#include "Evolution.hpp"

using namespace bltz;

Genome::Genome(vector<Gene> genes) : genes(genes){}

void Genome::mutate(float rate) {
    for (auto &gene : genes) {
        if (((double)arc4random() / (RAND_MAX)) < rate) {
            gene.value += 2 * gene.sigma * ((double)arc4random() / (RAND_MAX)) - gene.sigma;
        }
    }
}

Genome *Genome::crossover(Genome *mate, float rate) {
    vector<Gene> offspring(genes.size());
    Genome *genome = this;
    for (int i = 0; i < genes.size(); i++) {
        if (((double)arc4random() / (RAND_MAX)) < rate) {
            genome = genome == mate ? this : mate;
        }
        offspring[i] = genome->genes[i];
    }
    return new Genome(offspring);
}

struct fitnessSorter
{
    inline bool operator() (const Character& char1, const Character& char2)
    {
        return (char1.objective->fitness() > char2.objective->fitness());
    }
};



Evolution::Evolution(int populationSize) : populationSize(populationSize) {
    
}

void Evolution::step(float dt) {
    
}

void Evolution::next() {
    
}

void Evolution::runSimulation(int numGenerations) {
    
}

void Evolution::select() {
    sort(generation.begin(), generation.end(), fitnessSorter());
    for (auto &character : generation) {
//        cout << character.objective->fitness() << endl;
    }
}



TorsoUpObjective::TorsoUpObjective(SimpleBody *body) : body(body) {
    Q = body->torso->getGlobalPose().q;
    qdot = 0;
}

void TorsoUpObjective::update() {
    qdot += Q.dot(body->torso->getGlobalPose().q);
}

float TorsoUpObjective::fitness() {
    return qdot;
}
