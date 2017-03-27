//
//  Evolution.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/19/17.
//
//

#include "Evolution.hpp"
#include "Utils.hpp"
#include "Constants.hpp"

using namespace bltz;

Evolution *Evolution::instance = NULL;

Genome::Genome(vector<Gene> genes) : genes(genes){}

void Genome::mutate(float rate) {
    for (auto &gene : genes) {
        if (Utils::rand.nextFloat() < rate) {
//            gene.bmutate(rate);
            gene.fmutate(rate);
//            gene.fvalue = gene.mean + Utils::rand.nextGaussian() * gene.sigma;
//            gene.fvalue = Utils::clamp(gene.fvalue, gene.min, gene.max);
        }
    }
}

Genome Genome::crossover(Genome mate, float rate) {
    vector<Gene> offspring(genes.size());
//    Genome genome;
//    vector<Gene> gs = genes;
//    for (int i = 0; i < genes.size(); i++) {
//        if (((double)arc4random() / (RAND_MAX)) < rate) {
//            gs = gs == genes ? mate.genes : genes;
//        }
//        offspring[i] = gs[i];
//    }
    return Genome(offspring);
}

struct fitnessSorter
{
    inline bool operator() (const Member& mem1, const Member& mem2)
    {
        return (mem1.objective->fitness() > mem2.objective->fitness());
    }
};


Evolution *Evolution::getInstance() {
     return Evolution::instance;
}


Evolution::Evolution(int populationSize) : populationSize(populationSize) {
    instance = this;
}

void Evolution::step(float dt) {
    
}

void Evolution::next() {
    
}

void Evolution::runSimulation(int numGenerations) {
    for (int i = 0; i < populationSize; i++) {
        Member member;
        Character character;
        character.setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
//        character.brain = shared_ptr<Brain>(new Brain(10));
        character.brain = shared_ptr<MatsuokaNetwork>(new MatsuokaNetwork(5));
        
        member.character = character;
        member.objective = shared_ptr<Objective>(new TorsoUpObjective(character));
        
        generation.push_back(member);
        
        Isle island = Island::create(3);
        
        island->ground = RigidBody::create();
        island->ground->isGround = true;
        
        for (auto &bone : character.getBones()) {
            island->addBody(bone);
        }
        
        for (auto &joint : character.getJoints()) {
            island->addConstraint(joint);
        }
        
        islandMap[island->id] = island;
        islandMemberMap[island->id] = member;
    }
    
    for (int i = 0; i < numGenerations; i++) {
        cout << "Running generation " << i+1 << endl;
        for (int j = 0; j < 1200; j++) {
            for (auto &island : islandMap) {
                Member member = islandMemberMap[island.second->id];
                if (!member.objective->isDead) {
                    member.character.update(1/60.f);
                    island.second->step(1/60.f);
                    member.objective->update();
                }
            }
        }
        
        select();
        islandMap.clear();
        islandMemberMap.clear();
        generation.clear();
        
        for (int j = 0; j < populationSize; j++) {
            
            Member member;
            Character character;
            character.setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
//            character.brain = shared_ptr<Brain>(new Brain(10));
            character.brain = shared_ptr<MatsuokaNetwork>(new MatsuokaNetwork(5));
            character.brain->fromGenome(nextGen[j].genes);
            
            member.character = character;
            member.objective = shared_ptr<Objective>(new TorsoUpObjective(character));
            
            generation.push_back(member);
            
            Isle island = Island::create(3);
            
            island->ground = RigidBody::create();
            island->ground->isGround = true;
            
            for (auto &bone : character.getBones()) {
                island->addBody(bone);
            }
            
            for (auto &joint : character.getJoints()) {
                island->addConstraint(joint);
            }
            
            islandMap[island->id] = island;
            islandMemberMap[island->id] = member;
        }
    }
}

void Evolution::select() {
    sort(generation.begin(), generation.end(), fitnessSorter());
    
    for (int i = 0; i < min(3, (int) generation.size()); i++) {
        cout << generation[i].objective->fitness() << endl;
    }
    cout << endl;
    
    nextGen.clear();
    vector<Gene> genes = generation[0].character.brain->toGenome();
    Genome genome(genes);
    nextGen.push_back(genome);
    while (nextGen.size() < populationSize) {
        for (int i = 0; i < 1+populationSize/2; i++) {
            vector<Gene> genes = generation[i].character.brain->toGenome();
            Genome genome(genes);
            genome.mutate(0.15);
            nextGen.push_back(genome);
        }
    }
}


Genome Evolution::winner() {
    return nextGen[0];
}



TorsoUpObjective::TorsoUpObjective(Character character) : character(character) {
    Q = character.torso->q;
    y = character.pelvis->com.y;
    z = character.pelvis->com.z;
    dist = 0;
    qdot = 0;
    dy = 0;
    no = false;
    time = 0;
}

void TorsoUpObjective::update() {
    vec3 q = character.torso->R * vec3(0,1,0);
    float qdott = dot(vec3(0,1,0), q);
    
    time += 1/60.f;
    if (character.pelvis->com.y/y < 0.65) {
        if (!no) {
            no = true;
            isDead = true;
            float len = length(vec2(character.luleg->com.x, character.luleg->com.z));
            dist = U2M(len);
        }
        
    }
    
    if (!no) {
        qdot += qdott/240.f;
    }
    
    dy += character.pelvis->com.y;
    
//    dy += fabs(character.pelvis->com.y - y);
    
}

float TorsoUpObjective::fitness() {
    
//    return qdot*dy/(y*time);
    
    if (!no) {
        float len = length(vec2(character.luleg->com.x, character.luleg->com.z));
        dist = U2M(len);
    }
    return dist;
//    return qdot;
//    return character.pelvis->com.y;
    return (character.luleg->com.z - z) * qdot; // * dy * time/60.f;
    return dy;
}
