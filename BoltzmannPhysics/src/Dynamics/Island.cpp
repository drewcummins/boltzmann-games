//
//  Island.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#include "Island.hpp"
#include "Utils.hpp"

using namespace bltz;

Isle Island::create(uint seed, uint solverIterations) {
    shared_ptr<Island> island(new Island());
    island->seed = seed;
    island->solverIterations = solverIterations;
    island->reset();
    return island;
}

void Island::addBody(Body body) {
    bodies.push_back(body);
}

void Island::addConstraint(Constraint constraint) {
    uint id = constraint->b1->id ^ constraint->b2->id;
    jointMap[id] = true;
    constraints.push_back(constraint);
}

void Island::removeConstraint(Constraint constraint) {
    
}

void Island::reset() {
    rng = Rand(seed);
}

void Island::step(float dt) {
    
    for (auto &body : bodies) {
        body->addForce(gravity*body->m);
        body->integrateAcceleration(dt);
    }
    
    collision.createCache(bodies);
    
    vector<CandidatePair> candidates = collision.findCandidates();
    vector<Contact> contacts = collision.findContacts(candidates);
    vector<Contact> floorContacts = collision.findFloorContacts();
    
    for (auto &contact : floorContacts) {
        // make all collisions with the floor immovable
        contact.pair.b2 = ground;
    }
    
    // add floor contacts to contacts
    contacts.insert(contacts.end(), floorContacts.begin(), floorContacts.end());
    
    vector<Constraint> all(constraints.begin(), constraints.end());
    
    for (auto &contact : contacts) {
        uint id = contact.pair.b1->id ^ contact.pair.b2->id;
        if (jointMap[id]) {
            continue;
        }
        Constraint cc = ContactConstraint::create(contact);
        all.push_back(cc);
    }
    
    for (auto &constraint : all) {
        constraint->prepare(dt);
    }
    
    for (int i = 0; i < solverIterations; i++) {
        for (auto &constraint : all) {
            constraint->solve(dt);
        }
    }
    
    for (auto &body : bodies) {
        body->integrateVelocity(dt);
    }
    
    collision.clearCache();
    
}

Island::Island() {
    id = 0;
    while(id == 0) {
        id = Utils::rand.nextUint();
    }
}
