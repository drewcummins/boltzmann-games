//
//  Island.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#include "Island.hpp"
#include "Utils.hpp"
#include "Constants.hpp"

using namespace bltz;

Isle Island::create(uint seed, uint solverIterations) {
    shared_ptr<Island> island(new Island());
    island->seed = seed;
    island->rng = Rand(seed);
    island->solverIterations = 30; //solverIterations;
    island->reset();
    island->gravity = vec3(0,-9.8,0) * METERS_TO_UNITS;
    return island;
}

void Island::addBody(Body body) {
    bodies.push_back(body);
}

void Island::addConstraint(Constraint constraint) {
    if (!constraint->b2->isGround) {
        uint id = constraint->b1->id ^ constraint->b2->id;
        jointMap[id] = true;
    }
    constraints.push_back(constraint);
}

void Island::removeConstraint(Constraint constraint) {
    vector<Constraint>::iterator it;
    for (it = constraints.begin(); it < constraints.end(); it++) {
        if (*it == constraint) {
            break;
        }
    }
    constraints.erase(it);
}

void Island::reset() {
    rng = Rand(seed);
}



vector<Constraint> Island::integrateDVAndFindCollisions(float dt) {
    for (auto &body : bodies) {
        if (!body->isGround) {
            body->addForce(gravity*body->m);
            body->integrateAcceleration(dt);
        }
    }
    
    collision.createCache(bodies);
    
    //    vector<CandidatePair> candidates = collision.findCandidates();
    vector<Contact> contacts = {}; //collision.findContacts(candidates);
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
    
    return all;
}

void Island::integratePosition(float dt, vector<Constraint> all) {
    for (int i = 0; i < solverIterations; i++) {
        //        random_shuffle(all.begin(), all.end());
        for (auto &constraint : all) {
            constraint->solve(dt);
        }
    }
    
    for (auto &body : bodies) {
        if (!body->isGround) {
            body->integrateVelocity(dt);
        }
    }
    
    collision.clearCache();
}


void Island::step(float dt) {
    
    for (auto &body : bodies) {
        if (!body->isGround) {
            body->addForce(gravity*body->m);
            body->integrateAcceleration(dt);
        }
    }
    
    collision.createCache(bodies);
    
//    vector<CandidatePair> candidates = collision.findCandidates();
    vector<Contact> contacts = {}; //collision.findContacts(candidates);
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
//        random_shuffle(all.begin(), all.end());
        for (auto &constraint : all) {
            constraint->solve(dt);
        }
    }
    
    for (auto &body : bodies) {
        if (!body->isGround) {
            body->integrateVelocity(dt);
        }
    }
    
    collision.clearCache();
    
}

Island::Island() {
    id = 0;
    while(id == 0) {
        id = Utils::rand.nextUint();
    }
}
