//
//  Collision.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "Collision.hpp"

using namespace bltz;

vector<CollisionPair> Collision::findCandidates(vector<Body> bodies) {
    vector<CollisionPair> candidates;
    unordered_map<tuple<int, int, int>, vector<Body>> grid;
    unordered_map<CollisionPair, bool> cache;
    return candidates;
}
