//
//  Collision.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#ifndef Collision_hpp
#define Collision_hpp

#include <stdio.h>
#include "CollisionGeometry.hpp"

namespace bltz {
    
    typedef unsigned long long ull;
    
    class Collision {
    public:
        int gridSize;
        vector<CandidatePair> findCandidates(vector<Body> bodies);
        vector<CandidatePair> bruteForceFindCandidates(vector<Body> bodies);
        vector<Contact> findFloorContacts(vector<Body> bodies);
    protected:
        unordered_map<ull, vector<Candidate>> grid;
        unordered_map<uint, Candidate> cache;
    };
    
}

#endif /* Collision_hpp */
