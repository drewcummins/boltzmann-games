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
        float gridSize = 1.f;
        vector<CandidatePair> findCandidates();
        vector<CandidatePair> bruteForceFindCandidates();
        vector<Contact> findFloorContacts();
        vector<Contact> findContacts(vector<CandidatePair>);
        void createCache(vector<Body> bodies);
        void clearCache();
    protected:
        unordered_map<ull, vector<Candidate>> grid;
        vector<Candidate> cache;
    };
    
}

#endif /* Collision_hpp */
