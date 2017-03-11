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
        void calculateMaterialProperties(Contact &contact, Material m1, Material m2);
        vector<Candidate> cache;
        Contact OBBOBB(CandidatePair boxes);
        bool addBoxAxis(vec3 axis, Body box1, shared_ptr<Box> b1, vector<vec3> b1verts, Body box2, shared_ptr<Box> b2, vector<vec3> b2verts, Box::Overlap &overlap);
    };
    
}

#endif /* Collision_hpp */
