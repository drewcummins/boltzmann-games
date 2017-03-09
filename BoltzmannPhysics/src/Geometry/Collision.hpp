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
        set<CandidatePair> findCandidates(vector<Body> bodies, int gridSize=1);
    protected:
        
    };
    
}

#endif /* Collision_hpp */
