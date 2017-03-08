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
    
    class Collision {
    public:
        vector<CollisionPair> findCandidates(vector<Body> bodies);
    protected:
        
    };
    
}

#endif /* Collision_hpp */
