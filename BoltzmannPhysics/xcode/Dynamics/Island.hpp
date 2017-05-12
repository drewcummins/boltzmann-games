//
//  Island.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/14/17.
//
//

#ifndef Island_hpp
#define Island_hpp

#include <stdio.h>
#include "Shape.hpp"
#include "Material.hpp"
#include "RigidBody.hpp"
#include "Constraints.h"
#include "Collision.hpp"
#include "cinder/Rand.h"

namespace bltz {
    
    using namespace ci;
    using namespace std;
    
    class Island {
    public:
        static shared_ptr<Island> create(uint seed=1, uint solverIterations=10);
        uint id;
        uint seed;
        uint solverIterations;
        vec3 gravity;
        Body ground;
        Rand rng;
        
        vector<Body> bodies;
        vector<Constraint> constraints;
        
        void reset();
        void addBody(Body body);
        void addConstraint(Constraint constraint);
        void removeConstraint(Constraint constraint);
        void removeBody(Body body);
        void step(float dt);
        vector<Constraint> integrateDVAndFindCollisions(float dt);
        void integratePosition(float dt, vector<Constraint> all);
    protected:
        Island();
        Collision collision;
        unordered_map<uint, bool> jointMap;
    };
    
    typedef shared_ptr<Island> Isle;
    
}

#endif /* Island_hpp */
