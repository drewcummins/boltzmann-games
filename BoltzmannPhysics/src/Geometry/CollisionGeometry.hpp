//
//  CollisionGeometry.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#ifndef CollisionGeometry_h
#define CollisionGeometry_h

#include "RigidBody.hpp"

namespace bltz {

    typedef pair<Body, Body> CollisionPair;

    typedef struct ContactPoint {
        vec3 normal;
        vec3 p;
        float penetration;
        vec3 u1;
        vec3 u2;
    } ContactPoint;

    typedef vector<ContactPoint> Manifold;

    typedef struct Contact {
        Manifold manifold;
        CollisionPair pair;
    } Contact;

    static const Contact NULL_CONTACT = {};
    
}

#endif /* CollisionGeometry_h */
