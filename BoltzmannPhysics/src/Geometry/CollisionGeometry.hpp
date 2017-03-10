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

    typedef struct CollisionPair {
        Body b1, b2;
    } CollisionPair;
    
    typedef struct Candidate {
        Body body;
        ShapeCache cache;
        Shape shape;
        bool operator==(const Candidate &other) const {
            return body->id == other.body->id && shape->id == other.shape->id;
        }
    } Candidate;
    
    typedef struct CandidatePair {
        Candidate c1, c2;
        
        bool operator==(const CandidatePair &other) const {
            if ((c1 == other.c1 && c2 == other.c2) || (c1 == other.c2 && c2 == other.c1)) return true;
            return false;
        };

    } CandidatePair;

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
    
    enum CollisionRoutine {
        SPHERE_SPHERE = 1,
        BOX_BOX = 2,
        BOX_SPHERE = 3
    };
    
}

namespace std {
    
    template <>
    struct hash<bltz::CandidatePair>
    {
        size_t operator()(const bltz::CandidatePair& c) const
        {
            size_t h = 0;
            return h;
        }
    };
    
}

#endif /* CollisionGeometry_h */
