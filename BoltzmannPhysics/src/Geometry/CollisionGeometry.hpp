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
    
    typedef struct Candidate {
        Body body;
        ShapeCache cache;
        Shape shape;
        bool operator==(const Candidate &other) const {
            return body == other.body && shape == other.shape;
        }
    } Candidate;
    
    typedef struct CandidatePair {
        Candidate c1, c2;
        
        bool operator==(const CandidatePair &other) const {
            if ((c1 == other.c1 && c2 == other.c2) || (c1 == other.c2 && c2 == other.c1)) return true;
            return false;
        };
        
        bool operator<(const CandidatePair& lhs, const CandidatePair& rhs)
        {
            return lhs.c1.body.get() < rhs.c1.body.get();
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
