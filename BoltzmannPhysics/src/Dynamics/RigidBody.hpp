//
//  RigidBody.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef RigidBody_hpp
#define RigidBody_hpp

#include <stdio.h>
#include "Shape.hpp"

namespace bltz {
    
    using namespace ci;
    using namespace std;
    
    class RigidBody {
    public:
        static shared_ptr<RigidBody> create(Shape shape, float density);
        
        Shape shape;
        
        float m;
        float invM;
        vec3 x;
        vec3 v;
        quat q;
        vec3 omega;
        mat3 I;
        mat3 invIModel;
        mat3 invIWorld;
        mat3 R;
        vec3 f;
        vec3 tau;
        
        uint collisionGroup;
        uint collisionMask;
        
        bool isGround;
        
        void integrateAcceleration(float dt);
        void integrateVelocity(float dt);
        
        void addForce(vec3 force);
        void addTorque(vec3 torque);
        void addForceAtPoint(vec3 p, vec3 force);
        void addForceAtLocalPoint(vec3 p, vec3 force);
        void addImpulseAtPoint(vec3 p, vec3 force);
        void addImpulseAtLocalPoint(vec3 p, vec3 force);
        
        void setRotation(vec3 axis, float theta);
        
        vec3 velocityAtPoint(vec3 p);
        vec3 velocityAtLocalPoint(vec3 p);
    protected:
        RigidBody();
    };
}

#endif /* RigidBody_hpp */
