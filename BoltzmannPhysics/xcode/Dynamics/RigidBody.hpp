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
#include "Material.hpp"

namespace bltz {
    
    using namespace ci;
    using namespace std;
    
    typedef struct Element {
        Shape shape;
        Material material;
        vec3 x;
        mat3 R;
    } Element;
    
    class RigidBody {
    public:
        static shared_ptr<RigidBody> create(Shape shape, float density);
        static shared_ptr<RigidBody> create();
        
        uint id;
        vector<Element> elements;
        
        float m;
        float invM;
        
        vec3 x;
        vec3 xModel;
        vec3 com;
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
        
        bool didCollideWithGround = false;
        
        void integrateAcceleration(float dt);
        void integrateVelocity(float dt);
        void addElement(Shape shape, Material material, vec3 offset=vec3(), mat3 R=mat3());
        
        void addForce(vec3 force);
        void addTorque(vec3 torque);
        void addForceAtPoint(vec3 p, vec3 force);
        void addForceAtLocalPoint(vec3 p, vec3 force);
        void addImpulseAtPoint(vec3 p, vec3 force);
        void addImpulseAtLocalPoint(vec3 p, vec3 force);
        
        void setPosition(vec3 position);
        void setRotation(vec3 axis, float theta);
        
        vec3 getPositionInCOMSpace(vec3 p);
        
        vec3 velocityAtPoint(vec3 p);
        vec3 velocityAtLocalPoint(vec3 p);
    protected:
        RigidBody();
    };
    
    typedef shared_ptr<RigidBody> Body;
}

#endif /* RigidBody_hpp */
