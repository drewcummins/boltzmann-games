//
//  Scene.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/9/17.
//
//

#ifndef Scene_hpp
#define Scene_hpp

#include <stdio.h>
#include "RigidBody.hpp"
#include "Constraints.h"
#include "Collision.hpp"

namespace bltz {

    using namespace ci;
    using namespace std;

    class Scene {
    public:
        static shared_ptr<Scene> create(vec3 gravity=vec3(0,-9.8,0), int solverIterations=10, float deltaTime=1/60.f);
        
        gl::TextureRef tex;
        Body ground;
        
        vec3 gravity;
        int solverIterations;
        float deltaTime, currentTime;
        
        vector<Body> bodies;
        
        vector<Constraint> constraints;
        vector<Contact> contacts;
        
//        CollisionDetector *collisionDetector;
        
        void addBody(Body body);
        
        void addConstraint(Constraint constraint);
        void removeConstraint(Constraint constraint);
        
        void step(float dt);
        void singleStep();
        
        // view
        gl::GlslProgRef shader;
        
        CameraPersp cam;
        void render();
    protected:
        Scene(vec3 gravity=vec3(0,-9.8,0), int solverIterations=10, float deltaTime=1/60.f);
    };
    
}

#endif /* Scene_hpp */
