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
        typedef void(*SceneSetup)(Scene*);
        
        static shared_ptr<Scene> create(SceneSetup sceneSetup);
        static shared_ptr<Scene> create(vec3 gravity=vec3(0,-9.8,0), int solverIterations=10, float deltaTime=1/60.f);
        
        gl::TextureRef tex;
        Body ground;
        
        vec3 gravity;
        int solverIterations;
        float deltaTime, currentTime;
        
        vector<Body> bodies;
        
        vector<Constraint> constraints;
        vector<Contact> contacts;
        
        Collision collision;
        
        void addBody(Body body);
        
        void addConstraint(Constraint constraint);
        void removeConstraint(Constraint constraint);
        
        void step(float dt);
        void singleStep();
        
        // view
        gl::GlslProgRef shader;
        
        CameraPersp cam;
        void render();
        
        virtual void left();
        virtual void right();
        virtual void up();
        virtual void down();
        virtual void setup();
        virtual void reset();
        virtual void togglePause();
        virtual void drop();
        virtual void shoot();
        virtual void zoomIn();
        virtual void zoomOut();
        
    protected:
        Scene(vec3 gravity=vec3(0,-9.8,0), int solverIterations=10, float deltaTime=1/60.f);
        
        SceneSetup sceneSetup;
        
        double time = -1;
        
        bool isPaused = false;
        float theta = 0.f;
        float radius;
        float h;
        float yTarget;
        
        void updateCamera();
    };
    
}

#endif /* Scene_hpp */
