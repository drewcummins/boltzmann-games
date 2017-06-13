#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Constraints.h"
#include "Scene.hpp"
#include "Scenes.hpp"
#include "CharacterScene.hpp"
#include "Evolution.hpp"
#include "Constants.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace bltz;

class BoltzmannPhysicsApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
    void keyDown( KeyEvent event ) override;
    shared_ptr<Scene> scene;
    Evolution *evolution;
    bool isLearning;
    shared_ptr<MotorMuscle> muscle;
};



void BoltzmannPhysicsApp::mouseDown(MouseEvent event)
{
    
}

void BoltzmannPhysicsApp::keyDown(KeyEvent event) {
    if (isLearning) {
        if(event.getCode() == KeyEvent::KEY_SPACE){
            isLearning = false;
            scene->reset();
            scene->setup();
        }
        return;
    }
    if(event.getCode() == KeyEvent::KEY_SPACE){
        scene->togglePause();
    }
    if (event.getCode() == KeyEvent::KEY_RIGHT) {
        scene->right();
    }
    if (event.getCode() == KeyEvent::KEY_LEFT) {
        scene->left();
    }
    if (event.getChar() == 'w') {
        scene->zoomIn();
    }
    
    if (event.getChar() == 'e') {
        isLearning = true;
        return;
    }
    
    if (event.getChar() == 's') {
        scene->zoomOut();
    }
    if (event.getChar() == 'r') {
        scene->reset();
        muscle = MotorMuscle::create(static_pointer_cast<HingeJoint>(scene->defaultIsland->constraints[0]));
    }
    if (event.getCode() == KeyEvent::KEY_UP) {
        scene->up();
    }
    if (event.getCode() == KeyEvent::KEY_DOWN) {
        scene->down();
    }
    if (event.getChar() == 'b') {
        scene->shoot();
    }
    if (event.getChar() == 'd') {
//        scene->drop();
        auto torso = scene->defaultIsland->bodies[1];
        torso->addTorque(vec3(3,0,0));
    }
    if (event.getChar() == 'x') {
        scene->breakConstraint();
    }
}




float getTargetInJointSpace(float thetaWorld, float thetaJoint, Body body, vec3 jointAxis) {
    
    vec3 up = orthogonal(jointAxis);
    up = normalize(cross(up, jointAxis));
    vec3 upInJointSpace = inverse(body->R) * up;
    
    vector<vec2> plane = Utils::projectPointsOntoPlane({upInJointSpace, up}, vec3(), jointAxis);
    vec2 u1 = normalize(plane[0]);
    vec2 u2 = normalize(plane[1]);
    
    float dtheta = glm::acos(dot(u1, u2));
    if (Utils::isClockwise({u1, u2, vec2()})) {
        dtheta = -dtheta;
    }
    
    return thetaJoint + dtheta;
}

void BoltzmannPhysicsApp::update()
{
    if (isLearning) {
        evolution->next();
        if (evolution->currentGeneration > 5000) {
            isLearning = false;
            scene->reset();
            scene->setup();
        }
    } else {
        if (!scene->isPaused) {
        
//            auto pelvis = scene->defaultIsland->bodies[0];
//            auto torso = scene->defaultIsland->bodies[1];
//            Hinge joint = static_pointer_cast<HingeJoint>(scene->defaultIsland->constraints[0]);
////            joint->hasLimits = false;
//            float theta = getTargetInJointSpace(0.f, joint->cacheTheta(), torso, joint->a1);
//    //        cout << "HJ: " << joint->cacheTheta() << endl;
//    //        theta = gtijs(0.f, joint);
//    //        joint->setMotor(-theta*11);
//            muscle->mapThetaToT(theta);
//            muscle->update(1/60.f);
//            scene->deltaTime = 1/600.f;
            scene->step(1/60.f);
            
        }
        
    }
//    scene->step(1/120.f);
//    scene->step(1/120.f);
}



void rotationScene(CharacterScene *scene) {
    scene->cam.lookAt(vec3(0,4,9), vec3(0,2,0));
    
    auto box1 = Box::create(vec3(1,0.25,0.25));
    auto box2 = Box::create(vec3(0.25,1,0.25));
    auto sphere = bltz::Sphere::create(0.25);
    
    Material human;
    human.density = 1.f;
    human.bounciness = 0.f;
    human.friction = 0.8f;
    
    auto pelvis = RigidBody::create();
    pelvis->addElement(box1, human);
    pelvis->setPosition(vec3(0,3,0));
    scene->addBody(pelvis);
    
    auto torso = RigidBody::create();
    torso->addElement(box2, human);
    torso->setPosition(pelvis->x + vec3(0,0.5f,0));
    scene->addBody(torso);
    
    auto joint1 = HingeJoint::create(torso, vec3(0,-0.5f,0), vec3(1,0,0), pelvis);
    joint1->setLimits(-glm::pi<float>()/4, glm::pi<float>()/4);
//    joint1->setLimits(0, 0);
    scene->addConstraint(joint1);
    
    joint1 = HingeJoint::create(pelvis, vec3(0,0,0), vec3(1,0,0), scene->ground);
    scene->addConstraint(joint1);
//    joint1->setLimits(-glm::pi<float>()/8, glm::pi<float>()/8);
}

void BoltzmannPhysicsApp::setup()
{
    
    isLearning = false;
        scene = CharacterScene::create(simbiconScene);
//    scene = CharacterScene::create(rotationScene);
    scene->setup();
    muscle = MotorMuscle::create(static_pointer_cast<HingeJoint>(scene->defaultIsland->constraints[0]));
    return;
    
    isLearning = true;
    
    evolution = new Evolution(1);
    //    evolution->runSimulation(20);
    
    
    
    //    scene = Scene::create(motorScene);
    //    scene = CharacterScene::create(muscleScene);
    
    scene = CharacterScene::create([](CharacterScene *cs) {
        
        cs->cam.lookAt(vec3(0,3,25), vec3(0,2,0));
        
        shared_ptr<Character> character = Character::create();
        character->setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
        //        character.brain = shared_ptr<Brain>(new Brain(10));
        character->brain = shared_ptr<MatsuokaNetwork>(new MatsuokaNetwork(10));
        character->brain->fromGenome(Evolution::getInstance()->winner().genes);
        
        Isle island = cs->createIsland(3);
        cs->addCharacter(character, island->id);
    });
    
    //    scene->setup();
}

void BoltzmannPhysicsApp::draw()
{
    if (!isLearning) {
        scene->render();
    }
    
}

CINDER_APP( BoltzmannPhysicsApp, RendererGl( RendererGl::Options().msaa( 4 ) ) )
