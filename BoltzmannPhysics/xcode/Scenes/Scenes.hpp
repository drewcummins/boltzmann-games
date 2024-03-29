//
//  Scenes.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/10/17.
//
//

#include "Scene.hpp"
#include "Constraints.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Character.hpp"
#include "SimbiconCharacter.hpp"
#include "CharacterScene.hpp"
#include "Constants.hpp"

using namespace std;
using namespace bltz;

void motorScene(Scene *scene) {
    scene->cam.lookAt(vec3(0,3,15), vec3(0,2,0));
    scene->deltaTime = 1/200.f;
    scene->defaultIsland->solverIterations = 10;
    
    float s = 1;
    
    auto box = Box::create(vec3(s,s/2,s/2));
    auto body = RigidBody::create(box, 1.f);
    
    body->setPosition(vec3(0,4,0));
    scene->addBody(body);
    
    auto hinge = HingeJoint::create(body, vec3(box->extents.x/2,0,0), vec3(0,0,1), scene->ground);
    hinge->setMotor(12);
    scene->addConstraint(hinge);
}

void muscleScene(CharacterScene *scene) {
    scene->cam.lookAt(vec3(0,1,17), vec3(0,0.5,0));
    scene->solverIterations = 10;
//    scene->deltaTime = 1/120.f;
    scene->yTarget = 0.5f;
//    scene->addCharacter(1.3f);
}

void dudeScene(CharacterScene *scene) {
    scene->cam.lookAt(vec3(0,3,25), vec3(0,2,0));
    
    shared_ptr<Character> character = Character::create();
    character->setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
    //        character.brain = shared_ptr<Brain>(new Brain(10));
    character->brain = shared_ptr<MatsuokaNetwork>(new MatsuokaNetwork(10));
    
    Isle island = scene->createIsland(3);
    scene->addCharacter(character, island->id);
}



void simbiconScene(CharacterScene *scene) {
    scene->cam.lookAt(vec3(0,3,25), vec3(0,2,0));
//    scene->deltaTime = 1/240.f;
    
    shared_ptr<SimbiconCharacter> character = SimbiconCharacter::create();
    character->setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
    
//    Isle island = scene->createIsland(3);
    scene->addCharacter(character, 0);
}

void hingeScene(Scene *scene) {
    scene->cam.lookAt(vec3(0,3,15), vec3(0,2,0));
    scene->deltaTime = 1/200.f;
    scene->defaultIsland->solverIterations = 10;
    
    float s = 1;
    
    auto box = Box::create(vec3(s,s/2,s/2));
    auto body = RigidBody::create(box, 1.f);
    
    body->setPosition(vec3(0,4,0));
    scene->addBody(body);
    
    auto hinge = HingeJoint::create(body, vec3(box->extents.x/2,0,0), vec3(0,0,1), scene->ground);
    hinge->setLimits(-glm::pi<float>()/4.f, glm::pi<float>()/4);
    scene->addConstraint(hinge);
}

void bodyScene(Scene *scene) {
    scene->cam.lookAt(vec3(0,3,35), vec3(0,1,0));
    scene->solverIterations = 30;
    scene->yTarget = 1.f;
    
    Character character;
    character.setup(1.0, vec3(0,8,0));
    
    for (auto &bone : character.getBones()) {
        scene->addBody(bone);
    }
    
    for (auto &joint : character.getJoints()) {
        scene->addConstraint(joint);
    }
    
    scene->addConstraint(BallAndSocketJoint::create(character.pelvis, vec3(0,0,0), scene->ground));
}


void compositeScene(Scene *scene) {
    auto body = RigidBody::create();
    
    Material wood = {3.f, 0.96, 0.f};
    Material heavyWood = {11.f, 0.96, 0.f};
    auto sphere = bltz::Sphere::create(1.f);
    auto box = Box::create(vec3(2,2,2));
    
    body->addElement(box, heavyWood, vec3(2,3,0));
    body->addElement(box, wood, vec3(-2,4,0));
    
    scene->addBody(body);
}


void simpleScene(Scene *scene) {
    auto box = Box::create(vec3(2,1,2));
    auto body = RigidBody::create(box, 3.f);
    body->setPosition(vec3(0.1, 6, 0.1));
    scene->addBody(body);
    
    auto sphere = bltz::Sphere::create(1.f);
    body = RigidBody::create(sphere, 3.f);
    body->setPosition(vec3(0, 3, 0));
    scene->addBody(body);
}


void seesawScene(Scene *scene) {
    scene->solverIterations = 30;
    scene->cam.lookAt(vec3(0,4,25), vec3(0,2,0));
    
    auto plank = Box::create(vec3(10,1,1));
    Body seesaw = RigidBody::create(plank, 3.f);
    seesaw->setPosition(vec3(0,3,0));
    
    scene->addConstraint(HingeJoint::create(seesaw, vec3(), vec3(0,0,1), scene->ground));
    
    auto box = Box::create(vec3(3,1,1));
    Body actor1 = RigidBody::create(box, 3.f);
    actor1->setPosition(vec3(-3.5,4,0));
    
    
    Body actor2 = RigidBody::create(box, 1.f);
    actor2->setRotation(vec3(0,0,1), glm::pi<float>()*0.5);
    actor2->setPosition(vec3(-1.5,5,0));
    
    scene->addBody(seesaw);
    scene->addBody(actor1);
    scene->addBody(actor2);
}



void stonehengeScene(Scene *scene) {
    scene->solverIterations = 20;
    Shape box = Box::create(vec3(0.5,4,3));
    Body leg1 = RigidBody::create(box, 0.6f);
    Body leg2 = RigidBody::create(box, 3.f);
    leg1->setPosition(vec3(-1.75,2,0));
    leg2->setPosition(vec3(1.75,2,0));

    Body top = RigidBody::create(box, 3.f);
    top->setRotation(vec3(0,0,1), glm::pi<float>()*0.5);
    top->setPosition(vec3(0,4.25,0));

    box = Box::create(vec3(1,1,1));
    Body b = RigidBody::create(box, 3.f);
    b->setPosition(vec3(0,7,0));

    scene->addBody(b);
    scene->addBody(leg1);
    scene->addBody(leg2);
    scene->addBody(top);
}



void bridgeScene(Scene *scene) {
    scene->cam.lookAt(vec3(0,3,35), vec3(0,2,0));
    scene->solverIterations = 30;
    
    Isle island = scene->createIsland(1);
    
    auto box = Box::create(vec3(2,1,1));
    auto cube = Box::create(vec3(1,1,1));
    Material material = {3.f, 3.6, 0};
    
    // create the planks
    for (int i = 0; i < 10; i++) {
        auto body = RigidBody::create();
        body->addElement(box, material);
        body->addElement(cube, material, vec3(0,0.5,0));
        body->setPosition(vec3(-9+i*2, 6.5, 0));
        scene->addBody(body, island->id);
    }
    
    // create the joints between each neighboring pair
    for (int i = 1; i < 10; i++) {
        auto b1 = island->bodies[i];
        auto b2 = island->bodies[i-1];
        Hinge joint = HingeJoint::create(b1, vec3(-1,0,0), vec3(0,0,1), b2);
        joint->setLimits(-0.1, 0.1);
        scene->addConstraint(joint);
    }
    
    // create the end-joints to ground
    scene->addConstraint(HingeJoint::create(island->bodies[0], vec3(-1,0,0), vec3(0,0,1), scene->ground), island->id);
    scene->addConstraint(HingeJoint::create(island->bodies[9], vec3( 1,0,0), vec3(0,0,1), scene->ground), island->id);
}


