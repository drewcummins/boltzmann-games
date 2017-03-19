//
//  Scene.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/9/17.
//
//

#include "Scene.hpp"
#include "cinder/Rand.h"
#include "Utils.hpp"
#include "Constants.hpp"

using namespace bltz;

shared_ptr<Scene> Scene::create(SceneSetup sceneSetup) {
    shared_ptr<Scene> scene(new Scene());
    scene->sceneSetup = sceneSetup;
    return scene;
}

shared_ptr<Scene> Scene::create(vec3 gravity, int solverIterations, float deltaTime) {
    shared_ptr<Scene> scene(new Scene(gravity, solverIterations, deltaTime));
    return scene;
}

Scene::Scene(vec3 gravity, int solverIterations, float deltaTime) : gravity(gravity),
solverIterations(solverIterations),
deltaTime(deltaTime)
{
    currentTime = 0.f;
    cam.lookAt(vec3(0,4,20), vec3(0,2,0));
    
    this->gravity *= METERS_TO_UNITS; // * 0.7;
    
    auto lambert = gl::ShaderDef().lambert().color();
    shader = gl::getStockShader(lambert);
    
    gl::enableDepthWrite();
    gl::enableDepthRead();
    
    auto gs = Box::create(vec3(1,1,1));
    ground = RigidBody::create(gs, 1.f);
    ground->isGround = true;
    
    defaultIsland = createIsland(Utils::rand.nextUint());
    
}



void Scene::addBody(Body body, uint islandId) {
    for (auto &elem : body->elements) {
        elem.shape->prepareView(shader, shader);
    }
    Isle island = islandId == 0 ? defaultIsland : islands[islandId];
    island->addBody(body);
    bodyIslandMap[body->id] = island;
}

void Scene::addConstraint(Constraint constraint, uint islandId) {
    Isle island = islandId == 0 ? defaultIsland : islands[islandId];
    island->addConstraint(constraint);
    constraintIslandMap[constraint->id] = island;
}

Isle Scene::createIsland(uint seed, int solverIterations) {
    Isle island = Island::create(seed, solverIterations == -1 ? this->solverIterations : solverIterations);
    island->ground = ground;
    island->gravity = gravity;
    islands[island->id] = island;
    return island;
}

void Scene::removeConstraint(Constraint constraint) {
    Isle island = constraintIslandMap[constraint->id];
    island->removeConstraint(constraint);
}

void Scene::step(float dt) {
    singleStep();
    //    float targetTime = currentTime + dt;
    //    while (currentTime <= targetTime-deltaTime) {
    //        singleStep();
    //        currentTime += deltaTime;
    //    }
}

void Scene::singleStep() {
    
    if (isPaused) {
        return;
    }
    
    for (auto &island : islands) {
        island.second->step(deltaTime);
    }
}

void Scene::render() {
    
    gl::clear(Color(0.3, 0.3, 0.3));
    gl::enableDepthRead();
    gl::enableDepthWrite();
    
    gl::setMatrices(cam);
    
    Rand rando(31);
    for (auto &island : islands) {
        float baseR = 0.15f+rando.nextFloat();
        float baseG = 0.12f+rando.nextFloat();
        float baseB = 0.2f+rando.nextFloat();
        for (auto &body : island.second->bodies) {
            gl::color(baseR + rando.nextGaussian() * 0.1, baseG + rando.nextGaussian() * 0.1, baseB + rando.nextGaussian() * 0.1);
            for (auto &elem : body->elements) {
                gl::ScopedModelMatrix scpModelMatrix;
                gl::translate(body->com);
                gl::rotate(body->q);
                gl::translate(elem.x + body->xModel);
                elem.shape->view->draw();
            }
        }
        for (auto &constraint : island.second->constraints) {
            constraint->render();
        }
    }
}


void Scene::left() {
    theta += 0.05;
    updateCamera();
}

void Scene::right() {
    theta -= 0.05;
    updateCamera();
}

void Scene::up() {
    h += 1;
    updateCamera();
}

void Scene::down() {
    h -= 1;
    updateCamera();
}

void Scene::setup() {
    sceneSetup(this);
    theta = atan2(cam.getEyePoint().z, cam.getEyePoint().x);
    radius = sqrt(cam.getEyePoint().z * cam.getEyePoint().z + cam.getEyePoint().x * cam.getEyePoint().x);
    h = cam.getEyePoint().y;
}

void Scene::reset() {
    islands.clear();
    defaultIsland = createIsland(defaultIsland->seed);
    islands[defaultIsland->id] = defaultIsland;
    setup();
}

void Scene::togglePause() {
    isPaused = !isPaused;
}

void Scene::drop() {
    auto box = Box::create(vec3(1,2,1));
    Material wood = {4.5f, 0.96, 0.0};
    auto ball = RigidBody::create();
    ball->addElement(box, wood);
    ball->setPosition(vec3(-2,20,0));
    ball->setRotation(vec3(0,1,0), Utils::rand.nextFloat() * glm::pi<float>());
    addBody(ball);
}

void Scene::shoot() {
    auto sphere = bltz::Sphere::create(0.6f);
    auto ball = RigidBody::create(sphere, 3.5f);
    ball->setPosition(vec3(-25,1,0));
    ball->v.x = 30;
    addBody(ball);
}

void Scene::zoomIn() {
    radius -= 0.5;
    updateCamera();
}

void Scene::zoomOut() {
    radius += 0.5;
    updateCamera();
}


void Scene::updateCamera() {
    cam.lookAt(vec3(cos(theta) * radius, h, sin(theta) * radius), vec3(0,yTarget,0));
}

void Scene::breakConstraint() {
    if (defaultIsland->constraints.size() > 0) {
        Constraint constraint = defaultIsland->constraints[defaultIsland->constraints.size()-1];
        defaultIsland->removeConstraint(constraint);
    }
}




