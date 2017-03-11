//
//  Scene.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/9/17.
//
//

#include "Scene.hpp"
#include "cinder/Rand.h"

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
    
    auto lambert = gl::ShaderDef().lambert().color();
    shader = gl::getStockShader(lambert);
    
    gl::enableDepthWrite();
    gl::enableDepthRead();
    
    auto gs = Box::create(vec3(1,1,1));
    ground = RigidBody::create(gs, 1.f);
    ground->isGround = true;
}



void Scene::addBody(Body body) {
    for (auto &elem : body->elements) {
        elem.shape->prepareView(shader, shader);
    }
    bodies.push_back(body);
}

void Scene::addConstraint(Constraint constraint) {
    constraints.push_back(constraint);
}

void Scene::removeConstraint(Constraint constraint) {
    vector<Constraint>::iterator it;
    for (it = constraints.begin(); it < constraints.end(); it++) {
        if (*it == constraint) {
            break;
        }
    }
    constraints.erase(it);
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
    
    for (auto &body : bodies) {
        body->addForce(gravity*body->m);
        body->integrateAcceleration(deltaTime);
    }
    
    vector<Constraint> all(constraints.begin(), constraints.end());
    
    collision.createCache(bodies);
    
    vector<CandidatePair> candidates = collision.findCandidates();
    
    contacts = collision.findContacts(candidates);
    
    vector<Contact> floor = collision.findFloorContacts();
    for (auto &contact : floor) {
        contact.pair.b2 = ground;
    }
    contacts.insert(contacts.end(), floor.begin(), floor.end());

    for (auto &contact : contacts) {
        Constraint cc = ContactConstraint::create(contact);
        all.push_back(cc);
    }
    
    for (auto &constraint : all) {
        constraint->prepare(deltaTime);
    }
    
    for (int k = 0; k < solverIterations; k++) {
        random_shuffle(all.begin(), all.end());
        for (auto &constraint : all) {
            constraint->solve(deltaTime);
        }
    }
    
    for (auto &body : bodies) {
        body->integrateVelocity(deltaTime);
    }
    
    collision.clearCache();
}

void Scene::render() {
    
    gl::clear(Color(0.3, 0.3, 0.3));
    gl::enableDepthRead();
    gl::enableDepthWrite();
    
    gl::setMatrices(cam);
    
    Rand rando(6);
    for (auto &body : bodies) {
        gl::color(0.15f+rando.nextFloat(), 0.12f+rando.nextFloat(), 0.2f+rando.nextFloat());
        for (auto &elem : body->elements) {
            gl::ScopedModelMatrix scpModelMatrix;
            gl::translate(body->com);
            gl::rotate(body->q);
            gl::translate(elem.x + body->xModel);
            elem.shape->view->draw();
        }
    }
    
    for (auto &constraint : constraints) {
        constraint->render();
    }
    
    gl::color(1.0, 0.2, 0.2);
    gl::lineWidth(0.05);

    for (auto &contact : contacts) {
        for (auto &cp : contact.manifold) {
            gl::pushModelMatrix();
            gl::translate(cp.p);
            gl::drawSphere(vec3(), 0.095f);
            gl::popModelMatrix();
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
    yTarget = 2;
}

void Scene::reset() {
    bodies.clear();
    constraints.clear();
    contacts.clear();
    setup();
}

void Scene::togglePause() {
    isPaused = !isPaused;
}

void Scene::drop() {
    
}

void Scene::shoot() {
    auto sphere = bltz::Sphere::create(1.f);
    auto ball = RigidBody::create(sphere, 3.f);
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





