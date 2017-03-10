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
    
//    auto img = loadImage( app::loadAsset( "BasketballColor.jpg" ) );
//    tex = gl::Texture::create( img );
//    tex->bind();
    //
    auto lambert = gl::ShaderDef().lambert().color();
    //    auto lambert = gl::ShaderDef().texture().lambert();
    shader = gl::getStockShader(lambert);
    //    collisionDetector = new CollisionDetector();
    //
    gl::enableDepthWrite();
    gl::enableDepthRead();
    
    auto gs = Box::create(vec3(1,1,1));
    ground = RigidBody::create(gs, 1.f);
    ground->isGround = true;
    ground->q = quat(1,0,0,0);
    ground->R = glm::toMat3(ground->q);
    ground->invIWorld = ground->R * ground->invIModel * glm::transpose(ground->R);
}



void Scene::addBody(Body body) {
    for (auto &g : body->geometry) {
        g.shape->prepareView(shader, shader);
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
    
    for (auto &body : bodies) {
        body->addForce(gravity*body->m);
        body->integrateAcceleration(deltaTime);
    }
    
    vector<Constraint> all(constraints.begin(), constraints.end());
    
    collision.createCache(bodies);
    
    vector<CandidatePair> candidates = collision.bruteForceFindCandidates();
    
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
    
    random_shuffle(all.begin(), all.end());
    
    for (auto &constraint : all) {
        constraint->prepare(deltaTime);
    }
    
    for (int k = 0; k < solverIterations; k++) {
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
        for (auto &geometry : body->geometry) {
            gl::ScopedModelMatrix scpModelMatrix;
            gl::translate(body->x);
            gl::rotate(body->q);
            gl::translate(geometry.x);
            geometry.shape->view->draw();
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
    
    //    gl::setMatricesWindow( app::getWindowSize() );
    //    Rectf drawRect( 0, 0, tex->getWidth() / 3,
    //                   tex->getHeight() / 3 );
    //    gl::draw( tex, drawRect );
}


