//
//  Scene.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/9/17.
//
//

#include "Scene.hpp"

using namespace bltz;

Scene::Scene(vec3 gravity, int solverIterations, float deltaTime) : gravity(gravity),
solverIterations(solverIterations),
deltaTime(deltaTime)
{
    currentTime = 0.f;
    cam.lookAt(vec3(0,4,12), vec3(0,2,0));
    
    auto img = loadImage( app::loadAsset( "BasketballColor.jpg" ) );
    tex = gl::Texture::create( img );
    tex->bind();
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
        body->integrateVelocity(deltaTime);
    }
    
    vector<Constraint> all(constraints.begin(), constraints.end());
    
    vector<Contact> floor = collisionDetector->buildFloorContacts(bodies);
    for (auto &contact : floor) {
        contact.pair.b2 = ground;
        ContactConstraint *cc = new ContactConstraint(contact);
        all.push_back(cc);
    }
    
    contacts = collisionDetector->findContacts(bodies);
    contacts.insert(contacts.end(), floor.begin(), floor.end());
    for (auto &contact : contacts) {
        ContactConstraint *cc = new ContactConstraint(contact);
        all.push_back(cc);
    }
    
    for (auto &constraint : all) {
        constraint->prepare(deltaTime);
    }
    
    for (int k = 0; k < solverIterations; k++) {
        for (auto &constraint : all) {
            constraint->solve(deltaTime);
        }
    }
    
    for (auto &body : bodies) {
        body->integratePosition(deltaTime);
    }
}

void Scene::render() {
    
    gl::clear(Color(0.3, 0.3, 0.3));
    gl::enableDepthRead();
    gl::enableDepthWrite();
    
    gl::setMatrices(cam);
    
    Rand rando(6);
    for (auto &body : bodies) {
        gl::ScopedModelMatrix scpModelMatrix;
        gl::translate(body->x);
        gl::rotate(body->q);
        
        gl::color(0.15f+rando.nextFloat(), 0.12f+rando.nextFloat(), 0.2f+rando.nextFloat());
        body->shape->view->draw();
    }
    
    for (auto &constraint : constraints) {
        constraint->render();
    }
    
    //    gl::color(1.0, 0.2, 0.2);
    //    gl::lineWidth(0.05);
    //
    //    for (auto &contact : contacts) {
    //        for (auto &cp : contact.manifold) {
    //            gl::pushModelMatrix();
    //            gl::translate(cp.p);
    //            gl::drawSphere(vec3(), 0.095f);
    //            gl::popModelMatrix();
    //
    //        }
    //    }
    
    //    gl::setMatricesWindow( app::getWindowSize() );
    //    Rectf drawRect( 0, 0, tex->getWidth() / 3,
    //                   tex->getHeight() / 3 );
    //    gl::draw( tex, drawRect );
}


