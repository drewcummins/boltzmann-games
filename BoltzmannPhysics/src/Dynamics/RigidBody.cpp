//
//  RigidBody.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include "RigidBody.hpp"
#include "Utils.hpp"

using namespace bltz;

shared_ptr<RigidBody> RigidBody::create() {
    shared_ptr<RigidBody> body(new RigidBody());
    body->com = body->x;
    body->m = body->invM = 1.f;
    body->invIModel = mat3();
    body->q = quat(1,0,0,0);
    body->R = glm::toMat3(body->q);
    body->invIWorld = body->R * body->invIModel * glm::transpose(body->R);
    body->collisionGroup = body->collisionMask = 1;
    body->isGround = false;
    return body;
}

shared_ptr<RigidBody> RigidBody::create(Shape shape, float density) {
    shared_ptr<RigidBody> body(new RigidBody());
    Material material = {density, 0.8, 0.0};
    body->addElement(shape, material);
    body->q = quat(1,0,0,0);
    body->R = glm::toMat3(body->q);
    body->invIWorld = body->R * body->invIModel * glm::transpose(body->R);
    body->collisionGroup = body->collisionMask = 1;
    body->isGround = false;
    return body;
}

void RigidBody::addElement(Shape shape, Material material, vec3 position, mat3 R) {
    Element element = {shape, material, position, R};
    elements.push_back(element);
    
    m = 0;
    com = vec3();
    mat3 I = mat3() * 0.f;
    for (auto &elem : elements) {
        float mass = elem.shape->computeMass(elem.material.density);
        m += mass;
        I += elem.shape->computeInertiaTensor(mass);
        I += (mat3() * dot(elem.x, elem.x) - glm::outerProduct(elem.x, elem.x)) * mass;
        com += elem.x * mass;
    }
    
    invM = 1.f / m;
    com *= invM;
    I -= (mat3() * dot(com, com) - glm::outerProduct(com, com)) * m;
    invIModel = inverse(I);
    invIWorld = R * invIModel * glm::transpose(R);
    xModel = -com;
    com += x;
}

void RigidBody::integrateAcceleration(float dt) {
    v += (invM * f) * dt;
    v *= 1/(1+0.1*dt);
    omega += (invIWorld * tau) * dt;
    omega *= 1/(1+0.3*dt);
    f = vec3(0);
    tau = vec3(0);
}

void RigidBody::integrateVelocity(float dt) {
    com += v * dt;
    quat qmega = quat(0, omega.x, omega.y, omega.z);
    qmega *= q;
    q.x += qmega.x * 0.5 * dt;
    q.y += qmega.y * 0.5 * dt;
    q.z += qmega.z * 0.5 * dt;
    q.w += qmega.w * 0.5 * dt;
    q = normalize(q);
    R = glm::toMat3(q);
    invIWorld = R * invIModel * glm::transpose(R);
    x = com + R * xModel;
}

void RigidBody::addForce(vec3 force) {
    f += force;
}

void RigidBody::addTorque(vec3 torque) {
    tau += torque;
}

void RigidBody::addForceAtPoint(vec3 p, vec3 force) {
    addForce(force);
    addTorque(cross(p - com, force));
}

void RigidBody::setPosition(vec3 position) {
    vec3 dx = com - x;
    x = position;
    com = x + dx;
}

void RigidBody::setRotation(vec3 axis, float theta) {
    q.x = axis.x * sin(theta/2.f);
    q.y = axis.y * sin(theta/2.f);
    q.z = axis.z * sin(theta/2.f);
    q.w = cos(theta/2.f);
}

void RigidBody::addImpulseAtPoint(vec3 p, vec3 impulse) {
    v += impulse;
    omega += cross(p - com, impulse);
}

vec3 RigidBody::velocityAtPoint(vec3 p) {
    return v + cross(omega, p);
}

vec3 RigidBody::getPositionInCOMSpace(vec3 p) {
    return p + xModel;
}


RigidBody::RigidBody() {
    id = Utils::rand.nextUint();
}
