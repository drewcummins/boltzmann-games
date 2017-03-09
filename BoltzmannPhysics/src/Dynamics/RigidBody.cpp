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

shared_ptr<RigidBody> RigidBody::create(Shape shape, float density) {
    shared_ptr<RigidBody> body(new RigidBody());
    body->m = density;
    body->invM = 1 / body->m;
    Geometry geom;
    geom.shape = shape;
    geom.x = vec3(0,0,0);
    body->geometry.push_back(geom);
    mat3 I = shape->computeInertiaTensor(body->m);
    body->invIModel = inverse(I);
    body->q = quat(1,0,0,0);
    body->R = glm::toMat3(body->q);
    body->invIWorld = body->R * body->invIModel * glm::transpose(body->R);
    body->collisionGroup = body->collisionMask = 1;
    return body;
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
    x += v * dt;
    quat qmega = quat(0, omega.x, omega.y, omega.z);
    qmega *= q;
    q.x += qmega.x * 0.5 * dt;
    q.y += qmega.y * 0.5 * dt;
    q.z += qmega.z * 0.5 * dt;
    q.w += qmega.w * 0.5 * dt;
    q = normalize(q);
    R = glm::toMat3(q);
    invIWorld = R * invIModel * glm::transpose(R);
}

void RigidBody::addForce(vec3 force) {
    f += force;
}

void RigidBody::addTorque(vec3 torque) {
    tau += torque;
}

void RigidBody::addForceAtPoint(vec3 p, vec3 force) {
    addForce(force);
    addTorque(cross(p - x, force));
}

void RigidBody::setRotation(vec3 axis, float theta) {
    q.x = axis.x * sin(theta/2.f);
    q.y = axis.y * sin(theta/2.f);
    q.z = axis.z * sin(theta/2.f);
    q.w = cos(theta/2.f);
}

void RigidBody::addImpulseAtPoint(vec3 p, vec3 impulse) {
    v += impulse;
    omega += cross(p - x, impulse);
}

vec3 RigidBody::velocityAtPoint(vec3 p) {
    return v + cross(omega, p);
}




RigidBody::RigidBody() {
    id = Utils::rand.nextUint();
}
