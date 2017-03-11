//
//  ContactConstraint.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "ContactConstraint.hpp"
#include "Utils.hpp"

using namespace bltz;

Constraint ContactConstraint::create(Contact contact) {
    Constraint cc(new ContactConstraint(contact));
    return cc;
}

ContactConstraint::ContactConstraint(Contact contact) : BaseConstraint(contact.pair.b1, contact.pair.b2), contact(contact) {
    n = vector<C1DOF>(contact.manifold.size());
    u1 = vector<C1DOF>(contact.manifold.size());
    u2 = vector<C1DOF>(contact.manifold.size());
    beta = 0.2;
}

void ContactConstraint::prepare(float dt) {
    for (int i = 0; i < contact.manifold.size(); i++) {
        buildNonpenetrationConstraint(dt, i);
        ContactPoint cp = contact.manifold[i];
        buildFrictionConstraint(dt, i, cp.u1, u1);
        buildFrictionConstraint(dt, i, cp.u2, u2);
    }
}

void ContactConstraint::solve(float dt) {
    for (int i = 0; i < n.size(); i++) {
        ContactPoint cp = contact.manifold[i];
        solveFrictionConstraint(dt, i, cp.u1, u1);
        solveFrictionConstraint(dt, i, cp.u2, u2);
    }
    for (int i = 0; i < n.size(); i++) {
        solveNonpenetrationConstraint(dt, i);
    }
}

void ContactConstraint::buildNonpenetrationConstraint(float dt, int i) {
    ContactPoint cp = contact.manifold[i];
    
    n[i].lambda = 0.f;
    
    vec3 r1 = cp.p - b1->com;
    vec3 r2 = cp.p - b2->com;
    
    n[i].J.L1 = cp.normal;
    n[i].J.A1 = cross(r1, cp.normal);
    
    n[i].K = dot(n[i].J.L1 * b1->invM, n[i].J.L1) + dot(n[i].J.A1 * b1->invIWorld, n[i].J.A1);
    
    if (!b2->isGround) {
        n[i].J.L2 = -cp.normal;
        n[i].J.A2 = -cross(r2, cp.normal);
        n[i].K += dot(n[i].J.L2 * b2->invM, n[i].J.L2) + dot(n[i].J.A2 * b2->invIWorld, n[i].J.A2);
    }
    
    n[i].bias = cp.penetration * (beta/dt);
    n[i].K = 1 / n[i].K;
    
    solveNonpenetrationConstraint(dt, i);
}

void ContactConstraint::buildFrictionConstraint(float dt, int i, vec3 u, vector<C1DOF> &eqns) {
    ContactPoint cp = contact.manifold[i];
    eqns[i].J.L1 = u;
    eqns[i].J.A1 = cross(cp.p - b1->com, u);
    
    eqns[i].K = dot(eqns[i].J.L1 * b1->invM, eqns[i].J.L1) + dot(eqns[i].J.A1 * b1->invIWorld, eqns[i].J.A1);
    
    if (!b2->isGround) {
        eqns[i].J.L2 = -u;
        eqns[i].J.A2 = -cross(cp.p - b2->com, u);
        eqns[i].K += dot(eqns[i].J.L2 * b1->invM, eqns[i].J.L2) + dot(eqns[i].J.A2 * b2->invIWorld, eqns[i].J.A2);
    }
}

void ContactConstraint::solveNonpenetrationConstraint(float dt, int i) {
    ContactPoint cp = contact.manifold[i];
    float Cdot = dot(n[i].J.L1, b1->v) + dot(n[i].J.A1, b1->omega);
    if (!b2->isGround) {
        Cdot += dot(n[i].J.L2, b2->v) + dot(n[i].J.A2, b2->omega);
    }
    
    float lambda = -n[i].K * ((1+k) * Cdot + n[i].bias);
    
    n[i].lambda += lambda - n[i].lambda;
    if (n[i].lambda > 0) {
        n[i].lambda = 0;
    }
    
    b1->v += b1->invM * n[i].J.L1 * n[i].lambda;
    b1->omega += b1->invIWorld * n[i].J.A1 * n[i].lambda;
    
    if (!b2->isGround) {
        b2->v += b2->invM * n[i].J.L2 * n[i].lambda;
        b2->omega += b2->invIWorld * n[i].J.A2 * n[i].lambda;
    }
}

void ContactConstraint::solveFrictionConstraint(float dt, int i, vec3 u, vector<C1DOF> &eqns) {
    ContactPoint cp = contact.manifold[i];
    float Cdot = dot(eqns[i].J.L1, b1->v) + dot(eqns[i].J.A1, b1->omega);
    if (!b2->isGround) {
        Cdot += dot(eqns[i].J.L2, b2->v) + dot(eqns[i].J.A2, b2->omega);
    }
    
    float lambda = -eqns[i].K * Cdot;
    
    float nl = fabs(n[i].lambda) * mu;
    lambda = Utils::clamp(lambda, -nl, nl);
    
    b1->v += b1->invM * eqns[i].J.L1 * lambda;
    b1->omega += b1->invIWorld * eqns[i].J.A1 * lambda;
    
    if (!b2->isGround) {
        b2->v += b2->invM * eqns[i].J.L2 * lambda;
        b2->omega += b2->invIWorld * eqns[i].J.A2 * lambda;
    }
}

