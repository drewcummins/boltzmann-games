//
//  Sphere.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include <stdio.h>
#include "Shape.hpp"

using namespace bltz;

Shape bltz::Sphere::create(float radius) {
    Shape sphere(new bltz::Sphere(radius));
    return sphere;
}

bltz::Sphere::Sphere(float radius) : r(radius) {
    type = 1;
}

void bltz::Sphere::prepareView(gl::GlslProgRef shader, gl::GlslProgRef shadowShader) {
    geom::Sphere sphere = geom::Sphere().subdivisions(50);
    sphere.radius(r);
    view = gl::Batch::create(sphere, shader);
    shadow = gl::Batch::create(sphere, shadowShader);
}

float bltz::Sphere::computeMass(float density) {
    return density * 4 * glm::pi<float>() * r * r * r / 3.f;
}

mat3 bltz::Sphere::computeInertiaTensor(float mass) {
    float inflated = r * 1.2;
    return (2 * mass * inflated * inflated / 5.f) * mat3();
}

ShapeCache bltz::Sphere::cache(vec3 x, mat3 R, vec3 o) {
    ShapeCache sphere;
    vec3 C = x + R * o;
    sphere.vertices = {C};
    sphere.aabb.min = C - vec3(r);
    sphere.aabb.max = C + vec3(r);
    return sphere;
}
