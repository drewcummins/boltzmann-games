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

using namespace std;
using namespace bltz;

void bridgeScene(Scene *scene) {
    auto box = Box::create(vec3(2,1,2));
    auto body = RigidBody::create(box, 3.f);
    body->x.y = 3;
    scene->addBody(body);
}
