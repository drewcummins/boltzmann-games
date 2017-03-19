//
//  CharacterScene.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/18/17.
//
//

#include "CharacterScene.hpp"
#include "Utils.hpp"

using namespace bltz;

shared_ptr<Scene> CharacterScene::create(CharacterSceneSetup sceneSetup) {
    shared_ptr<CharacterScene> scene(new CharacterScene());
    scene->characterSetup = sceneSetup;
    return scene;
}

void CharacterScene::setup() {
    characterSetup(this);
    for (auto &character : characters) {
        for (auto &motor : character.motors) {
            Isle island = bodyIslandMap[character.getBones()[0]->id];
            motor->sineCounter = island->rng.nextFloat();// * glm::pi<float>();
            motor->frequency = 1+island->rng.nextFloat()*4;
        }
    }
    theta = atan2(cam.getEyePoint().z, cam.getEyePoint().x);
    radius = sqrt(cam.getEyePoint().z * cam.getEyePoint().z + cam.getEyePoint().x * cam.getEyePoint().x);
    h = cam.getEyePoint().y;
}

void CharacterScene::reset() {
    characters.clear();
    Scene::reset();
}

void CharacterScene::addCharacter(float height) {
    Character character;
    character.setup(height, vec3(0,3,0));
    
    for (auto &bone : character.getBones()) {
        addBody(bone);
    }
    
    for (auto &joint : character.getJoints()) {
        addConstraint(joint);
    }
    
    addConstraint(BallAndSocketJoint::create(character.pelvis, vec3(0,0,0), ground));

    characters.push_back(character);
}

void CharacterScene::singleStep() {
    if (isPaused) {
        return;
    }
    
    for (auto &character : characters) {
        character.update(deltaTime);
    }
    
    for (auto &island : islands) {
        island.second->step(deltaTime);
    }
}

void CharacterScene::render() {
    Scene::render();
    gl::color(0.6, 1.0, 0.4);
    for (auto &character : characters) {
        for (auto &muscle : character.muscles) {
            vec3 x1, x2;
            float len = muscle->getMuscleLength(x1, x2);
            
//            gl::ScopedModelMatrix scpModelMatrix;
//            gl::translate(x1);
            gl::drawVector(x1, x2);
        }
    }
}
