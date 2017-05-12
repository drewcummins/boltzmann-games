//
//  CharacterScene.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/18/17.
//
//

#include "CharacterScene.hpp"
#include "Utils.hpp"
#include "Constants.hpp"

using namespace bltz;

shared_ptr<Scene> CharacterScene::create(CharacterSceneSetup sceneSetup) {
    shared_ptr<CharacterScene> scene(new CharacterScene());
    scene->characterSetup = sceneSetup;
    return scene;
}

void CharacterScene::setup() {
    characterSetup(this);
    
    theta = atan2(cam.getEyePoint().z, cam.getEyePoint().x);
    radius = sqrt(cam.getEyePoint().z * cam.getEyePoint().z + cam.getEyePoint().x * cam.getEyePoint().x);
    h = cam.getEyePoint().y;
}

void CharacterScene::reset() {
    characters.clear();
    Scene::reset();
}

void CharacterScene::addCharacter(shared_ptr<Character> character, uint islandId) {
    for (auto &bone : character->getBones()) {
        addBody(bone, islandId);
    }
    
    for (auto &joint : character->getJoints()) {
        addConstraint(joint, islandId);
    }
    
    character->islandId = islandId;
    characters.push_back(character);
}


void CharacterScene::singleStep() {
    if (isPaused) {
        return;
    }
    
    unordered_map<uint, vector<Constraint>> constraints;
    
    for (auto &island : islands) {
//        island.second->step(deltaTime);
        constraints[island.second->id] = island.second->integrateDVAndFindCollisions(deltaTime);
    }
    
    for (auto &character : characters) {
        character->update(deltaTime);
    }
    
    for (auto &constraint : constraints) {
        islands[constraint.first]->integratePosition(deltaTime, constraint.second);
    }
    
//    for (int i = 0; i < islands.size(); i++) {
//        islands[i]->integratePosition(deltaTime, constraints[i]);
//    }
}

void CharacterScene::render() {
    Scene::render();
    gl::color(0.6, 1.0, 0.4);
    for (auto &character : characters) {
//        for (auto &muscle : character.muscles) {
//            vec3 x1, x2;
//            
////            gl::ScopedModelMatrix scpModelMatrix;
////            gl::translate(x1);
//            gl::drawVector(x1, x2);
//        }
    }
}
