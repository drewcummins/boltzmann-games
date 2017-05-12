//
//  CharacterScene.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/18/17.
//
//

#ifndef CharacterScene_hpp
#define CharacterScene_hpp

#include <stdio.h>
#include "Scene.hpp"
#include "Character.hpp"

namespace bltz {
    class CharacterScene : public Scene {
    public:
        typedef void(*CharacterSceneSetup)(CharacterScene*);
        
        void addCharacter(shared_ptr<Character> character, uint islandId);
        vector<shared_ptr<Character>> characters;
        static shared_ptr<Scene> create(CharacterSceneSetup sceneSetup);
        virtual void setup();
        virtual void reset();
        virtual void render();
    protected:
        virtual void singleStep();
        CharacterSceneSetup characterSetup;
    };
}

#endif /* CharacterScene_hpp */
