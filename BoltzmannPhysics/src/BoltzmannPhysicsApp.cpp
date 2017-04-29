#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Constraints.h"
#include "Scene.hpp"
#include "Scenes.hpp"
#include "CharacterScene.hpp"
#include "Evolution.hpp"
#include "Constants.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace bltz;

class BoltzmannPhysicsApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
    void keyDown( KeyEvent event ) override;
    shared_ptr<Scene> scene;
    Evolution *evolution;
    bool isLearning;
};

void BoltzmannPhysicsApp::setup()
{
    
//    MatsuokaNeuron neuron;
//    neuron.mask = std::numeric_limits<uint64_t>::max();
//    cout << neuron.mask << endl;
//    cout << neuron.hasSynapse(62) << endl;
//    neuron.disconnect(62);
//    cout << neuron.mask << endl;
//    cout << neuron.hasSynapse(62) << endl;
//    neuron.connect(62);
//    cout << neuron.mask << endl;
//    cout << neuron.hasSynapse(62) << endl;
    
//    MatsuokaNetwork *brain = new MatsuokaNetwork(5);
//    for (int i = 0; i < 1000; i++) {
//        brain->update(1/100.f);
//        cout << brain->network[0]->output << ", " << brain->network[1]->output << endl;
//    }
    isLearning = true;
    
    evolution = new Evolution(1);
//    evolution->runSimulation(20);
    
    
    
//    scene = Scene::create(motorScene);
//    scene = CharacterScene::create(muscleScene);
    
    scene = CharacterScene::create([](CharacterScene *cs) {
        
        cs->cam.lookAt(vec3(0,3,25), vec3(0,2,0));
        
        Character character;
        character.setup(1.3, vec3(0,M2U(2*1.3/3.0),0));
//        character.brain = shared_ptr<Brain>(new Brain(10));
        character.brain = shared_ptr<MatsuokaNetwork>(new MatsuokaNetwork(5));
        character.brain->fromGenome(Evolution::getInstance()->winner().genes);
        
        Isle island = cs->createIsland(3);
        cs->addCharacter(character, island->id);
    });
    
//    scene->setup();
}

void BoltzmannPhysicsApp::mouseDown(MouseEvent event)
{
    
}

void BoltzmannPhysicsApp::keyDown(KeyEvent event) {
    if (isLearning) {
        if(event.getCode() == KeyEvent::KEY_SPACE){
            isLearning = false;
            scene->reset();
            scene->setup();
        }
        return;
    }
    if(event.getCode() == KeyEvent::KEY_SPACE){
        scene->togglePause();
    }
    if (event.getCode() == KeyEvent::KEY_RIGHT) {
        scene->right();
    }
    if (event.getCode() == KeyEvent::KEY_LEFT) {
        scene->left();
    }
    if (event.getChar() == 'w') {
        scene->zoomIn();
    }
    
    if (event.getChar() == 'e') {
        isLearning = true;
        return;
    }
    
    if (event.getChar() == 's') {
        scene->zoomOut();
    }
    if (event.getChar() == 'r') {
        scene->reset();
    }
    if (event.getCode() == KeyEvent::KEY_UP) {
        scene->up();
    }
    if (event.getCode() == KeyEvent::KEY_DOWN) {
        scene->down();
    }
    if (event.getChar() == 'b') {
        scene->shoot();
    }
    if (event.getChar() == 'd') {
        scene->drop();
    }
    if (event.getChar() == 'x') {
        scene->breakConstraint();
    }
}

void BoltzmannPhysicsApp::update()
{
    if (isLearning) {
        evolution->next();
        if (evolution->currentGeneration > 500) {
            isLearning = false;
            scene->reset();
            scene->setup();
        }
    } else {
        scene->step(1/60.f);
    }
//    scene->step(1/120.f);
//    scene->step(1/120.f);
}

void BoltzmannPhysicsApp::draw()
{
    if (!isLearning) {
        scene->render();
    }
    
}

CINDER_APP( BoltzmannPhysicsApp, RendererGl( RendererGl::Options().msaa( 4 ) ) )
