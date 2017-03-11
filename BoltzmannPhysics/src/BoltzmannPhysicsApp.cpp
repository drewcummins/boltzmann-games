#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Constraints.h"
#include "Scene.hpp"
#include "Scenes.hpp"

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
};

void BoltzmannPhysicsApp::setup()
{
    scene = Scene::create(stonehengeScene);
    scene->setup();
}

void BoltzmannPhysicsApp::mouseDown( MouseEvent event )
{
    
}

void BoltzmannPhysicsApp::keyDown(KeyEvent event) {
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
}

void BoltzmannPhysicsApp::update()
{
    scene->step(1/60.f);
}

void BoltzmannPhysicsApp::draw()
{
    scene->render();
}

CINDER_APP( BoltzmannPhysicsApp, RendererGl( RendererGl::Options().msaa( 4 ) ) )
