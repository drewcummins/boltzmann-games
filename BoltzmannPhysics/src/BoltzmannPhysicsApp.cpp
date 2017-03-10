#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Constraints.h"
#include "Scene.hpp"

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
    shared_ptr<Scene> scene;
};

void BoltzmannPhysicsApp::setup()
{
    scene = Scene::create();
    
    auto sphere = bltz::Sphere::create(1.f);
    auto body = RigidBody::create(sphere, 1.f);
    body->x.y = 3;
    
    scene->addBody(body);
    
    auto box = Box::create(vec3(1,2,2));
    body = RigidBody::create(box, 1.f);
    body->x.y = 6;
    body->x.x = 0.1;
    scene->addBody(body);
    
//    auto ground = GroundConstraint::create(body);
}

void BoltzmannPhysicsApp::mouseDown( MouseEvent event )
{
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
