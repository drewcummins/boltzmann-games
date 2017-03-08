#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "RigidBody.hpp"
#include "Shape.hpp"
#include "Constraints.h"

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
};

void BoltzmannPhysicsApp::setup()
{
    auto sphere = bltz::Sphere::create(1.f);
    auto body = RigidBody::create(sphere, 1.f);
    
    auto ground = GroundConstraint::create(body);
}

void BoltzmannPhysicsApp::mouseDown( MouseEvent event )
{
}

void BoltzmannPhysicsApp::update()
{
}

void BoltzmannPhysicsApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP( BoltzmannPhysicsApp, RendererGl )
