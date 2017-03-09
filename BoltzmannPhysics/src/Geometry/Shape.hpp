//
//  Shape.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef Shape_hpp
#define Shape_hpp

#include <stdio.h>
namespace bltz {
    using namespace ci;
    using namespace std;
    
    typedef struct AABB {
        vec3 min;
        vec3 max;
    } AABB;
    
    typedef struct ShapeCache {
        vector<vec3> vertices;
        AABB aabb;
    } ShapeCache;
    
    class BaseShape {
    public:
        virtual float computeMass(float density) = 0;
        virtual mat3 computeInertiaTensor(float mass) = 0;
        virtual void prepareView(gl::GlslProgRef shader, gl::GlslProgRef shadowShader) = 0;
        virtual ShapeCache cache(vec3 x, mat3 R, vec3 o) = 0;
        gl::BatchRef view;
        gl::BatchRef shadow;
        uint type;
        uint id;
    };
    
    typedef shared_ptr<BaseShape> Shape;
    
    class Sphere : public BaseShape {
    public:
        static Shape create(float radius);
        virtual float computeMass(float density);
        virtual mat3 computeInertiaTensor(float mass);
        virtual void prepareView(gl::GlslProgRef shader, gl::GlslProgRef shadowShader);
        virtual ShapeCache cache(vec3 x, mat3 R, vec3 o);
        float r;
    protected:
        Sphere(float radius);
    };
    
    class Box : public BaseShape {
    public:
        enum FeatureType {
            FACE,
            VERTEX,
            EDGE
        };
        typedef struct Feature {
            vector<int> vertices; // vertex indices
            vector<vec3> v; // actual vertices in world space
            FeatureType type;
            float proj;
            vec3 axis;
        } Feature;
        typedef struct Support {
            Feature min;
            Feature max;
        } Support;
        typedef struct Overlap {
            Feature f1, f2;
            float overlap;
        } Overlap;
        
        static Shape create(vec3 extents);
        virtual float computeMass(float density);
        virtual mat3 computeInertiaTensor(float mass);
        virtual void prepareView(gl::GlslProgRef shader, gl::GlslProgRef shadowShader);
        virtual ShapeCache cache(vec3 x, mat3 R, vec3 o);
        vec3 extents;
        vector<vec3> getVerticesInWorldSpace(vec3 x, mat3 R);
        Support getSupport(vector<vec3> vertices, vec3 axis);
        static Overlap getOverlap(Support s1, Support s2, bool &doesOverlap);
        static vector<vec3> getManifold(Overlap overlap);
        static vector<vec3> clipToFace(Feature face, Feature feat, bool stop=false);
        static vector<vec3> clipToEdge(Feature edge, Feature feat);
        static vector<int> getFace(vec3 axis);
    protected:
        Box(vec3 extents);
        unordered_map<int, vector<int>> faceMap;
        unordered_map<int, FeatureType> typeMap;
    };
    
    typedef struct Geometry {
        Shape shape;
        vec3 x; // offset
    } Geometry;
}

#endif /* Shape_hpp */
