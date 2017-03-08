//
//  Utils.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#ifndef Utils_hpp
#define Utils_hpp

#include <stdio.h>
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace bltz {

    class Utils {
    public:
        static mat3 skew(vec3 v);
        static float clamp(float value, float min, float max);
        static vec3 vmin(vec3 a, vec3 x);
        static vec3 vmax(vec3 a, vec3 x);
        static vec3 orthogonal(vec3 v);
        static bool isPointInPolygon(vector<vec3> poly, vec3 x);
        static vector<vec2> projectPointsOntoPlane(vector<vec3> points, vec3 x, vec3 n);
        static vector<vec3> unprojectPointsFromPlane(vector<vec2> points, vec3 x, vec3 n);
        static vector<vec2> lineLineIntersction(vec2 a1, vec2 a2, vec2 b1, vec2 b2, bool clipToSegment=true);
        static vector<float> lineLineIntersctionParameters(vec2 a1, vec2 a2, vec2 b1, vec2 b2, bool clipToSegment=true);
        static bool isClockwise(vector<vec2> p);
    };
    
}

#endif /* Utils_hpp */
