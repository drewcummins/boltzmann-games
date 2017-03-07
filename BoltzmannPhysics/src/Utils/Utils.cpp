//
//  Utils.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include "Utils.hpp"

using namespace bltz;


mat3 Utils::skew(vec3 v) {
    return mat3( 0.0, -v.z,  v.y,
                v.z,  0.0, -v.x,
                -v.y,  v.x,  0.0);
}

vec3 Utils::orthogonal(vec3 v) {
    return vec3(v.y, v.z, -v.x);
}

float Utils::clamp(float value, float min, float max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

bool Utils::isPointInPolygon(vector<vec3> poly, vec3 x) {
    vec3 p = poly[poly.size()-1];
    for (auto &v : poly) {
        vec3 edge = v - p;
        vec3 conn = x - p;
        vec3 vp;
        float t = dot(edge, conn) / length2(edge);
        if (t < 0) {
            vp = p;
        } else if (t > 1) {
            vp = v;
        } else {
            vp = p + edge * t;
        }
    }
    return false;
}

vector<vec2> Utils::projectPointsOntoPlane(vector<vec3> points, vec3 x, vec3 n) {
    vector<vec2> pts;
    vec3 norm = orthogonal(n);
    vec3 b1 = normalize(cross(n, norm));
    vec3 b2 = normalize(cross(n, b1));
    for (auto &pt : points) {
        vec2 pt2;
        pt2.x = dot(pt - x, b1);
        pt2.y = dot(pt - x, b2);
        pts.push_back(pt2);
    }
    return pts;
}

vector<vec3> Utils::unprojectPointsFromPlane(vector<vec2> points, vec3 x, vec3 n) {
    vector<vec3> pts;
    vec3 norm = orthogonal(n);
    vec3 b1 = normalize(cross(n, norm));
    vec3 b2 = normalize(cross(n, b1));
    for (auto &pt : points) {
        vec3 pt3 = x;
        pt3 += b1 * pt.x;
        pt3 += b2 * pt.y;
        pts.push_back(pt3);
    }
    return pts;
}

vector<vec2> Utils::lineLineIntersction(vec2 a1, vec2 a2, vec2 b1, vec2 b2, bool clipToSegment) {
    vector<vec2> intersection;
    
    vec2 ab = b1 - a1;
    vec2 va = a2 - a1;
    vec2 vb = b2 - b1;
    
    float vaxvb = va.x * vb.y - va.y * vb.x;
    
    if (length2(vaxvb) < 1e-6) {
        // parallel
        if (clipToSegment) {
            return intersection;
        }
        //        return {a1 + va*0.5f};
        return intersection;
    }
    
    float abxva = ab.x * va.y - ab.y * va.x;
    
    if (length(abxva) < 1e-6) {
        // collinear
        float pb0 = dot(ab, va);
        float pb1 = dot(ab, b2 - a1);
        vector<float> proj = {0, 1, pb0, pb1};
        sort(proj.begin(), proj.end());
        intersection.push_back(a1 + va * proj[1]);
        intersection.push_back(a1 + va * proj[2]);
    } else {
        float abxvb = ab.x * vb.y - ab.y * vb.x;
        float q = 1.f/vaxvb;
        float t = abxvb * q;
        float u = abxva * q;
        if (!clipToSegment || (t >= 0 && t <= 1.f && u >= 0 && u <= 1.f)) {
            intersection.push_back(a1 + va * t);
        }
    }
    
    return intersection;
}



vector<float> Utils::lineLineIntersctionParameters(vec2 a1, vec2 a2, vec2 b1, vec2 b2, bool clipToSegment) {
    vector<float> intersection;
    
    vec2 ab = b1 - a1;
    vec2 va = a2 - a1;
    vec2 vb = b2 - b1;
    
    float vaxvb = va.x * vb.y - va.y * vb.x;
    
    if (length2(vaxvb) < 1e-6) {
        // parallel
        return intersection;
    }
    
    float abxva = ab.x * va.y - ab.y * va.x;
    
    if (length(abxva) < 1e-6) {
        // collinear
        float pb0 = dot(ab, va);
        float pb1 = dot(ab, b2 - a1);
        vector<float> proj = {0, 1, pb0, pb1};
        sort(proj.begin(), proj.end());
        intersection.push_back(proj[1]);
        intersection.push_back(proj[2]);
    } else {
        float abxvb = ab.x * vb.y - ab.y * vb.x;
        float q = 1.f/vaxvb;
        float t = abxvb * q;
        float u = abxva * q;
        if (!clipToSegment || (t >= 0 && t <= 1.f && u >= 0 && u <= 1.f)) {
            intersection.push_back(t);
        }
    }
    
    return intersection;
}

bool Utils::isClockwise(vector<vec2> p) {
    vec2 L = p[p.size()-1];
    float area = 0.f;
    for (int i = 0; i < p.size(); i++) {
        area += (p[i].x - L.x) * (p[i].y + L.y);
    }
    return area > 0;
}

