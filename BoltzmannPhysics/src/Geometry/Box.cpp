//
//  Box.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/7/17.
//
//

#include <stdio.h>
#include "Shape.hpp"
#include "Utils.hpp"

using namespace bltz;



Shape Box::create(vec3 extents) {
    Shape box(new Box(extents));
    return box;
}



static const int PRIMES[8] = {2,3,5,7,11,13,17,19};
static int getProduct(vector<int> face, vector<int> idxs) {
    int product = 1;
    for (int i = 0; i < idxs.size(); i++) {
        int vertex = face[idxs[i]];
        product *= PRIMES[vertex];
    }
    return product;
}
static int getProduct(vector<int> face) {
    int product = 1;
    for (int i = 0; i < face.size(); i++) {
        int vertex = face[i];
        product *= PRIMES[vertex];
    }
    return product;
}

Box::Box(vec3 extents) : extents(extents) {
    type = 2;
    if (faceMap.size() < 1) {
        typeMap[1] = VERTEX;
        typeMap[2] = EDGE;
        typeMap[4] = FACE;
        vector<vec3> axes = {vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), vec3(-1,0,0), vec3(0,-1,0), vec3(0,0,-1)};
        for (auto &axis : axes) {
            vector<int> face = getFace(axis);
            vector<int> keys = {
                getProduct(face, {0,1,2,3}),
                getProduct(face, {0,1,2}),
                getProduct(face, {1,2,3}),
                getProduct(face, {0,2,3}),
                getProduct(face, {0,1,3}),
                getProduct(face, {0,2}),
                getProduct(face, {1,3}),
            };
            for (auto &key : keys) {
                faceMap[key] = face;
            }
        }
    }
}

void Box::prepareView(gl::GlslProgRef shader, gl::GlslProgRef shadowShader) {
    geom::Cube cube = geom::Cube();
    cube.size(extents);
    view = gl::Batch::create(cube, shader);
    shadow = gl::Batch::create(cube, shadowShader);
}

float Box::computeMass(float density) {
    return extents.x * extents.y * extents.z * density;
}

mat3 Box::computeInertiaTensor(float mass) {
    mat3 I = mat3();
    vec3 inflated(extents * 1.25f);
    I[0][0] = (inflated.y*inflated.y + inflated.z*inflated.z) / 12.f;
    I[1][1] = (inflated.x*inflated.x + inflated.z*inflated.z) / 12.f;
    I[2][2] = (inflated.y*inflated.y + inflated.x*inflated.x) / 12.f;
    return I * mass;
}

vector<vec3> Box::getVerticesInWorldSpace(vec3 x, mat3 R) {
    vector<vec3> vertices;
    vec3 r = extents / 2.f;
    vertices.push_back(x + R * vec3(r.x,r.y,r.z));
    vertices.push_back(x + R * vec3(r.x,-r.y,r.z));
    vertices.push_back(x + R * vec3(r.x,r.y,-r.z));
    vertices.push_back(x + R * vec3(r.x,-r.y,-r.z));
    vertices.push_back(x + R * vec3(-r.x,r.y,r.z));
    vertices.push_back(x + R * vec3(-r.x,-r.y,r.z));
    vertices.push_back(x + R * vec3(-r.x,r.y,-r.z));
    vertices.push_back(x + R * vec3(-r.x,-r.y,-r.z));
    return vertices;
}

vector<int> Box::getFace(vec3 axis) {
    if (axis.x > 0) {
        return {0,2,3,1};
    } else if (axis.x < 0) {
        return {4,5,7,6};
    } else if (axis.y > 0) {
        return {0,4,6,2};
    } else if (axis.y < 0) {
        return {1,3,7,5};
    } else if (axis.z > 0) {
        return {0,1,5,4};
    }
    // -z
    return {2,6,7,3};
}

Box::Support Box::getSupport(vector<vec3> vertices, vec3 axis) {
    Box::Support support;
    support.max.proj = -FLT_MAX;
    support.min.proj = FLT_MAX;
    for (int i = 0; i < vertices.size(); i++) {
        vec3 vertex = vertices[i];
        
        float proj = dot(axis, vertex);
        
        float dmin = support.min.proj - proj;
        float dmax = support.max.proj - proj;
        
        if (abs(dmin) < 0.005) {
            support.min.vertices.push_back(i);
        } else if (proj < support.min.proj) {
            support.min.vertices = {i};
            support.min.proj = proj;
        }
        
        if (abs(dmax) < 0.005) {
            support.max.vertices.push_back(i);
        } else if (proj > support.max.proj) {
            support.max.vertices = {i};
            support.max.proj = proj;
        }
    }
    
    int minSig = getProduct(support.min.vertices);
    int maxSig = getProduct(support.max.vertices);
    
    if (faceMap.find(minSig) != faceMap.end()) {
        support.min.vertices = faceMap[minSig];
    }
    
    if (faceMap.find(maxSig) != faceMap.end()) {
        support.max.vertices = faceMap[maxSig];
    }
    
    for (int i = 0; i < support.min.vertices.size(); i++) {
        support.min.v.push_back(vertices[support.min.vertices[i]]);
    }
    
    for (int i = 0; i < support.max.vertices.size(); i++) {
        support.max.v.push_back(vertices[support.max.vertices[i]]);
    }
    
    support.min.type = typeMap[support.min.vertices.size()];
    support.max.type = typeMap[support.max.vertices.size()];
    
    return support;
}

Box::Overlap Box::getOverlap(Support s1, Support s2, bool &doesOverlap) {
    Overlap overlap;
    if (s1.min.proj > s2.max.proj + 0.05 ||
        s1.max.proj < s2.min.proj - 0.05) {
        doesOverlap = false;
    } else {
        float o1 = s2.max.proj - s1.min.proj;
        float o2 = s1.max.proj - s2.min.proj;
        if (abs(o1) < abs(o2)) {
            overlap.f1 = s1.min;
            overlap.f2 = s2.max;
            overlap.overlap = o1;
        } else {
            overlap.f1 = s1.max;
            overlap.f2 = s2.min;
            overlap.overlap = o2;
        }
        doesOverlap = true;
    }
    return overlap;
}

vector<vec3> Box::getManifold(Overlap overlap) {
    if (overlap.f1.type == FACE) {
        if (overlap.f2.type == VERTEX) {
            return overlap.f2.v;
        }
        return clipToFace(overlap.f1, overlap.f2);
    } else if (overlap.f1.type == EDGE) {
        if (overlap.f2.type == VERTEX) {
            return overlap.f2.v;
        }
        return clipToEdge(overlap.f1, overlap.f2);
    } else {
        return overlap.f1.v;
    }
}

vector<vec3> Box::clipToFace(Feature face, Feature feat, bool stop) {
    vector<vec2> f1 = Utils::projectPointsOntoPlane(face.v, face.v[0], face.axis);
    vector<vec2> f2 = Utils::projectPointsOntoPlane(feat.v, face.v[0], face.axis);
    vector<vec2> out(f2.begin(), f2.end());
    vec2 L = f1[f1.size()-1];
    bool isClockwise = Utils::isClockwise(f1);
    for (int i = 0; i < f1.size(); i++) {
        vector<vec2> in(out.begin(), out.end());
        out = {};
        vec2 d = f1[i] - L;
        
        vec2 n;
        if (isClockwise) {
            n.x = -d.y;
            n.y = d.x;
        } else {
            n.x = d.y;
            n.y = -d.x;
        }
        
        if (stop) {
            n = -n;
        }
        
        vec2 S = in[in.size()-1];
        for (int j = 0; j < in.size(); j++) {
            vec2 E = in[j];
            bool Sinside = dot(S-L, n) > 0;
            if (dot(E-L, n) > 0) {
                // inside clip polygon
                if (!Sinside) {
                    // S outside polygon, clip edge:
                    vector<vec2> intersection = Utils::lineLineIntersction(L, f1[i], S, E, false);
                    if (intersection.size()) {
                        out.push_back(intersection[0]);
                    }
                }
                out.push_back(E);
            } else if (Sinside) {
                // S outside polygon, clip edge:
                vector<vec2> intersection = Utils::lineLineIntersction(L, f1[i], S, E, false);
                if (intersection.size()) {
                    out.push_back(intersection[0]);
                }
            }
            S = E;
        }
        if (out.size() < 1) {
            if(!stop) return clipToFace(face, feat, true);
            // face is completely inside feature face
            return feat.v;
        }
        L = f1[i];
    }
    
    return Utils::unprojectPointsFromPlane(out, face.v[0], face.axis);
}

vector<vec3> Box::clipToEdge(Feature edge, Feature feat) {
    if (feat.type == FACE) {
        return clipToFace(feat, edge);
    } else if (feat.type == EDGE) {
        // return point where line segments intersect
        vector<vec2> edge1 = Utils::projectPointsOntoPlane(edge.v, edge.v[0], edge.axis);
        vector<vec2> edge2 = Utils::projectPointsOntoPlane(feat.v, edge.v[0], feat.axis);
        
        vector<float> intersection = Utils::lineLineIntersctionParameters(edge1[0], edge1[1], edge2[0], edge2[1]);
        vector<vec3> out;
        for (auto t : intersection) {
            out.push_back(edge.v[0] + (edge.v[1] - edge.v[0]) * t);
        }
        return out;
    } else {
        return feat.v;
    }
}
