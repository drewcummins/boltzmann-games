//
//  Collision.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "Collision.hpp"

using namespace bltz;

bool Collision::addBoxAxis(vec3 axis, Body box1, shared_ptr<Box> b1, vector<vec3> b1verts, Body box2, shared_ptr<Box> b2, vector<vec3> b2verts, Box::Overlap &overlap) {
    axis = normalize(axis);
    Box::Support s1 = b1->getSupport(b1verts, axis);
    Box::Support s2 = b2->getSupport(b2verts, axis);
    bool doesOverlap;
    Box::Overlap o = Box::getOverlap(s1, s2, doesOverlap);
    if (doesOverlap && o.overlap < overlap.overlap) {
        overlap = o;
        overlap.f1.axis = overlap.f2.axis = axis;
    }
    return doesOverlap;
}

Contact Collision::OBBOBB(CandidatePair boxes) {
    auto box1 = boxes.c1.body;
    auto box2 = boxes.c2.body;
    
    auto b1 = static_pointer_cast<Box>(boxes.c1.shape);
    auto b2 = static_pointer_cast<Box>(boxes.c2.shape);
    
    vector<vec3> b1verts = boxes.c1.cache.vertices;
    vector<vec3> b2verts = boxes.c2.cache.vertices;
    
    Box::Overlap overlap;
    overlap.overlap = FLT_MAX;
    
    vector<vec3> axes;
    
    vec3 axis = box1->R * vec3(1,0,0);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = box1->R * vec3(0,1,0);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = box1->R * vec3(0,0,1);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = box2->R * vec3(1,0,0);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = box2->R * vec3(0,1,0);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = box2->R * vec3(0,0,1);
    if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    axes.push_back(axis);
    
    axis = cross(axes[0], axes[3]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[0], axes[4]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[0], axes[5]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[1], axes[3]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[1], axes[4]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[1], axes[5]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[2], axes[3]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[2], axes[4]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    axis = cross(axes[2], axes[5]);
    if (glm::length2(axis) > 1e-6) {
        axis = normalize(axis);
        if (!addBoxAxis(axis, box1, b1, b1verts, box2, b2, b2verts, overlap)) return NULL_CONTACT;
    }
    
    Contact contact;
    contact.pair = {box1, box2};
    
    axis = overlap.f1.axis;
    
    vec3 dbox = box2->com - box1->com;
    if (dot(dbox, axis) < 0) {
        axis = -axis;
    }
    
    vector<vec3> m = Box::getManifold(overlap);
    for (auto &p : m) {
        ContactPoint cp;
        cp.normal = axis;
        cp.p = p;
        cp.penetration = overlap.overlap;
        vec3 ut = vec3(-p.y,p.z,p.x);
        cp.u1 = normalize(cross(ut, cp.normal));
        cp.u2 = normalize(cross(cp.u1, cp.normal));
        contact.manifold.push_back(cp);
    }
    
    return contact;
}

ull dumb_hash(int x, int y, int z) {
    return (ull) (((ull) (x & 0xFFFFF)) << 40) | (((ull) (y & 0xFFFFF)) << 20) | ((ull) (z & 0xFFFFF));
}

vector<CandidatePair> Collision::bruteForceFindCandidates() {
    vector<CandidatePair> candidates;
    
    for (int i = 0; i < cache.size()-1; i++) {
        for (int j = i+1; j < cache.size(); j++) {
            candidates.push_back({cache[i], cache[j]});
        }
    }
    
    return candidates;
}

vector<CandidatePair> Collision::findCandidates() {
    vector<CandidatePair> candidates;
    unordered_map<ull, vector<Candidate>> grid;
    
    for (auto &candidate : cache) {
        ShapeCache sc = candidate.cache;
        int sx = floor(sc.aabb.min.x / gridSize);
        int sy = floor(sc.aabb.min.y / gridSize);
        int sz = floor(sc.aabb.min.z / gridSize);
        int ex = floor(sc.aabb.max.x / gridSize);
        int ey = floor(sc.aabb.max.y / gridSize);
        int ez = floor(sc.aabb.max.z / gridSize);
        for (int x = sx; x <= ex; x++) {
            for (int y = sy; y <= ey; y++) {
                for (int z = sz; z <= ez; z++) {
                    ull hash = dumb_hash(x, y, z);
                    if (grid.find(hash) == grid.end()) {
                        grid[hash] = {candidate};
                    } else {
                        grid[hash].push_back(candidate);
                    }
                }
            }
        }
    }
    
    unordered_map<uint, bool> filter;
    
    for (auto &cell : grid) {
        for (int i = 0; i < cell.second.size() - 1; i++) {
            Candidate ci = cell.second[i];
            for (int j = i + 1; j < cell.second.size(); j++) {
                Candidate cj = cell.second[j];
                if (filter[ci.cache.id ^ cj.cache.id] || ci.body == cj.body) {
                    continue;
                }
                // i guess this is sorta risky as a false positive will
                // mean we don't check a pair
                filter[ci.cache.id ^ cj.cache.id] = true;
                if (!(ci == cj) && (ci.body->collisionMask & cj.body->collisionGroup)) {
                    candidates.push_back({ci, cj});
                }
            }
        }
    }
    
    return candidates;
}


vector<Contact> Collision::findFloorContacts() {
    vector<Contact> contacts;
    for (auto candidate : cache) {
        if (candidate.cache.aabb.min.y <= 0) {
            if (candidate.shape->type == SPHERE) {
                // sphere
                auto sphere = static_pointer_cast<bltz::Sphere>(candidate.shape);
                
                Contact contact;
                contact.pair = {candidate.body};
                
                ContactPoint cp;
                cp.p = candidate.cache.C - vec3(0, sphere->r, 0); // center point of circle minus radius
                cp.normal = vec3(0,-1,0);
                cp.penetration = -cp.p.y;
                cp.u1 = vec3(1,0,0);
                cp.u2 = vec3(0,0,1);
                
                contact.friction = candidate.material.friction;
                contact.bounciness = candidate.material.bounciness;
                
                contact.manifold = {cp};
                contacts.push_back(contact);
                
            } else if (candidate.shape->type == BOX) {
                // box
                auto box = static_pointer_cast<Box>(candidate.shape);
                
                Contact contact;
                contact.pair = {candidate.body};
                
                vec3 normal = vec3(0,-1,0);
                
                for (auto &vertex : candidate.cache.vertices) {
                    if (vertex.y <= 0.05) {
                        contact.manifold.push_back({normal, vertex, -vertex.y, vec3(1,0,0), vec3(0,0,1)});
                    }
                }
                
                contact.friction = candidate.material.friction;
                contact.bounciness = candidate.material.bounciness;
                contacts.push_back(contact);
            }
        }
    }
    return contacts;
}

vector<Contact> Collision::findContacts(vector<CandidatePair> pairs) {
    vector<Contact> contacts;
    for (auto &candidate : pairs) {
        CollisionRoutine routine = static_cast<CollisionRoutine>((int) candidate.c1.shape->type | candidate.c2.shape->type);
        if (routine == SPHERE_SPHERE) {
            
            vec3 C1 = candidate.c1.cache.C;
            vec3 C2 = candidate.c2.cache.C;
            vec3 normal = C2 - C1;
            float lenSq = length2(normal);
            auto s1 = static_pointer_cast<bltz::Sphere>(candidate.c1.shape);
            auto s2 = static_pointer_cast<bltz::Sphere>(candidate.c2.shape);
            if (lenSq <= pow(s1->r + s2->r, 2)) {
                CollisionPair pair = {candidate.c1.body, candidate.c2.body};
                Contact contact;
                contact.pair = pair;
                
                float len = sqrt(lenSq);
                
                ContactPoint cp;
                cp.normal = normal / len;
                cp.penetration = s1->r + s2->r - len;
                cp.p = C1 + s1->r * cp.normal;
                vec3 ut = vec3(-cp.normal.z, cp.normal.x, cp.normal.y);
                cp.u1 = cross(ut, cp.normal);
                cp.u2 = cross(cp.u1, cp.normal);
                contact.manifold = {cp};
                calculateMaterialProperties(contact, candidate.c1.material, candidate.c2.material);
                contacts.push_back(contact);
            }
            
        } else if (routine == BOX_SPHERE) {
            shared_ptr<bltz::Sphere> s;
            shared_ptr<Box> b;
            Body box, sphere;
            vec3 B, S;
            if (candidate.c1.shape->type == SPHERE) {
                sphere = candidate.c1.body;
                s = static_pointer_cast<bltz::Sphere>(candidate.c1.shape);
                S = candidate.c1.cache.C;
                box = candidate.c2.body;
                b = static_pointer_cast<Box>(candidate.c2.shape);
                B = candidate.c2.cache.C;
            } else {
                sphere = candidate.c2.body;
                s = static_pointer_cast<bltz::Sphere>(candidate.c2.shape);
                S = candidate.c2.cache.C;
                box = candidate.c1.body;
                b = static_pointer_cast<Box>(candidate.c1.shape);
                B = candidate.c1.cache.C;
            }
            
            vec3 dx = S - B;
            mat3 invR = inverse(box->R);
            vec3 sibs = invR * dx; // sphere in box space
            vec3 c = vec3(sibs.x, sibs.y, sibs.z);
            
            if (c.x > b->extents.x/2.f) {
                c.x = b->extents.x/2.f;
            } else if (c.x < -b->extents.x/2.f) {
                c.x = -b->extents.x/2.f;
            }
            
            if (c.y > b->extents.y/2.f) {
                c.y = b->extents.y/2.f;
            } else if (c.y < -b->extents.y/2.f) {
                c.y = -b->extents.y/2.f;
            }
            
            if (c.z > b->extents.z/2.f) {
                c.z = b->extents.z/2.f;
            } else if (c.z < -b->extents.z/2.f) {
                c.z = -b->extents.z/2.f;
            }
            
            dx = sibs - c;
            if (length2(dx) <= s->r*s->r) {
                CollisionPair pair = {sphere, box};
                Contact contact;
                contact.pair = pair;
                ContactPoint cp;
                vec3 cpws = B + box->R * c;
                cp.normal = cpws - S;
                float len = length(cp.normal);
                cp.normal /= len;
                cp.p = cpws;
                cp.penetration = s->r - len;
                vec3 ut = vec3(-cp.normal.y, cp.normal.z, cp.normal.x);
                cp.u1 = cross(ut, cp.normal);
                cp.u2 = cross(cp.u1, cp.normal);
                contact.manifold = {cp};
                calculateMaterialProperties(contact, candidate.c1.material, candidate.c2.material);
                contacts.push_back(contact);
            }
        } else if (routine == BOX_BOX) {
            Contact contact = OBBOBB(candidate);
            if (contact.manifold.size()) {
                calculateMaterialProperties(contact, candidate.c1.material, candidate.c2.material);
                contacts.push_back(contact);
            }
        }
    }
    return contacts;
}

void Collision::createCache(vector<Body> bodies) {
    for (auto &body : bodies) {
        for (auto &elem : body->elements) {
            Candidate ci;
            ci.body = body;
            ci.cache = elem.shape->cache(body->com, body->R, elem.x + body->xModel);
            ci.shape = elem.shape;
            ci.material = elem.material;
            cache.push_back(ci);
        }
    }
}

void Collision::clearCache() {
    cache.clear();
}

void Collision::calculateMaterialProperties(Contact &contact, Material m1, Material m2) {
    contact.bounciness = (m1.bounciness + m2.bounciness) * 0.5;
    contact.friction = (m1.friction + m2.friction) * 0.5;
}

