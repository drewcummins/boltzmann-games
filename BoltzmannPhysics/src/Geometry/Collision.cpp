//
//  Collision.cpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#include "Collision.hpp"

using namespace bltz;

ull dumb_hash(int x, int y, int z) {
    return (ull) (((ull) (x & 0xFFFFF)) << 40) | (((ull) (y & 0xFFFFF)) << 20) | ((ull) (z & 0xFFFFF));
}

int hash_y(ull h) {
    return (int) (h >> 20) & 0xFFFFF;
}

vector<CandidatePair> Collision::bruteForceFindCandidates(vector<Body> bodies) {
    vector<CandidatePair> candidates;
    for (int i = 0; i < bodies.size()-1; i++) {
        for (auto &gi : bodies[i]->geometry) {
            Candidate ci;
            ci.body = bodies[i];
            ci.cache = gi.shape->cache(ci.body->x, ci.body->R, gi.x);
            ci.shape = gi.shape;
            for (int j = i+1; j < bodies.size(); j++) {
                for (auto &gj : bodies[i]->geometry) {
                    Candidate cj;
                    cj.body = bodies[j];
                    cj.cache = gj.shape->cache(cj.body->x, cj.body->R, gj.x);
                    cj.shape = gj.shape;
                    candidates.push_back({ci, cj});
                }
            }
        }
    }
    return candidates;
}

vector<CandidatePair> Collision::findCandidates(vector<Body> bodies) {
    vector<CandidatePair> candidates;
    
    for (auto &body : bodies) {
        for (auto &geometry : body->geometry) {
            ShapeCache sc = geometry.shape->cache(body->x, body->R, geometry.x);
            int sx = floor(sc.aabb.min.x / gridSize);
            int sy = floor(sc.aabb.min.y / gridSize);
            int sz = floor(sc.aabb.min.z / gridSize);
            int ex = floor(sc.aabb.max.x / gridSize);
            int ey = floor(sc.aabb.max.y / gridSize);
            int ez = floor(sc.aabb.max.z / gridSize);
            Candidate candidate;
            candidate.body = body;
            candidate.cache = sc;
            candidate.shape = geometry.shape;
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
    }
    
    unordered_map<uint, bool> filter;
    
    for (auto &cell : grid) {
        for (int i = 0; i < cell.second.size() - 1; i++) {
            Candidate ci = cell.second[i];
            for (int j = i + 1; j < cell.second.size(); j++) {
                Candidate cj = cell.second[j];
                if (filter[ci.shape->id ^ cj.shape->id]) {
                    continue;
                }
                // i guess this is sorta risky as a false positive will
                // mean we don't check a pair
                filter[ci.shape->id ^ cj.shape->id] = true;
                if (!(ci == cj) && (ci.body->collisionMask & cj.body->collisionGroup)) {
                    candidates.push_back({ci, cj});
                }
            }
        }
    }
    
    return candidates;
}


vector<Contact> Collision::findFloorContacts(vector<Body> bodies) {
    vector<Contact> contacts;
    for (auto body : bodies) {
        if (body->shape->type == 1) {
            SphereShape *si = (SphereShape*) body->shape;
            if (body->x.y < si->r) {
                Contact contact;
                contact.pair = {body};
                ContactPoint cp;
                cp.p = body->x + vec3(0,-si->r,0);
                cp.normal = vec3(0,-1,0);
                cp.penetration = si->r - body->x.y;
                cp.u1 = vec3(1,0,0);
                cp.u2 = vec3(0,0,1);
                contact.manifold = {cp};
                contacts.push_back(contact);
            }
        } else if (body->shape->type == 2) {
            BoxShape *si = (BoxShape*) body->shape;
            vector<vec3> vertices = si->getVerticesInWorldSpace(body->x, body->R);
            vec3 normal = vec3(0,-1,0);
            Contact contact;
            contact.pair = {body};
            for (auto &vertex : vertices) {
                if (vertex.y <= 0) {
                    contact.manifold.push_back({normal, vertex, -vertex.y, vec3(1,0,0), vec3(0,0,1)});
                }
            }
            if (contact.manifold.size() > 0) {
                contacts.push_back(contact);
            }
        }
    }
    return contacts;
}



