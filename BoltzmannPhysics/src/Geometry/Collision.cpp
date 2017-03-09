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

set<CandidatePair> Collision::findCandidates(vector<Body> bodies, int gridSize) {
    set<CandidatePair> candidates;
    unordered_map<ull, vector<Candidate>> grid;
    
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
    
    for (auto &cell : grid) {
        for (int i = 0; i < cell.second.size() - 1; i++) {
            Candidate ci = cell.second[i];
            for (int j = i + 1; j < cell.second.size(); j++) {
                Candidate cj = cell.second[j];
                CandidatePair cp = {ci, cj};
                if (!(ci == cj) && (ci.body->collisionMask & cj.body->collisionGroup)) {
                    candidates.insert(cp);
                }
            }
        }
    }
    
    return candidates;
}
