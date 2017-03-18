//
//  HingeJoint.hpp
//  BoltzmannPhysics
//
//  Created by Drew on 3/8/17.
//
//

#ifndef HingeJoint_hpp
#define HingeJoint_hpp

#include <stdio.h>
#include "BallAndSocketJoint.hpp"

namespace bltz {
    
    class HingeJoint : public BallAndSocketJoint {
    public:
        static shared_ptr<HingeJoint> create(Body b1, vec3 r1, vec3 axis, Body b2);
        virtual void solve(float dt);
        virtual void prepare(float dt);
        vec3 a1, a2;
        void setLimits(float minTheta, float maxTheta);
    protected:
        HingeJoint(Body b1, vec3 r1, vec3 axis, Body b2);
        C2DOF rqn;
        quat q0;
        bool hasLimits;
        float minTheta, maxTheta;
        C1DOF lim1, lim2;
        void prepareLimits(float dt);
        void solveLimit(C1DOF &limit, float dt);
    };
    
    typedef shared_ptr<HingeJoint> Hinge;
}

#endif /* HingeJoint_hpp */
