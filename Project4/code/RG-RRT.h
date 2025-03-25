///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Jessica M Rhodes
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl {
namespace control {

class RGRRT : public ompl::control::RRT {
public:
    RGRRT(const SpaceInformationPtr &si);
    
    virtual ~RGRRT() = default;
    
    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    
    virtual void getPlannerData(base::PlannerData &data) const override;
    
    void setReachabilityTimeStep(double dt) { reachabilityDt_ = dt; }
    double getReachabilityTimeStep() const { return reachabilityDt_; }
    
protected:
    class Motion : public RRT::Motion {
    public:
        Motion(const SpaceInformation *si) : RRT::Motion(si) {}
        
        std::vector<base::State*> reachableStates;
    };
    
    Motion* selectNode(Motion *sample);
    void computeReachableSet(Motion *motion);
    double reachabilityDistance(const Motion *a, const Motion *b) const;
    
    double reachabilityDt_{0.1};
};

}
}

#endif