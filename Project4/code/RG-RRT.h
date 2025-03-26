///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Jessica M Rhodes, Kruthika Gangaraju
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>
#include <limits>
#include <cmath>

namespace ompl 
{
namespace control 
{

class RGRRT : public base::Planner {
public:
    RGRRT(const SpaceInformationPtr &si);
    
    ~RGRRT() override;
    
    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    
    void clear() override;

    void setGoalBias(double goalBias)
    {
        goalBias_ = goalBias;
    }

    double getGoalBias() const
    {
        return goalBias_;
    }

    void setIntermediateStates(bool addIntermediateStates)
    {
        addIntermediateStates_ = addIntermediateStates;
    }

    bool getIntermediateStates() const
    {
        return addIntermediateStates_;
    }

    
    void getPlannerData(base::PlannerData &data) const override;

    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if (nn_ && nn_->size() != 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        nn_ = std::make_shared<NN<Motion*>>();
        setup();
    }

    void setup() override;
    
    void setReachabilityTimeStep(double dt) { reachabilityDt_ = dt; }
    double getReachabilityTimeStep() const { return reachabilityDt_; }
    
protected:
    class Motion{
    public:
        Motion(const SpaceInformation *si, const SpaceInformation* siC)
        : si_(si), siC_(siC), state(si->allocState()), control(siC->allocControl()) {}
        const base::SpaceInformation* si_;
        const SpaceInformation* siC_;
        base::State* state;
        Control* control;
        unsigned int steps{ 0 };
        Motion* parent{ nullptr };
        std::vector<base::State*> reachableStates;
    };

    void freeMemory();
    
    Motion* selectNode(Motion *sample);
    void computeReachableSet(Motion *motion);
    double reachabilityDistance(const Motion *a, const Motion *b) const;

    base::StateSamplerPtr sampler_;
    DirectedControlSamplerPtr controlSampler_;

    const SpaceInformation* siC_;
    std::shared_ptr<NearestNeighbors<Motion*>> nn_;
    
    double reachabilityDt_{0.1};
    double goalBias_{ 0.05 };
    bool addIntermediateStates_{ false };

    Motion* lastGoalMotion_{ nullptr };
};

}
}

#endif