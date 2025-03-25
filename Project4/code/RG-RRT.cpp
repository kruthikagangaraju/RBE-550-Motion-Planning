///////////////////////////////////////
// RBE550
// Project 4
// Authors: Jessica M Rhodes
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/util/Exception.h>
#include <limits>
#include <cmath>
#include <fstream>
#include <iostream>

// TODO: Implement RGRRT as described

using namespace ompl;

// Helper functions
void visualizePendulumPath(const std::vector<base::State*>& states, const std::string& filename) {
    std::ofstream file(filename);
    for (auto state : states) {
        auto angle = state->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0)->value;
        auto velocity = state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1)->values[0];
        file << angle << " " << velocity << "\n";
    }
    file.close();
    std::cout << "Pendulum path saved to " << filename << "\n";
}

void visualizeCarPath(const std::vector<base::State*>& states, const std::string& filename) {
    std::ofstream file(filename);
    for (auto state : states) {
        auto x = state->as<base::SE2StateSpace::StateType>()->getX();
        auto y = state->as<base::SE2StateSpace::StateType>()->getY();
        auto yaw = state->as<base::SE2StateSpace::StateType>()->getYaw();
        file << x << " " << y << " " << yaw << "\n";
    }
    file.close();
    std::cout << "Car path saved to " << filename << "\n";
}


RGRRT::RGRRT(const SpaceInformationPtr &si) : RRT(si) {
    setName("RG-RRT");
}

base::PlannerStatus RGRRT::solve(const base::PlannerTerminationCondition &ptc) {
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    
    // Initialize tree with start state(s)
    while (const base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        computeReachableSet(motion);
        nn_->add(motion);
    }
    
    if (nn_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    
    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = siC_->allocControl();
    base::State *xstate = si_->allocState();
    
    while (!ptc) {
        // Sample random state
        sampler_->sampleUniform(rstate);
        
        // Find nearest node with reachability consideration
        Motion *nmotion = selectNode(static_cast<Motion*>(nn_->nearest(rmotion)));
        
        // Sample random control
        controlSampler_->sample(rctrl);
        
        // Propagate for fixed duration
        unsigned int cd = controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
        cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, rstate);
        
        if (cd >= siC_->getMinControlDuration()) {
            Motion *motion = new Motion(siC_);
            si_->copyState(motion->state, rstate);
            motion->control = siC_->allocControl();
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;
            
            computeReachableSet(motion);
            nn_->add(motion);
            
            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if (solved) {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif) {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }
    
    // Handle solution
    bool solved = false;
    if (solution) {
        // Construct solution path
        std::vector<Motion*> mpath;
        while (solution != nullptr) {
            mpath.push_back(solution);
            solution = solution->parent;
        }
        
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i) {
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        }
        pdef_->addSolutionPath(path);
        solved = true;

        // Extract states for visualization
        std::vector<base::State*> states = path->getStates();
        
        // Determine system type based on state space
        if (dynamic_cast<base::CompoundStateSpace*>(si_->getStateSpace().get())) {
            // Likely pendulum system
            visualizePendulumPath(states, "pendulum_path.txt");
        } 
        else if (dynamic_cast<base::SE2StateSpace*>(si_->getStateSpace().get())) {
            // Likely car system
            visualizeCarPath(states, "car_path.txt");
        }
    }
    
    si_->freeState(xstate);
    siC_->freeControl(rctrl);
    delete rmotion;
    
    return base::PlannerStatus(solved, false);
}

void RGRRT::computeReachableSet(Motion *motion) {
    motion->reachableStates.clear();
    
    // Get control bounds
    const base::RealVectorBounds &bounds = 
        static_cast<const ompl::control::RealVectorControlSpace*>(siC_->getControlSpace().get())->getBounds();
    
    // Generate reachable states
    unsigned int dim = bounds.low.size();
    std::vector<double> cvalues(dim);
    
    // For pendulum: only 1 control dimension (torque)
    if (dim == 1) {
        for (double tau = bounds.low[0]; tau <= bounds.high[0]; 
             tau += (bounds.high[0] - bounds.low[0]) / 10.0) {
            cvalues[0] = tau;
            Control *ctrl = siC_->allocControl();
            siC_->getControlSpace()->getValueAddressAtIndex(ctrl, 0)->setValue(tau);
            
            base::State *newState = si_->allocState();
            siC_->propagate(motion->state, ctrl, 1, newState);
            motion->reachableStates.push_back(newState);
            
            siC_->freeControl(ctrl);
        }
    }
    // For car: only use first control dimension (steering) for reachability
    else if (dim >= 2) {
        for (double omega = bounds.low[0]; omega <= bounds.high[0]; 
             omega += (bounds.high[0] - bounds.low[0]) / 10.0) {
            cvalues[0] = omega;
            cvalues[1] = 0.0; // Ignore acceleration for reachability
            
            Control *ctrl = siC_->allocControl();
            for (unsigned int i = 0; i < dim; ++i)
                siC_->getControlSpace()->getValueAddressAtIndex(ctrl, i)->setValue(cvalues[i]);
            
            base::State *newState = si_->allocState();
            siC_->propagate(motion->state, ctrl, 1, newState);
            motion->reachableStates.push_back(newState);
            
            siC_->freeControl(ctrl);
        }
    }
}

RGRRT::Motion* RGRRT::selectNode(Motion *sample) {
    std::vector<Motion*> motions;
    nn_->nearestK(sample, 10, motions); // Check 10 nearest nodes
    
    Motion *selected = nullptr;
    double minDist = std::numeric_limits<double>::infinity();
    
    for (auto m : motions) {
        double d = reachabilityDistance(m, sample);
        if (d < minDist) {
            minDist = d;
            selected = m;
        }
    }
    
    return selected ? selected : static_cast<Motion*>(nn_->nearest(sample));
}

double RGRRT::reachabilityDistance(const Motion *a, const Motion *b) const {
    double minDist = si_->distance(a->state, b->state);
    
    for (const auto &reachable : a->reachableStates) {
        double d = si_->distance(reachable, b->state);
        if (d < minDist)
            minDist = d;
    }
    
    return minDist;
}

void RGRRT::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);
    
    std::vector<Motion*> motions;
    nn_->list(motions);
    
    for (auto motion : motions) {
        if (motion->parent)
            data.addEdge(motion->parent->state, motion->state);
        else
            data.addStartVertex(motion->state);
        
        // Add reachable states to planner data for visualization
        for (auto reachable : motion->reachableStates) {
            data.addEdge(motion->state, reachable);
        }
    }
}