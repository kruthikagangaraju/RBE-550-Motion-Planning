///////////////////////////////////////
// RBE550
// Project 4
// Authors: Jessica M Rhodes, Kruthika Gangaraju
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/util/Exception.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <limits>
#include <cmath>
#include <fstream>
#include <iostream>

// TODO: Implement RGRRT as described

namespace ompl
{
    namespace control
    {

        RGRRT::RGRRT(const SpaceInformationPtr& si) : base::Planner(si, "RGRRT"), siC_(si.get())
        {
            specs_.approximateSolutions = true;
            //si_ = si.get();
            //siC_ = si.get();

            Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
            Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates, "0,1");
        }

        RGRRT::~RGRRT()
        {
            freeMemory();
        }

        void RGRRT::setup()
        {
            base::Planner::setup();
            if (!nn_)
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
            nn_->setDistanceFunction([this](const Motion* a, const Motion* b) { return reachabilityDistance(a, b); });
        }

        void RGRRT::clear()
        {
            Planner::clear();
            sampler_.reset();
            controlSampler_.reset();
            freeMemory();
            if (nn_)
                nn_->clear();
            lastGoalMotion_ = nullptr;
        }

        void RGRRT::freeMemory()
        {
            if (nn_)
            {
                std::vector<Motion*> motions;
                nn_->list(motions);
                for (auto& motion : motions)
                {
                    if (motion->state)
                        si_->freeState(motion->state);
                    if (motion->control)
                        siC_->freeControl(motion->control);
                    delete motion;
                }
            }
        }


        base::PlannerStatus RGRRT::solve(const base::PlannerTerminationCondition& ptc) {
            checkValidity();
            auto* goal = pdef_->getGoal().get();
            auto* goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

            if (!goal_s) {
                OMPL_ERROR("Goal is undefined");
                return base::PlannerStatus::INVALID_START;
            }

            // Initialize tree with start state(s)
            while (const base::State* st = pis_.nextStart()) {
                auto* motion = new Motion(siC_);
                si_->copyState(motion->state, st);
                siC_->nullControl(motion->control);
                computeReachableSet(motion);
                nn_->add(motion);
            }

            if (nn_->size() == 0) {
                OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                return base::PlannerStatus::INVALID_START;
            }

            if (!sampler_)
                sampler_ = si_->allocStateSampler();
            if (!controlSampler_)
                controlSampler_ = siC_->allocDirectedControlSampler();

            OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

            Motion* solution = nullptr;
            Motion* approxsol = nullptr;
            double approxdif = std::numeric_limits<double>::infinity();

            auto* rmotion = new Motion(siC_);
            base::State* rstate = rmotion->state;
            //Control* rctrl = siC_->allocControl();
            //base::State* xstate = si_->allocState();
            

            while (!ptc) {
                // Sample random state
                //sampler_->sampleUniform(rstate);

                // Find nearest node with reachability consideration
                int id = -1;
                //Motion* nmotion = selectNode(static_cast<Motion*>(nn_->nearest(rmotion)));
                Motion* nmotion = nullptr;

                if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                    goal_s->sampleGoal(rstate);
                else
                    sampler_->sampleUniform(rstate);

                // Sample random control
                // controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, state);

                // Propagate for fixed duration
                nmotion = nn_->nearest(rmotion);
                id = selectNode(nmotion, rmotion);
                
                //unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, xstate);
                // cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, rstate);

                if (id!=-1) 
                {
                    auto* motion = new Motion(siC_);
                    si_->copyState(motion->state, nmotion->reachable[id]->state);
                    //motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, nmotion->reachable[id]->control);
                    motion->steps = nmotion->reachable[id]->steps;
                    motion->parent = nmotion;

                    computeReachableSet(motion);
                    nn_->add(motion);

                    double dist = 0.0;
                    bool solved = goal->isSatisfied(motion->state, &dist);
                    if (solved) 
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif) 
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }
                else
                {
                    // Could not find a reachable motion that brings us closer
                    // Try to sample a control to move towards the random state
                    Control* rctrl = siC_->allocControl();
                    base::State* xstate = si_->allocState();
                    unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, xstate);

                    if (cd >= siC_->getMinControlDuration())
                    {
                        // Create a new motion
                        auto* motion = new Motion(siC_);
                        si_->copyState(motion->state, xstate);
                        siC_->copyControl(motion->control, rctrl);
                        motion->steps = cd;
                        motion->parent = nmotion;

                        // Compute reachable set for the new motion
                        computeReachableSet(motion);

                        nn_->add(motion);

                        // Check for solution
                        double dist = 0.0;
                        bool solv = goal->isSatisfied(motion->state, &dist);
                        if (solv)
                        {
                            approxdif = dist;
                            solution = motion;
                            break;
                        }
                        if (dist < approxdif)
                        {
                            approxdif = dist;
                            approxsol = motion;
                        }
                    }

                    si_->freeState(xstate);
                    siC_->freeControl(rctrl);
                }
                
            }

            // Handle solution
            bool solved = false;
            bool approximate = false;
            if (solution == nullptr)
            {
                solution = approxsol;
                approximate = true;
            }
            if (solution!= nullptr) {
                // Construct solution path
                lastGoalMotion_ = solution;
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
                //std::vector<base::State*> states = path->getStates();

                // Determine system type based on state space
                /*if (dynamic_cast<base::CompoundStateSpace*>(si_->getStateSpace().get())) {
                    // Likely pendulum system
                    visualizePendulumPath(states, "pendulum_path.txt");
                }
                else if (dynamic_cast<base::SE2StateSpace*>(si_->getStateSpace().get())) {
                    // Likely car system
                    visualizeCarPath(states, "car_path.txt");
                }*/
            }

            //si_->freeState(xstate);
            //siC_->freeControl(rctrl);
            delete rmotion;

            OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

            return base::PlannerStatus(solved, approximate);
        }

        void RGRRT::computeReachableSet(Motion* motion) 
        {
            motion->reachableStates.clear();

            // Get control bounds
            const base::RealVectorBounds& bounds =
                static_cast<const ompl::control::RealVectorControlSpace*>(siC_->getControlSpace().get())->getBounds();

            // Generate reachable states
            unsigned int dim = bounds.low.size();
            std::vector<double> cvalues(dim);

            // For pendulum: only 1 control dimension (torque)
            if (dim == 1) {
                for (double tau = bounds.low[0]; tau <= bounds.high[0];
                    tau += (bounds.high[0] - bounds.low[0]) / 10.0) {
                    cvalues[0] = tau;
                    Control* ctrl = siC_->allocControl();
                    *siC_->getControlSpace()->getValueAddressAtIndex(ctrl, 0) = tau;

                    base::State* newState = si_->allocState();
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

                    Control* ctrl = siC_->allocControl();
                    for (unsigned int i = 0; i < dim; ++i)
                        *siC_->getControlSpace()->getValueAddressAtIndex(ctrl, i) = cvalues[i];

                    base::State* newState = si_->allocState();
                    siC_->propagate(motion->state, ctrl, 1, newState);
                    motion->reachableStates.push_back(newState);

                    siC_->freeControl(ctrl);
                }
            }
            /*const std::vector<double>& low_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().low;
            const std::vector<double>& high_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().high;
            for (int i = 0; i < this->RSIZE; ++i)
            {
                Motion* motion = new Motion(siC_);
                siC_->copyControl(motion->control, m->control);
                double*& controls = motion->control->as<RealVectorControlSpace::ControlType>()->values;
                controls[0] = low_bound[0] + control_offset[0] * (i + 1);
                for (int j = 1; j < low_bound.size(); ++j)
                    controls[j] = high_bound[j] / 2;

                motion->steps = siC_->propagateWhileValid(m->state, motion->control, siC_->getMinControlDuration(), motion->state);

                if (motion->steps != 0)
                    m->reachable.push_back(motion);
            }*/
        }

        /*Motion* RGRRT::selectNode(Motion* sample) {
            std::vector<Motion*> motions;
            nn_->nearestK(sample, 10, motions); // Check 10 nearest nodes

            Motion* selected = nullptr;
            double minDist = std::numeric_limits<double>::infinity();

            for (auto m : motions) {
                double d = reachabilityDistance(m, sample);
                if (d < minDist) {
                    minDist = d;
                    selected = m;
                }
            }

            return selected ? selected : static_cast<Motion*>(nn_->nearest(sample));
        }*/

        int RGRRT::selectNode(Motion* qnear, Motion* qrand)
        {
            const double nearD = si_->distance(qnear->state, qrand->state);
            double minD = nearD;
            const auto& reachable = qnear->reachable;
            int id = -1;
            for (int i = 0; i < reachable.size(); ++i)
            {
                double newD = si_->distance(reachable[i]->state, qrand->state);
                if (newD < minD)
                {
                    minD = newD;
                    id = i;
                }
            }
            return id;
        }

        double RGRRT::reachabilityDistance(const Motion* a, const Motion* b) const 
        {
            double minDist = si_->distance(a->state, b->state);

            for (const auto& reachable : a->reachableStates) {
                double d = si_->distance(reachable, b->state);
                if (d < minDist)
                    minDist = d;
            }

            return minDist;
        }

        void RGRRT::getPlannerData(base::PlannerData& data) const {
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
    }
}