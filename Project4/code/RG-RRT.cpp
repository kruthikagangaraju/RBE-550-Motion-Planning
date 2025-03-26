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

        // Helper functions
        /*void visualizePendulumPath(const std::vector<base::State*>& states, const std::string& filename) {
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
                RGRRT::RGRRT(const SpaceInformationPtr& si) : RRT(si) {
            setName("RG-RRT");
        }*/
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
                    //if (motion->state)
                      //  si_->freeState(motion->state);
                    //if (motion->control)
                      //  siC_->freeControl(motion->control);
                    delete motion;
                }
            }
        }


        base::PlannerStatus RGRRT::solve(const base::PlannerTerminationCondition& ptc) {
            checkValidity();
            auto* goal = pdef_->getGoal().get();
            auto* goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

            // Initialize tree with start state(s)
            while (const base::State* st = pis_.nextStart()) {
                auto* motion = new Motion(si_.get(), siC_);
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

            auto* rmotion = new Motion(si_.get(), siC_);
            base::State* rstate = rmotion->state;
            

            while (!ptc) {
                // Sample random state
                sampler_->sampleUniform(rstate);

                // Find nearest node with reachability consideration
                Motion* nmotion = selectNode(static_cast<Motion*>(nn_->nearest(rmotion)));

                // Sample random control
                // controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, state);

                // Propagate for fixed duration
                Control* rctrl = siC_->allocControl();
                base::State* xstate = si_->allocState();
                unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->xstate, xstate);
                // cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, rstate);

                if (cd >= siC_->getMinControlDuration()) {
                    auto* motion = new Motion(si_.get(), siC_);
                    si_->copyState(motion->state, rstate);
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = cd;
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
                si_->freeState(state);
                siC_->freeControl(ctrl);
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
                    siC_->getControlSpace()->getValueAddressAtIndex(ctrl, 0)->setValue(tau);

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
                        siC_->getControlSpace()->getValueAddressAtIndex(ctrl, i)->setValue(cvalues[i]);

                    base::State* newState = si_->allocState();
                    siC_->propagate(motion->state, ctrl, 1, newState);
                    motion->reachableStates.push_back(newState);

                    siC_->freeControl(ctrl);
                }
            }
        }

        int RGRRT::selectNode(Motion* sample) {
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
        }

        double RGRRT::reachabilityDistance(const Motion* a, const Motion* b) const 
        {
            double minDist = si_->distance(a->state, b->state);

            for (const auto& reachable : a->reachableStates) {
                double d = si_->distance(reachable->state, b->state);
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