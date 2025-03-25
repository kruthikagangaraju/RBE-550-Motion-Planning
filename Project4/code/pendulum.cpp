///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = s->values[0];
        projection[1] = s->values[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control, ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    double g = 9.81;
    double tau = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double theta = q[0];
    double omega = q[1];
    qdot.resize(2, 0.0);
    qdot[0] = omega;
    qdot[1] = -g * cos(theta) + tau;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds sbounds(2);
    sbounds.setLow(0, -M_PI);
    sbounds.setHigh(0, M_PI);
    sbounds.setLow(1, -10.0);
    sbounds.setHigh(1, 10.0);
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(sbounds);

    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 1);
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    controlSpace->setBounds(cbounds);

    auto ss = std::make_shared<oc::SimpleSetup>(controlSpace);
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si](const ob::State *state) 
    { 
        return isStateValid(si, state); 
    });
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2;
    start[1] = 0;

    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = +M_PI / 2;
    goal[1] = 0;

    ss->setStartAndGoalStates(start, goal, 0.05);

    if (choice == 1)
        ss->setPlanner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
    else if (choice == 2)
        ss->setPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
    else if (choice == 3)
        ss->setPlanner(std::make_shared<RGRRT>(ss->getSpaceInformation())); // Corrected planner

    ob::PlannerStatus solved = ss->solve(5.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        og::PathGeometric &path = ss->getSolutionPath();
        path.interpolate(50);

        path.printAsMatrix(std::cout);

        std::ofstream output("pendulum_" + std::to_string(choice) + ".txt"); // Different filenames
        output << "Pendulum " << std::endl;
        path.printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No Solution found" << std::endl;
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss, double torque)
{
    // TODO: Do some benchmarking for the pendulum
    std::cout << "Starting benchmark..." << std::endl;
    auto space = ss->getStateSpace();

    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space));

    ot::Benchmark b(*ss, "Pendulum_Benchmark");

    auto si = ss->getSpaceInformation();

    b.addPlanner(std::make_shared<oc::KPIECE1>(si));
    b.addPlanner(std::make_shared<oc::RRT>(si));
    b.addPlanner(std::make_shared<RGRRT>(si));

    ot::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 8000.0;
    req.runCount = 30;
    req.displayProgress = true;

    b.benchmark(req);

    std::string filename = "pendulum_benchmark_torque" + std::to_string(torque) + ".log"; // Fixed torque reference
    b.saveResultsToFile(filename.c_str());
    std::cout << "Benchmark results saved to: " << filename << std::endl;
}

int main(int argc, char **argv)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss, torque);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
