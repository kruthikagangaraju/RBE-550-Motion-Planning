///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/base/ProjectionEvaluator.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        const auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
        projection[0] = se2->getX();
        projection[1] = se2->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control *control, ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    double theta = q[2];
    double v = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    double omega = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1];
    qdot.resize(3);
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = omega;
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obstacle1;
    obstacle1.x = -2.0;
    obstacle1.y = -2.0;
    obstacle1.width = 4.0;
    obstacle1.height = 0.5;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = -2.0;
    obstacle2.y = 1.5;
    obstacle2.width = 4.0;
    obstacle2.height = 0.5;
    obstacles.push_back(obstacle2);

}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto stateSpace = std::make_shared<ompl::base::SE2StateSpace>();
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(5);
    stateSpace->setBounds(bounds);

    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, 2);
    ompl::base::RealVectorBounds controlBounds(2);
    controlBounds.setLow(-1.0);
    controlBounds.setHigh(1.0);
    controlSpace->setBounds(controlBounds);

    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    ss->getStateSpace()->registerProjection("CarProjection", std::make_shared<CarProjection>(ss->getStateSpace().get()));

    ss->setStateValidityChecker([&](const ompl::base::State *state) {
        const auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
        return isValidSquare(se2->getX(), se2->getY(), se2->getYaw(), 0.2, obstacles);
    });

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    ompl::base::PlannerPtr planner;
    if (choice == 1) planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    else if (choice == 2) planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    else planner = std::make_shared<RG_RRT>(ss->getSpaceInformation());
    
    ss->setPlanner(planner);
    ompl::base::ScopedState<> start(ss->getStateSpace());
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    goal[0] = 4.5;
    goal[1] = 4.5;
    goal[2] = 0.0;
    
    ss->setStartAndGoalStates(start, goal);
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(5.0);
    
    if (solved) std::cout << "Found a solution!" << std::endl;
    else std::cout << "No solution found." << std::endl;
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the car
    ompl::tools::Benchmark benchmark(*ss, "Car Benchmark");
    benchmark.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<RG_RRT>(ss->getSpaceInformation()));
    ompl::tools::Benchmark::Request request(10.0, 1000.0, 5);
    benchmark.benchmark(request);
    benchmark.saveResultsToFile();
}

int main(int argc, char **argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
