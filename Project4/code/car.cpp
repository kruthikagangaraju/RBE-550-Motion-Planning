///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Kruthika Gangaraju, Jessica M Rhodes
//////////////////////////////////////

#include <iostream>
#include <fstream>

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
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>


// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void defaultCellSizes(void) override
    {
        // Set the cell sizes for the grid KPIECE uses
        cellSizes_.resize(2);
        cellSizes_[0] = 0.5;  // For x
        cellSizes_[1] = 0.5;  // For y
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        //const auto* carstate = state->as<ob::CompoundState>();
        const auto *se2 = state->as<ob::SE2StateSpace::StateType>();
        projection[0] = se2->getX();
        projection[1] = se2->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control *control, ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    double theta = q[2];
    double v = q[3];
    double omega = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    double v_dot = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1];
    qdot.resize(q.size(), 0);
    qdot[0] = omega * cos(theta);
    qdot[1] = omega * sin(theta);
    qdot[2] = omega * tan(v_dot)/0.2;
    //qdot[3] = v_dot;
}

void postODE(const ob::State* state, const oc::Control* control, const double duration, ob::State* result)
{
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
    //as<ob::CompoundState>()->
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obstacle1;
    obstacle1.x = -5.0;
    obstacle1.y = -4.0;
    obstacle1.width = 2.0;
    obstacle1.height = 3.0;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = -5.0;
    obstacle2.y = 6.0;
    obstacle2.width = 2.0;
    obstacle2.height = 3.0;
    obstacles.push_back(obstacle2);

    // Add more obstacles as needed
    // For example:
    Rectangle obstacle3;
    obstacle3.x = -3.0;
    obstacle3.y = 6.0;
    obstacle3.width = 3.0;
    obstacle3.height = 1.0;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = 1.0;
    obstacle4.y = -4.0;
    obstacle4.width = 4.0;
    obstacle4.height = 1.0;
    obstacles.push_back(obstacle4);

    Rectangle obstacle5;
    obstacle5.x = 5.0;
    obstacle5.y = -4.0;
    obstacle5.width = 1.0;
    obstacle5.height = 7.0;
    obstacles.push_back(obstacle5);

    Rectangle obstacle6;
    obstacle6.x = 3.0;
    obstacle6.y = 3.0;
    obstacle6.width = 2.0;
    obstacle6.height = 2.0;
    obstacles.push_back(obstacle6);

    Rectangle obstacle7;
    obstacle7.x = 7.0;
    obstacle7.y = -8.0;
    obstacle7.width = 1.0;
    obstacle7.height = 4.0;
    obstacles.push_back(obstacle7);

    std::ofstream output("obstacles.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " "
            << obstacle.y << " "
            << obstacle.width << " "
            << obstacle.height << std::endl;
    }
    output.close();

}

bool isSquareStateValid(const oc::SpaceInformation* si, const ob::State* state, const std::vector<Rectangle>& obstacles)
{
    const auto* se2state = state->as<ob::SE2StateSpace::StateType>();
    const double x = se2state->getX();
    const double y = se2state->getY();
    const double theta = se2state->getYaw();

    // Check if state satisfies bounds and collision constraints
    return si->satisfiesBounds(state) && isValidSquare(x, y, theta, 0.2, obstacles, 10, -10, 10, -10);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto stateSpace = std::make_shared<ob::SE2StateSpace>();
    //auto steer = std::make_shared<ob::SE2StateSpace>();
    //auto v = std::make_shared<ob::RealVectorStateSpace>(1);

    ob::RealVectorBounds sbounds(2);
    sbounds.setLow(0, -10);
    sbounds.setHigh(0, 10);
    sbounds.setLow(1, -10);
    sbounds.setHigh(1, 10);
    //sbounds.setLow(2, -M_PI);
    //sbounds.setHigh(2, M_PI);
    stateSpace->setBounds(sbounds);

    //ob::RealVectorBounds vbounds(1);
    //vbounds.setLow(-5);
    //vbounds.setHigh(5);
    //v->setBounds(vbounds);

    //stateSpace->addSubspace(steer, 1.0);
    //stateSpace->addSubspace(v, 1.0);

    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);
    ompl::base::RealVectorBounds controlBounds(2);
    
    controlBounds.setLow(0, 0.0);
    controlBounds.setHigh(0, 1.0);
    controlBounds.setLow(1, -M_PI/6);
    controlBounds.setHigh(1, M_PI/6);
    controlSpace->setBounds(controlBounds);

    auto ss = std::make_shared<oc::SimpleSetup>(controlSpace);
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, postODE));
    stateSpace->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new CarProjection(stateSpace)));
    oc::SpaceInformation* si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, obstacles](const ob::State* state)
        {
            return isSquareStateValid(si, state, obstacles);
        });

    ob::ScopedState<ob::SE2StateSpace> start(stateSpace);
    //ob::ScopedState<ob::RealVectorStateSpace> vel_s(stateSpace);
    start->setX(-7.0);
    start->setY(-8.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(stateSpace);
    //ob::ScopedState<ob::RealVectorStateSpace> vel_g(stateSpace);
    goal->setX(6.0);
    goal->setY(9.0);
    goal->setYaw(0.0);
    /*ob::ScopedState<ompl::base::CompoundStateSpace> start(stateSpace);
    start[0] = 0.5;
    start[1] = 4.5;
    start[2] = 0.0;
    start[3] = 1.5;

    // create a  goal state; use the hard way to set the elements
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(stateSpace);
    goal[0] = 6.5;
    goal[1] = 1.0;
    goal[2] = 0.0;
    goal[3] = 1.5;*/
    ss->setStartAndGoalStates(start, goal, 0.05);
    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    ob::PlannerPtr planner;

    switch (choice)
    {
    case 1:
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        break;
    case 2:
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        planner->as<oc::KPIECE1>()->setProjectionEvaluator("myProjection");
        ss->setPlanner(planner);
        break;
    case 3:
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        break;
    }

    //ss->getSpaceInformation()->setMinMaxControlDuration(1, 10);
    //ss->getSpaceInformation()->setPropagationStepSize(0.05);

    ob::PlannerStatus solved = ss->solve(5.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        oc::PathControl& path = ss->getSolutionPath();
        path.interpolate();

        path.asGeometric().printAsMatrix(std::cout);

        std::ofstream output("car_" + std::to_string(choice) + ".txt"); // Different filenames
        output << "Car " << std::endl;
        path.asGeometric().printAsMatrix(output);
        output.close();
    }
    else
        std::cout << "No Solution found" << std::endl;
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the car
    std::cout << "Starting benchmark..." << std::endl;
    auto space = ss->getStateSpace();

    space->registerDefaultProjection(std::make_shared<CarProjection>(space));

    ot::Benchmark b(*ss, "Car_Benchmark");

    auto si = ss->getSpaceInformation();

    b.addPlanner(std::make_shared<oc::KPIECE1>(si));
    b.addPlanner(std::make_shared<oc::RRT>(si));
    b.addPlanner(std::make_shared<oc::RGRRT>(si));

    ot::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 8000.0;
    req.runCount = 30;
    req.displayProgress = true;

    b.benchmark(req);

    std::string filename = "car_benchmark.log"; // Fixed torque reference
    b.saveResultsToFile(filename.c_str());
    std::cout << "Benchmark results saved to: " << filename << std::endl;
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
