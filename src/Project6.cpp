///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Jason Zhang(jz118), Shaun Lin(hl116), Lauren Peterson (lp57)
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RVO
#include "RVO.h"

// Your projection for the robot
class RobotProjection : public ompl::base::ProjectionEvaluator
{
public:
    RobotProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the robot
        return 0;
    }

    void project(const ompl::base::State * /* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the robot
    }
};

void RobotODE(const ompl::control::ODESolver::StateType & /* q */, const ompl::control::Control * /* control */,
            ompl::control::ODESolver::StateType & /* qdot */)
{
    // TODO: Fill in the ODE for the robot's dynamics
}

void makeEnv(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Fill in the vector of rectangles with your street environment.
}

ompl::control::SimpleSetupPtr createRobot(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Create and setup the robot's state space, control space, validity checker, everything you need for planning.
    return nullptr;
}

void planRobot(ompl::control::SimpleSetupPtr &/* ss */, int /* choice */)
{
    // TODO: Do some motion planning for the robot
    // choice is what planner to use.
}

void benchmarkRobot(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the robot
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeEnv(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createRobot(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RVO" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planRobot(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkRobot(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
