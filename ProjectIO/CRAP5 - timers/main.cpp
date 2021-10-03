#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include "la.h"
#include "linesearch.h"
#include "forwardkinematics.h"
#include "BFGS.h"

#define GOAL_AXIS_MAX 0.3
#define GOAL_AXIS_MIN 0.1
#define DEBUG_START_INFO true

int main () {
    LineSearch ls(0.057,0.365,0.430,0);
    ForwardKinematics fktoo;
    LA::vecd<5> yeet = LA::vecd<5>(0.0);
    LA::vecd<5> start = LA::vecd<5>(0.2);
    start[4] = 0.0;

    double x, y, z = 0;
    srand (static_cast <unsigned> (time(0)));
    
    LA::vecd<3> position;
    bool goalInBounds = false;
    while (!goalInBounds) {
        x = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            x = -x;
        y = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            y = -y;
        z = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            z = -z;
        ls.set_goal(x, y, z);
        position = { x, y, z };
        goalInBounds = ls.InBoundsPos(position);
    }
    
    
    if (DEBUG_START_INFO) {
        std::cout << "---------------------" << std::endl;
        std::cout << "Goal: " << x << ", " << y << ", " << z << std::endl;
        std::cout << "---------------------" << std::endl;
        std::cout << "Starting motor angles: ";
        print(start);
        std::cout << "Starting Coordinates: ";
        print(fktoo.GetExtendedPositionVector(start),3);
        std::cout << "Start in-bounds test: " << ls.InBounds(start) << std::endl;
        std::cout << "---------------------" << std::endl;
    }
    
    std::cout << "Running BFGS Algorithm..." << std::endl;
    BFGS bfgs = BFGS();
    bfgs.debugIteration = true;
    LA::vecd<5> end;
    bool reachedGoal = bfgs.Run(x, y, z, start, end);
    if (reachedGoal)
        std::cout << "Reached goal: " << reachedGoal << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "Caluclated Motor Angles: ";
    std::cout << end[0] << " " << end[1] << " " << end[2] << " " << end[3] << " " << end[4] << std::endl;
    LA::vecd<5> vmat = end;

    bool isResultInBounds = ls.InBounds(end);
    std::cout << "Result in Bounds: " << isResultInBounds << std::endl;
    
    ls.set_goal(x,y,z);
    double cost = ls.cost_function(vmat, 0, 0.00001);
    std::cout << "Final Cost = " << cost << std::endl;

    std::cout << "Final Grad = " << std::endl;
    LA::print(ls.cost_function_gradient(vmat, 0.0, 0.00001));
    
    LA::vecd<3> pos = fktoo.GetExtendedPositionVector(vmat);
    std::cout << "---------------------" << std::endl;
    std::cout << "Target Position: " << x << " " << y << " " << z << std::endl;
    std::cout << "FK Calculated Position Based off Angles: ";
    std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "Press Enter Key to Continue..." << std::endl;     
    std::cin.get();
    return 0;
}