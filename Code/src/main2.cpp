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


int main () {
    // BFGS subsystems for debugging
    LineSearch ls(0.057,0.365,0.430,0);
    ForwardKinematics fktoo;

    // set start motor positions
    LA::vecd<5> start = LA::vecd<5>(0.2);
    start[4] = 0.0;

    // set target position
    double x = 0.1, y = 0.1, z = 0.1;
    LA::vecd<3> position = {x, y, z};
    bool goalInBounds = false;
    
    std::cout << "Running BFGS Algorithm..." << std::endl;
    // instance a BFGS algorithm object
    BFGS bfgs = BFGS();
    // make it debug iterations so its progress can be monitored
    bfgs.debugIteration = true;

    // create vector to fill with final motor angles
    LA::vecd<5> end;
    // run algorithm
    bool reachedGoal = bfgs.Run(x, y, z, start, end);
    // print if completed successfully
    if (reachedGoal)
        std::cout << "Reached goal: " << reachedGoal << std::endl;

    std::cout << "---------------------" << std::endl;
    std::cout << "Caluclated Motor Angles: ";
    LA::print(end, true);

    // checks if final position is in bounds
    bool isResultInBounds = ls.InBounds(end);
    std::cout << "Result in Bounds: " << isResultInBounds << std::endl;
    
    // checks final cost & gradient
    ls.set_goal(x,y,z);
    double cost = ls.cost_function(end, 0, 0.00001);
    std::cout << "Final Cost = " << cost << std::endl;
    std::cout << "Final Grad = " << std::endl;
    LA::print(ls.cost_function_gradient(end, 0.0, 0.00001));
    
    // get final position
    LA::vecd<3> pos = fktoo.GetExtendedPositionVector(end);
    // print debug
    std::cout << "---------------------" << std::endl;
    std::cout << "Target Position: " << x << " " << y << " " << z << std::endl;
    std::cout << "FK Calculated Position Based off Angles: ";
    LA::print(pos, true);
    std::cout << "---------------------" << std::endl;
    std::cout << "Press Enter Key to Continue..." << std::endl;     
    std::cin.get();
    return 0;
}