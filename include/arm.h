#pragma once

// Standard Libraries
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <ctime>

// C.R.A.P Sub-Systems
#include "linesearch.h"
#include "forwardkinematics.h"
#include "la.h"
#include "BFGS.h"

/*
Prime class that encapsulates all accesses to the arm with proper integration of all systems.
*/
class Arm {
    private:
        LineSearch ls = LineSearch(0.057,0.365,0.430,0);
        ForwardKinematics fk = ForwardKinematics();
        BFGS bfgs = BFGS();
        LA::vecd<5> currentMotors = LA::vecd<5>();

    public:
        /*
        Calls arm to move to specified 3D co-ordinate immediately.
        @param x co-ordinate
        @param y co-ordinate
        @param z co-ordinate
        @param error code written int pointer for validation (OPTIONAL)
        @returns true if position was reached, false otherwise
        */
        bool MoveTo(double x, double y, double z);

        /*
        Retrieves current arm position.
        @param x co-ordinate pointer
        @param y co-ordinate pointer
        @param z co-ordinate pointer
        @returns true if success, false otherwise
        */
        bool GetPosition(double* x, double* y, double* z);
};