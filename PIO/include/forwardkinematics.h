#pragma once
#include "la.h"

#define CRAP_NUM_FRAMES 6
#define CRAP_NUM_REVOLUTE_FRAMES 5
// ! these constant names need to be fixed, esp. because
// ! of that last prismatic joint

/*
Class to calculate the final position of the end affector based on the
configuration of the robot.
*/
class ForwardKinematics {
   private:
    double a[CRAP_NUM_FRAMES];
    double d[CRAP_NUM_FRAMES];
    double alpha[CRAP_NUM_FRAMES];

    double cos_alpha[CRAP_NUM_FRAMES];
    double sin_alpha[CRAP_NUM_FRAMES];

    LA::matd<4, 4> A[CRAP_NUM_FRAMES];

   public:
    /*
    Default constructor, fills attributes with DH parameters.
    */
    ForwardKinematics();

    /*
    Calculate the position of the end affector for a certain robot
    configuration.
    @param array of the angles of the robot's five joints.
    @returns 3-vector of the x, y and z coordinates of the end affector as
    measured from the ground frame.
    */
    LA::vecd<3> GetExtendedPositionVector(LA::vecd<5> q);
};