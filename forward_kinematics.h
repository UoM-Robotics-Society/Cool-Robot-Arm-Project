#pragma once
#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>

#define CRAP_NUM_FRAMES 6
#define CRAP_NUM_REVOLUTE_FRAMES 5
// ! these constant names need to be fixed, esp. because
// ! of that last prismatic joint

/*
Class to calculate the final position of the end affector based on the
configuration of the robot.
*/
class ForwardKinematics{
   private:
    double a[CRAP_NUM_FRAMES];
    double d[CRAP_NUM_FRAMES];
    double alpha[CRAP_NUM_FRAMES];

    double cos_alpha[CRAP_NUM_FRAMES];
    double sin_alpha[CRAP_NUM_FRAMES];

    arma::mat::fixed<4, 4> A[CRAP_NUM_FRAMES];
   public:
    ForwardKinematics();

    /*
    Calculate the position of the end affector for a certain robot configuration.
    @param array of the angles of the robot's five joints.
    @returns 4-vector of the x, y and z coordinates of the end affector as measured from the ground frame.
    The fourth element of the vector is always 1. This is a by-product of the forward kinematics equation.
    */
    arma::vec GetExtendedPositionVector(arma::vec5 q);
};