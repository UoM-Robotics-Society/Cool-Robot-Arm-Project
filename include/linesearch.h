#pragma once
#include "forward_kinematics.h"

class LineSearch {
    private:
    double goal_x, goal_y, goal_z;
    
    double radius_inner, radius_outer;
    double height_max, height_min;

    
    ForwardKinematics fk;

    public:
    double min_angle[CRAP_NUM_REVOLUTE_FRAMES] = {-LA::PI, -LA::PI/2, -2*LA::PI/3,   -LA::PI/2,   -LA::PI};
    double max_angle[CRAP_NUM_REVOLUTE_FRAMES] = {LA::PI,  LA::PI/2,  2*LA::PI/3,    LA::PI/2,    LA::PI};

    /*
    Initialize line search.
    TODO improve documentation here
    @param the inner radius. The arm cannot get closer to the base than this.
    @param the outer radius. The arm can't stretch further than this.
    @param the highest the arm can go.
    @param the lowest the arm can go.
    */
    LineSearch(double r_inner, double r_outer, double h_max, double h_min);

    /*
    Set the goal coordinates for the arm's motion.
    @param the x coordinate of the goal.
    @param the y coordinate of the goal.
    @param the z coordinate of the goal.
    */
    void set_goal(double x, double y, double z);
    
    /*
    Calculate the value of the cost function.
    @param array of the angles of the 5 revolute joints.
    @param the position of the final prismatic joint.
    @returns the value of the cost function.
    */
    double cost_function(LA::vecd<5> q, double d, double MU);
    double dist_to_goal(LA::vecd<5> q, double d);
    /*
    Calculate the gradient of the cost function with respect
    to the angles of the five revolute joints.
    @param array of the angles of the 5 revolute joints.
    @param the position of the final prismatic joint.
    @returns the gradient of the cost function as a vector.
    */
    LA::vecd<5> cost_function_gradient(LA::vecd<5> q, double d, double MU);

    /*
    Check if 3D position is in bounds
    @param array of the 3D co-ordinates.
    @returns true if in bounds
    */
    int InBoundsPos(LA::vecd<3> pos);
    int InBoundsPos(double x, double y, double z);
    /*
    Check if angle positions are in bounds
    @param array of the angles of the 5 revolute joints.
    @returns true if in bounds
    */
    int InBounds(LA::vecd<5> angles);

// https://en.wikipedia.org/wiki/Golden-section_search

// tolerance == numerical tolerance threshold
// tau == golden ration number for sectioni

// x_min = where the moters are rn
// x_max = where the moters are if they go out of bounds
// w_min = lower tau search point
// w_max = upper tau search point

// Golden Search function
// @param current_pos - current angles of motors as vector
// @param direction - direction of search
// @return step as float value

    LA::vecd<5> GoldenSearch(LA::vecd<5> current_pos, LA::vecd<5> direction, double mu);
};