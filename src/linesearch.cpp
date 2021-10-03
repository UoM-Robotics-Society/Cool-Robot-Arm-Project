#include <cmath>

#include "linesearch.h"

#define USE_BOUNDS true

LineSearch::LineSearch(double r_inner, double r_outer, double h_max, double h_min) {
    radius_inner = r_inner;
    radius_outer = r_outer;
    height_max = h_max;
    height_min = h_min;

    fk = ForwardKinematics();
}

void LineSearch::set_goal(double x, double y, double z) {
    goal_x = x;
    goal_y = y;
    goal_z = z;
}

double LineSearch::cost_function(LA::vecd<5> q, double d, double MU) {
    LA::vecd<3> actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords[0];
    double actual_y = actual_coords[1];
    double actual_z = actual_coords[2];

    double radius = sqrt(pow(actual_x,2) + pow(actual_y,2) + pow(actual_z - 0.065,2));

    double cost = pow(goal_x - actual_x, 2) + pow(goal_y - actual_y, 2) + pow(goal_z - actual_z, 2);
    
    if (USE_BOUNDS) {
        for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
            cost -= MU * (log(max_angle[i] - q[i]) + log(q[i] - min_angle[i]));
        }
        //cost += MU * (log(radius_outer - radius));
    }
    
    return cost;
}

double LineSearch::dist_to_goal(LA::vecd<5> q, double d) {
    LA::vecd<3> actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords[0];
    double actual_y = actual_coords[1];
    double actual_z = actual_coords[2];

    double cost = sqrt(pow(goal_x - actual_x, 2) + pow(goal_y - actual_y, 2) + pow(goal_z - actual_z, 2));

    return cost;
}

LA::vecd<5> LineSearch::cost_function_gradient(LA::vecd<5> q, double d, double MU) {
    LA::vecd<5> grad(0.0);
    double q0 = q[0];
    double q1 = q[1] + LA::Pi / 2;;
    double q2 = q[2];
    double q3 = q[3] - LA::Pi / 2;;
    double q4 = q[4];
    LA::vecd<3> actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords[0];
    double actual_y = actual_coords[1];
    double actual_z = actual_coords[2];
    //distance to goal
    grad[0] = -0.095*goal_y*cos(q0 + q1) + 0.13*goal_y*sin(q0 + q3 + q1 + q2) - 0.095*goal_y*cos(q0 + q1 + q2) - 0.095*goal_y*cos(q0 - q1) - 0.13*goal_y*sin(q0 - q3 - q1 - q2) - 0.095*goal_y*cos(q0 - q1 - q2) + 0.095*goal_x*sin(q0 - q1 - q2) + 0.095*goal_x*sin(q0 - q1) - 0.13*goal_x*cos(q0 - q3 - q1 - q2) + 0.095*goal_x*sin(q0 + q1) + 0.13*goal_x*cos(q0 + q3 + q1 + q2) + 0.095*goal_x*sin(q0 + q1 + q2);
    grad[1] = -0.095*goal_y*cos(q0 + q1) + 0.13*goal_y*sin(q0 + q3 + q1 + q2) + 0.095*goal_y*cos(q0 - q1) + 0.13*goal_y*sin(q0 - q3 - q1 - q2) + 0.095*goal_y*cos(q0 - q1 - q2) + 0.012350*cos(q1) + 0.01235*cos(q1 + q2) - 0.190*goal_z*cos(q1) - 0.095*goal_y*cos(q0 + q1 + q2) - 0.01690*sin(q3 + q1 + q2) + 0.26*goal_z*sin(q3 + q1 + q2) - 0.19*goal_z*cos(q1 + q2) + 0.095*goal_x*sin(q0 + q1) - 0.095*goal_x*sin(q0 - q1) + 0.095*goal_x*sin(q0 + q1 + q2) - 0.095*goal_x*sin(q0 - q1 - q2) + 0.13*goal_x*cos(q0 - q3 - q1 - q2) + 0.13*goal_x*cos(q0 + q3 + q1 + q2);
    grad[2] = 0.13*goal_y*sin(q0 + q3 + q1 + q2) + 0.13*goal_y*sin(q0 - q3 - q1 - q2) + 0.095*goal_y*cos(q0 - q1 - q2) - 0.01805*sin(q2) + 0.01235*cos(q1 + q2) - 0.095*goal_y*cos(q0 + q1 + q2) - 0.01690*sin(q3 + q1 + q2) + 0.26*goal_z*sin(q3 + q1 + q2) - 0.19*goal_z*cos(q1 + q2) - 0.02470*cos(q2 + q3) + 0.095*goal_x*sin(q0 + q1 + q2) - 0.095*goal_x*sin(q0 - q1 - q2) + 0.13*goal_x*cos(q0 - q3 - q1 - q2) + 0.13*goal_x*cos(q0 + q3 + q1 + q2);
    grad[3] = 0.13*goal_y*sin(q0 + q3 + q1 + q2) + 0.13*goal_y*sin(q0 - q3 - q1 - q2) - 0.02470*cos(q3) - 0.01690*sin(q3 + q1 + q2) + 0.26*goal_z*sin(q3 + q1 + q2) - 0.02470*cos(q2 + q3) + 0.13*goal_x*cos(q0 - q3 - q1 - q2) + 0.13*goal_x*cos(q0 + q3 + q1 + q2);
    grad[4] = 0;
    //motor angle limits
    if(USE_BOUNDS){
        for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
            double a = -1/(max_angle[i] - q[i]);
            double b = 1/(q[i] - min_angle[i]);
            grad[i] = grad[i] - MU*(a + b);
        }
        //grad(2) = grad(2) + MU*((-0.009025000000*sin(q2) - 0.01235000000*cos(q3 + q2))/(sqrt(-0.02470000000*sin(q3) + 0.01805000000*cos(q2) + 0.03495000000 - 0.02470000000*sin(q3 + q2))*(-1.*radius_outer + sqrt(0.03495 - 0.0247*sin(q3) + 0.01805*cos(q2) - 0.02470000000*sin(q3 + q2)))));
        //grad(3) = grad(3) + MU*(-0.01235000000*(cos(q3 + q2) + cos(q3))/((-radius_outer + sqrt(0.03495 - 0.0247*sin(q3) + 0.01805*cos(q2) - 0.02470000000*sin(q3 + q2)))*sqrt(-0.02470000000*sin(q3) + 0.01805000000*cos(q2) + 0.03495000000 - 0.02470000000*sin(q3 + q2))));
    }
    return grad;
}

bool LineSearch::InBoundsPos(LA::vecd<3> pos) {
    double actual_x = pos[0];
    double actual_y = pos[1];
    double actual_z = pos[2];
    double radius = sqrt(pow(actual_x,2) + pow(actual_y,2) + pow(actual_z - 0.065,2));
    double radius2 = sqrt(pow(actual_x,2) + pow(actual_y,2));
    if (radius > radius_outer - 0.05) {
        // std::cout << "ERROR: arm outside maximum radius" << std::endl;
        return false;
    } else if (actual_z < height_min) {
        // std::cout << "ERROR: arm outside maximum height" << std::endl;
        return false;
    } else if (radius2 < radius_inner && actual_z < 0.2){
        // std::cout << "ERROR: arm inside minimum radius or height" << std::endl;
        return false;
    }
    return true;
}

bool LineSearch::InBounds(LA::vecd<5> angles) {
    LA::vecd<3> actual_coords = fk.GetExtendedPositionVector(angles);
    double actual_x = actual_coords[0];
    double actual_y = actual_coords[1];
    double actual_z = actual_coords[2];
    double radius = sqrt(pow(actual_x,2) + pow(actual_y,2) + pow(actual_z - 0.065,2));
    if(USE_BOUNDS){
        for (int i = 0; i < 5; i++) {
            if (angles[i] > max_angle[i] || angles[i] < min_angle[i]) {
                // std::cout << "ERROR: angle beyound bounds of motor" << i << std::endl;
                return false;
            }
        }
    }
    if (radius > radius_outer) {
        // std::cout << "ERROR: arm outside maximum radius" << std::endl;
        return false;
    }
    if (actual_z < height_min) {
        // std::cout << "ERROR: arm inside minimum radius" << std::endl;
        return false;
    }
    return true;
}

LA::vecd<5> LineSearch::GetAngleBounds(LA::vecd<5> current_pos, LA::vecd<5> direction) {
    // CONSTANTS
    const double accuracy = 0.00001;
    LA::vecd<5> dir_norm = LA::normalise(direction);
    LA::vecd<5> barrier = current_pos;
    LA::vecd<5> new_barrier = current_pos;
    double step = 1.0;
    while(step > accuracy) {
        new_barrier = barrier + step * dir_norm;
        if (InBounds(new_barrier)) {
            barrier = new_barrier;
            step *= 2;
        } else {
            step *= 0.5;
        }
    }
    return barrier;
}


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
LA::vecd<5> LineSearch::GoldenSearch(LA::vecd<5> current_pos, LA::vecd<5> direction, double mu) {
    // CONSTANT
    LA::vecd<5> x_min = current_pos;
    LA::vecd<5> x_max = LA::vecd<5>(0.0);
    LA::vecd<5> start_min = x_min;
    LA::vecd<5> start_max = x_max;
    
    double tolerance = 0.000001;
    double tau = 2 / (1 + sqrt(5));
    
    x_max = GetAngleBounds(current_pos, direction);

    // VARIALBES
    int k = 0; // interations;
    LA::vecd<5> w_min = tau * x_min + (1 - tau) * x_max; // initialise lower tau search point
    LA::vecd<5> w_max = (1 - tau) * x_min + tau * x_max; // initialise upper tau search point
    while (LA::max(LA::abs(x_max - x_min)) > tolerance) {
        float j_w_min = this->cost_function(w_min,0,mu);
        float j_w_max = this->cost_function(w_max,0,mu);
        if (j_w_min < j_w_max) {
            x_max = w_max;
            w_max = w_min;
            w_min = tau * x_min + (1 - tau) * x_max;
        } else {
            x_min = w_min;
            w_min = w_max;
            w_max = (1 - tau) * x_min + tau * x_max;
        }
        k += 1;
        //std::cout << k << ": " << LA::max(LA::abs(x_max - x_min)) << std::endl;
    }
    LA::vecd<5> v = (x_min + x_max) * 0.5;
    if (false) {
        std::cout << "start_min: " << this->cost_function(start_min, 0, 0.1) << std::endl; 
        std::cout << "final_res: " << this->cost_function(v, 0, 0.1) << std::endl;
        std::cout << "start_max: " << this->cost_function(start_max, 0, 0.1) << std::endl; 
    }
    if (false) {
        std::cout << "x_min: " << this->cost_function(x_min, 0, 0.1) << std::endl; 
        std::cout << "x_max: " << this->cost_function(x_max, 0, 0.1) << std::endl; 
    }
    return v;
}