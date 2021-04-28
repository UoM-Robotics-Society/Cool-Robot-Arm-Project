#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <cmath>

#include "linesearch.h"

#define USE_BOUNDS true

void print_mat(arma::mat mat, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            std::cout << mat[i * m + j] << "   ";
        }
        std::cout << std::endl;
    }
}

void print_vec(arma::vec v, int size) {
    std::cout << v[0];
    for (int i = 1; i < size; i++) {
        std::cout << "   " << v[i] ;
    }
    std::cout << std::endl;
}

LineSearch::LineSearch(double r_inner, double r_outer, double h_max, double h_min) {
    radius_inner = r_inner;
    radius_outer = r_outer;
    height_max = h_max;
    height_min = h_min;

    // TODO assign max and min angles
    //min_angle = {-arma::datum::pi, 0, -2*arma::datum::pi/3, -arma::datum::pi, arma::datum::pi};
    //max_angle = {arma::datum::pi, arma::datum::pi, 2*arma::datum::pi/3, 0, arma::datum::pi};
    /*
    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        min_angle[i] = -arma::datum::pi;
        max_angle[i] = arma::datum::pi;
    } 
    */

    fk = ForwardKinematics();
}

void LineSearch::set_goal(double x, double y, double z) {
    goal_x = x;
    goal_y = y;
    goal_z = z;
}

double LineSearch::cost_function(arma::vec5 q, double d, double MU) {
    arma::vec actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords(0);
    double actual_y = actual_coords(1);
    double actual_z = actual_coords(2);

    double radius = sqrt(actual_x*actual_x + actual_y*actual_y);

    double cost = pow(goal_x - actual_x, 2) + pow(goal_y - actual_y, 2) + pow(goal_z - actual_z, 2);
    
    if (USE_BOUNDS) {
        for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
            cost += MU * (log(max_angle[i] - q[i]) + log(q[i] - min_angle[i]));
        }
        cost += MU * (log(radius_outer - radius) + log(radius - radius_inner) + log(height_max - actual_z) + log(actual_z - height_min));
    }
    
    return cost;
}

arma::vec LineSearch::cost_function_gradient(arma::vec5 q, double d, double MU) {
    arma::vec grad(5, arma::fill::zeros);
    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);
    arma::vec actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords(0);
    double actual_y = actual_coords(1);
    double actual_z = actual_coords(2);

    double radius = sqrt(actual_x*actual_x + actual_y*actual_y);

    bool chris = true;

    double dx_dq[] = {
        - 0.065*cos(q(0) + q(3) + q(1) + q(2)) + 0.065*cos(q(0) - q(3) - q(1) - q(2)) - 0.0475*sin(q(0) - q(1) - q(2)) - 0.0475*sin(q(0) + q(1) + q(2)) - 0.0475*sin(q(0) - q(1)) - 0.0475*sin(q(0) + q(1)),
        - 0.065*cos(q(0) + q(3) + q(1) + q(2)) - 0.065*cos(q(0) - q(3) - q(1) - q(2)) + 0.0475*sin(q(0) - q(1) - q(2)) - 0.0475*sin(q(0) + q(1) + q(2)) + 0.0475*sin(q(0) - q(1)) - 0.0475*sin(q(0) + q(1)),
        - 0.065*cos(q(0) + q(3) + q(1) + q(2)) - 0.065*cos(q(0) - q(3) - q(1) - q(2)) + 0.0475*sin(q(0) - q(1) - q(2)) - 0.0475*sin(q(0) + q(1) + q(2)),
        - 0.065*cos(q(0) + q(3) + q(1) + q(2)) - 0.065*cos(q(0) - q(3) - q(1) - q(2)),
        0
    };

    double dy_dq[] = {
        0.065*sin(q(0) - q(3) - q(1) - q(2)) - 0.065*sin(q(0) + q(3) + q(1) + q(2)) + 0.0475*cos(q(0) + q(1) + q(2)) + 0.0475*cos(q(0) - q(1) - q(2)) + 0.0475*cos(q(0) + q(1)) + 0.0475*cos(q(0) - q(1)),
        - 0.065*sin(q(0) - q(3) - q(1) - q(2)) - 0.065*sin(q(0) + q(3) + q(1) + q(2)) + 0.0475*cos(q(0) + q(1) + q(2)) - 0.0475*cos(q(0) - q(1) - q(2)) + 0.0475*cos(q(0) + q(1)) - 0.0475*cos(q(0) - q(1)),
        - 0.065*sin(q(0) - q(3) - q(1) - q(2)) - 0.065*sin(q(0) + q(3) + q(1) + q(2)) + 0.0475*cos(q(0) + q(1) + q(2)) - 0.0475*cos(q(0) - q(1) - q(2)),
        - 0.065*sin(q(0) - q(3) - q(1) - q(2)) - 0.065*sin(q(0) + q(3) + q(1) + q(2)),
        0
    };

    double dz_dq[] = {
        0,
        - 0.13*sin(q(3) + q(1) + q(2)) + 0.095*cos(q(1) + q(2)) + 0.095*cos(q(1)),
        - 0.13*sin(q(3) + q(1) + q(2)) + 0.095*cos(q(1) + q(2)),
        - 0.13*sin(q(3) + q(1) + q(2)),
        0
    };

    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        double a = -1/(max_angle[i] - q(i));
        double b = 1/(q(i) - min_angle[i]);
        double c = -1/(radius_outer - radius) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]));
        double d = 1/(radius - radius_inner) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]));
        double e = -1/(height_max - actual_z) * dz_dq[i];
        double f = 1/(actual_z - height_min) * dz_dq[i];
        grad(i) =
            - 2 * (goal_x - actual_x) * dx_dq[i] - 2 * (goal_y - actual_y) * dy_dq[i] - 2 * (goal_z - actual_z) * dz_dq[i]
            - MU * (a + b + c + d + e + f);
        if (!USE_BOUNDS) {
            grad(i) = - 2 * (goal_x - actual_x) * dx_dq[i] - 2 * (goal_y - actual_y) * dy_dq[i] - 2 * (goal_z - actual_z) * dz_dq[i];
        }
    }

    if(chris){
        grad(0) = -0.130*goal_x*cos(q0 - q3 - q1 - q2) + 0.0950*goal_x*sin(q0 - q1 - q2) + 0.0950*goal_x*sin(q0 - q1) - 0.130*goal_y*sin(q0 - q3 - q1 - q2) - 0.0950*goal_y*cos(q0 - q1 - q2) - 0.0950*goal_y*cos(q0 - q1) + 0.130*goal_x*cos(q0 + q3 + q1 + q2) + 0.0950*goal_x*sin(q0 + q1 + q2) + 0.0950*goal_x*sin(q0 + q1) + 0.130*goal_y*sin(q0 + q3 + q1 + q2) - 0.0950*goal_y*cos(q0 + q1 + q2) - 0.0950*goal_y*cos(q0 + q1);
        grad(1) = 0.130*goal_y*sin(q0 + q3 + q1 + q2) - 0.0950*goal_y*cos(q0 + q1 + q2) - 0.0950*goal_y*cos(q0 + q1) + 0.130*goal_x*cos(q0 + q3 + q1 + q2) + 0.0950*goal_x*sin(q0 + q1 + q2) + 0.0950*goal_x*sin(q0 + q1) + 0.012350*cos(q1 + q2) + 0.130*goal_y*sin(q0 - q3 - q1 - q2) + 0.0950*goal_y*cos(q0 - q1 - q2) + 0.0950*goal_y*cos(q0 - q1) + 0.130*goal_x*cos(q0 - q3 - q1 - q2) - 0.0950*goal_x*sin(q0 - q1 - q2) - 0.0950*goal_x*sin(q0 - q1) - 0.01690*sin(q3 + q1 + q2) + 0.012350*cos(q1) + 0.26*goal_z*sin(q3 + q1 + q2) - 0.190*goal_z*cos(q1 + q2) - 0.190*goal_z*cos(q1);
        grad(2) = 0.130*goal_y*sin(q0 + q3 + q1 + q2) - 0.0950*goal_y*cos(q0 + q1 + q2) + 0.130*goal_x*cos(q0 + q3 + q1 + q2) + 0.0950*goal_x*sin(q0 + q1 + q2) + 0.012350*cos(q1 + q2) + 0.130*goal_y*sin(q0 - q3 - q1 - q2) + 0.0950*goal_y*cos(q0 - q1 - q2) + 0.130*goal_x*cos(q0 - q3 - q1 - q2) - 0.0950*goal_x*sin(q0 - q1 - q2) - 0.01690*sin(q3 + q1 + q2) - 0.01805000000*sin(q2) - 0.02470000000*cos(q3 + q2) + 0.26*goal_z*sin(q3 + q1 + q2) - 0.190*goal_z*cos(q1 + q2);
        grad(3) = 0.130*goal_y*sin(q0 + q3 + q1 + q2) + 0.130*goal_x*cos(q0 + q3 + q1 + q2) + 0.130*goal_y*sin(q0 - q3 - q1 - q2) + 0.130*goal_x*cos(q0 - q3 - q1 - q2) - 0.01690*sin(q3 + q1 + q2) - 0.02470000000*cos(q3 + q2) - 0.02470000000*cos(q3) + 0.26*goal_z*sin(q3 + q1 + q2);
        grad(4) = 0;
    }
    return grad;
}

int LineSearch::InBoundsPos(arma::vec pos) {
    double actual_x = pos(0);
    double actual_y = pos(1);
    double actual_z = pos(2);
    double radius = sqrt(pow(actual_x,2) + pow(actual_y,2));
    if(USE_BOUNDS){
        if(radius < radius_inner){
            return 3;
        }
    }
    if (radius > radius_outer) {
        return 1;
    }
    if (actual_z < height_min) {
        return 2;
    }
    return 0;
}

int LineSearch::InBounds(arma::vec angles) {
    arma::vec actual_coords = fk.GetExtendedPositionVector(angles);
    double actual_x = actual_coords(0);
    double actual_y = actual_coords(1);
    double actual_z = actual_coords(2);
    double radius = sqrt(pow(actual_x,2) + pow(actual_y,2));
    if(USE_BOUNDS){
        for (int i = 0; i < 5; i++) {
            if (angles[i] > max_angle[i] || angles[i] < min_angle[i]) {
                return 4;
            }
        }
        if(radius < radius_inner){
            return 3;
        }
    }
    if (radius > radius_outer) {
        return 1;
    }
    if (actual_z < height_min) {
        return 2;
    }
    return 0;
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
arma::vec LineSearch::GoldenSearch(arma::vec current_pos, arma::vec direction, double mu) {
    // CONSTANTS
    arma::vec dir_norm = arma::normalise(direction);
    float tolerance = 0.0000001;
    float tau = 2 / (1 + sqrt(5));
    float dir_step = 0.01;
    arma::vec x_min = current_pos;
    arma::vec x_max = arma::vec::fixed<5>();
    arma::vec start_min = x_min;
    arma::vec start_max = x_max;
    x_max = current_pos;
    int dir_count = 0;
    arma::vec dir_inc = dir_norm * dir_step;

    //std::cout << "dir_norm = "; print_vec(dir_norm);

    while (InBounds(x_max) == 0) {
        x_max = x_max + dir_inc;
        dir_count += 1;
    }
    std::cout << InBounds(x_max) << std::endl;
    if (dir_count == 1) {
        std::cout << "wah" << std::endl;
    }
    while (InBounds(x_max) != 0) {
         x_max = x_max - dir_inc * 0.5;
    }
    std::cout << InBounds(x_max) << std::endl;
    std::cout << dir_count << std::endl;
    //   for (int i = 0; i < 5; i++) {
    //     x_max[i] = (direction[i] >= 0) ?  this->max_angle[i] : this->min_angle[i];
    //   }

    // VARIALBES
    int k = 0; // interations;
    arma::vec w_min = tau * x_min + (1 - tau) * x_max; // initialise lower tau search point
    arma::vec w_max = (1 - tau) * x_min + tau * x_max; // initialise upper tau search point

    while (arma::max(arma::abs(x_max - x_min)) > tolerance) {
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
        //std::cout << k << ": " << arma::max(arma::abs(x_max - x_min)) << std::endl;
    }
    arma::vec v = (x_min + x_max) * 0.5;
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