#include <armadillo>
#include <cmath>

#include "line_search.hpp"

LineSearch::LineSearch(double r_inner, double r_outer, double h_max, double h_min) {
    radius_inner = r_inner;
    radius_outer = r_outer;
    height_max = h_max;
    height_min = h_min;

    // TODO assign max and min angles
    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        min_angle[i] = -arma::datum::pi;
        max_angle[i] = arma::datum::pi;
    }

    fk = ForwardKinematics();
}

void LineSearch::set_goal(double x, double y, double z) {
    goal_x = x;
    goal_y = y;
    goal_z = z;
}

double LineSearch::cost_function(arma::vec5 q, double d) {
    arma::vec actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords(0);
    double actual_y = actual_coords(1);
    double actual_z = actual_coords(2);

    double radius = sqrt(actual_x*actual_x + actual_y*actual_y);

    double cost = (goal_x - actual_x) + (goal_y - actual_y) + (goal_z - actual_z);
    std::cout<<cost<<std::endl;
    
    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        cost -= CRAP_MU * (log(max_angle[i] - q[i]) + log(q[i] - min_angle[i]));
    }

    cost -= CRAP_MU * (log(radius_outer - radius) + log(radius - radius_inner) + log(height_max - actual_z) + log(actual_z - height_min));

    return cost;
}

arma::vec LineSearch::cost_function_gradient(arma::vec5 q, double d) {
    arma::vec grad(5, arma::fill::zeros);

    arma::vec actual_coords = fk.GetExtendedPositionVector(q);
    double actual_x = actual_coords(0);
    double actual_y = actual_coords(1);
    double actual_z = actual_coords(2);

    double radius = sqrt(actual_x*actual_x + actual_y*actual_y);

    double dx_dq[] = {
        - 0.65*cos(q(0) + q(3) + q(1) + q(2)) + 0.65*cos(q(0) - q(3) - q(1) - q(2)) - 0.475*sin(q(0) - q(1) - q(2)) - 0.475*sin(q(0) + q(1) + q(2)) - 0.475*sin(q(0) - q(1)) - 0.475*sin(q(0) + q(1)),
        - 0.65*cos(q(0) + q(3) + q(1) + q(2)) - 0.65*cos(q(0) - q(3) - q(1) - q(2)) + 0.475*sin(q(0) - q(1) - q(2)) - 0.475*sin(q(0) + q(1) + q(2)) + 0.475*sin(q(0) - q(1)) - 0.475*sin(q(0) + q(1)),
        - 0.65*cos(q(0) + q(3) + q(1) + q(2)) - 0.65*cos(q(0) - q(3) - q(1) - q(2)) + 0.475*sin(q(0) - q(1) - q(2)) - 0.475*sin(q(0) + q(1) + q(2)),
        - 0.65*cos(q(0) + q(3) + q(1) + q(2)) - 0.65*cos(q(0) - q(3) - q(1) - q(2)),
        0
    };

    double dy_dq[] = {
        0.65*sin(q(0) - q(3) - q(1) - q(2)) - 0.65*sin(q(0) + q(3) + q(1) + q(2)) + 0.475*cos(q(0) + q(1) + q(2)) + 0.475*cos(q(0) - q(1) - q(2)) + 0.475*cos(q(0) + q(1)) + 0.475*cos(q(0) - q(1)),
        - 0.65*sin(q(0) - q(3) - q(1) - q(2)) - 0.65*sin(q(0) + q(3) + q(1) + q(2)) + 0.475*cos(q(0) + q(1) + q(2)) - 0.475*cos(q(0) - q(1) - q(2)) + 0.475*cos(q(0) + q(1)) - 0.475*cos(q(0) - q(1)),
        - 0.65*sin(q(0) - q(3) - q(1) - q(2)) - 0.65*sin(q(0) + q(3) + q(1) + q(2)) + 0.475*cos(q(0) + q(1) + q(2)) - 0.475*cos(q(0) - q(1) - q(2)),
        - 0.65*sin(q(0) - q(3) - q(1) - q(2)) - 0.65*sin(q(0) + q(3) + q(1) + q(2)),
        0
    };

    double dz_dq[] = {
        0,
        - 1.3*sin(q(3) + q(1) + q(2)) + 0.95*cos(q(1) + q(2)) + 0.95*cos(q(1)),
        - 1.3*sin(q(3) + q(1) + q(2)) + 0.95*cos(q(1) + q(2)),
        - 1.3*sin(q(3) + q(1) + q(2)),
        0
    };

    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        grad(i) =
            - dx_dq[i] - dy_dq[i] - dz_dq[i]
            - CRAP_MU * (-1/(max_angle[i] - q(i)) + 1/(q(i) - min_angle[i]) 
                - 1/(radius_outer - radius) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]))
                + 1/(radius - radius_inner) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]))
                - 1/(height_max - actual_z) * dz_dq[i]
                + 1/(actual_z - height_min) * dz_dq[i]
                );
    }

    return grad;
}