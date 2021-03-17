#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <cmath>

#include "linesearch.h"

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

    double cost = (goal_x - actual_x) + (goal_y - actual_y) + (goal_z - actual_z);
    std::cout<<cost<<std::endl;
    
    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        cost -= MU * (log(max_angle[i] - q[i]) + log(q[i] - min_angle[i]));
    }

    cost -= MU * (log(radius_outer - radius) + log(radius - radius_inner) + log(height_max - actual_z) + log(actual_z - height_min));

    return cost;
}

arma::vec LineSearch::cost_function_gradient(arma::vec5 q, double d, double MU) {
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
            - MU * (-1/(max_angle[i] - q(i)) + 1/(q(i) - min_angle[i]) 
                - 1/(radius_outer - radius) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]))
                + 1/(radius - radius_inner) * (1/radius * (actual_x*dx_dq[i] - actual_y*dy_dq[i]))
                - 1/(height_max - actual_z) * dz_dq[i]
                + 1/(actual_z - height_min) * dz_dq[i]
                );
    }

    return grad;
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
  double min_angle[] = {-arma::datum::pi, 0, -2*arma::datum::pi/3, -arma::datum::pi, arma::datum::pi};
  double max_angle[] = {arma::datum::pi, arma::datum::pi, 2*arma::datum::pi/3, 0, arma::datum::pi};
  float tolerance = 0.001;
  float tau = 2 / (1 + sqrt(5));
  arma::vec x_min = current_pos;
  arma::vec x_max = arma::vec::fixed<5>();
  for (int i = 0; i < 5; i++) {
    x_max[i] = (direction[i] >= 0) ? max_angle[i] : min_angle[i];
  }
  // VARIALBES
  int k = 0; // interations;
  arma::vec w_min = tau * x_min + (1 - tau) * x_max; // initialise lower tau search point
  arma::vec w_max = (1 - tau) * x_min + tau * x_max; // initialise upper tau search point

  while (arma::max(arma::abs(x_max - x_min)) > tolerance) {
    float j_w_min = this->cost_function(w_min,0,mu); // Fix me
    float j_w_max = this->cost_function(w_min,0,mu); // Fix me
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
    std::cout << k << ": " << arma::max(arma::abs(x_max - x_min)) << std::endl;
  }
  return (x_min + x_max) / 2;
}