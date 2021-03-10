#include <armadillo>
#include <iostream>
#include <cmath>

const double PI = 3.141592653589793238463;

// https://en.wikipedia.org/wiki/Golden-section_search

// tolerance == numerical tolerance threshold
// tau == golden ration number for sectioni

// x_min = where the moters are rn
// x_max = where the moters are if they go out of bounds
// w_min = lower tau search point
// w_max = upper tau search point

// Test cost function that takes 5d vector as weights
double Cost(arma::vec::fixed<5> w) {
  return w[0] * w[0] + 1 - w[1] + w[2] * 0 + w[3] * 0 + w[4] * 0;
}

// Golden Search function
// -x_min = current angles of motors as vector (x-min)
// -x_max = maximum angle of motors as vector in direction of search
// returns step as float value
arma::vec GoldenSearch(arma::vec current_pos, arma::vec direction) {
  // CONSTANTS
  float tolerance = 0.001;
  float tau = 2 / (1 + sqrt(5));
  arma::vec x_min = current_pos;
  arma::vec x_max = arma::vec::fixed<5>();
  for (int i = 0; i < 5; i++) {
    x_max[i] = (direction[i] >= 0) ? PI : -PI;
  }
  // VARIALBES
  int k = 0; // interations;
  arma::vec w_min = tau * x_min + (1 - tau) * x_max; // initialise lower tau search point
  arma::vec w_max = (1 - tau) * x_min + tau * x_max; // initialise upper tau search point

  while (arma::max(arma::abs(x_max - x_min)) > tolerance) {
    float j_w_min = Cost(w_min);;
    float j_w_max = Cost(w_max);
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

int test() {
arma::vec::fixed<5> start = arma::vec(5, arma::fill::zeros);
arma::vec::fixed<5> max = arma::vec(5, arma::fill::ones);
arma::vec res = GoldenSearch(start, max);
std:: cout << "Result:"<< std::endl << res << std::endl;
return 0;
}
