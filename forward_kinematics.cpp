#include <armadillo>

#include "forward_kinematics.hpp"

ForwardKinematics::ForwardKinematics() {
    // assign DH parameters
    a[0] = 0;
    a[1] = 0.95;
    a[2] = 0.95;
    a[3] = 0;
    a[4] = 0;
    a[5] = 0;
    
    d[0] = 0.65;
    d[1] = 0;
    d[2] = 0;
    d[3] = 0;
    d[4] = 1.30;
    d[5] = 0;

    alpha[0] = arma::datum::pi / 2;
    alpha[1] = 0;
    alpha[2] = 0;
    alpha[3] = -arma::datum::pi / 2;
    alpha[4] = 0;
    alpha[5] = 0;

    // cache the sines and cosines of the constant parameters 
    for(int i = 0; i < CRAP_NUM_FRAMES; i++) {
        sin_alpha[i] = sin(alpha[i]);
        cos_alpha[i] = cos(alpha[i]);
    }
}

arma::vec ForwardKinematics::GetExtendedPositionVector(double q[]) {
    q[1] += arma::datum::pi / 2;
    q[3] -= arma::datum::pi / 2;
    
    for(int i = 0; i < CRAP_NUM_REVOLUTE_FRAMES; i++) {
        A[i](0, 0) = cos(q[i]);
        A[i](0, 1) = -sin(q[i]) * cos_alpha[i];
        A[i](0, 2) = sin(q[i]) * sin_alpha[i];
        A[i](0, 3) = a[i] * cos(q[i]);

        A[i](1, 0) = sin(q[i]);
        A[i](1, 1) = cos(q[i]) * cos_alpha[i];
        A[i](1, 2) = -cos(q[i]) * sin_alpha[i];
        A[i](1, 3) = a[i] * sin(q[i]);
        
        A[i](2, 0) = 0;
        A[i](2, 1) = sin_alpha[i];
        A[i](2, 2) = cos_alpha[i];
        A[i](2, 3) = d[i];

        A[i](3, 0) = 0;
        A[i](3, 1) = 0;
        A[i](3, 2) = 0;
        A[i](3, 3) = 1;
    }

    // the final joint is a prismatic joint
    // for now we assume that the arm is always in the open position (d[5] = 0)
    int final_frame = CRAP_NUM_FRAMES - 1;
    A[final_frame](0, 0) = cos(0);
    A[final_frame](0, 1) = -sin(0) * cos_alpha[final_frame];
    A[final_frame](0, 2) = sin(0) * sin_alpha[final_frame];
    A[final_frame](0, 3) = a[final_frame] * cos(0);

    A[final_frame](1, 0) = sin(0);
    A[final_frame](1, 1) = cos(0) * cos_alpha[final_frame];
    A[final_frame](1, 2) = -cos(0) * sin_alpha[final_frame];
    A[final_frame](1, 3) = a[final_frame] * sin(0);
    
    A[final_frame](2, 0) = 0;
    A[final_frame](2, 1) = sin_alpha[final_frame];
    A[final_frame](2, 2) = cos_alpha[final_frame];
    A[final_frame](2, 3) = d[final_frame];
    // * the final d parameter should eventually be an input

    A[final_frame](3, 0) = 0;
    A[final_frame](3, 1) = 0;
    A[final_frame](3, 2) = 0;
    A[final_frame](3, 3) = 1;

    arma::mat::fixed<4, 4> transform_matrix = A[0];
    for(int i = 1; i < CRAP_NUM_FRAMES; i++) {
        transform_matrix *= A[i];
    }

    arma::vec pn(4, arma::fill::zeros);
    pn(3) = 1;

    arma::vec p0 = transform_matrix * pn;

    return p0;
}
