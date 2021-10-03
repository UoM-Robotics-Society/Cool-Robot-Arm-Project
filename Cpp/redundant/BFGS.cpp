#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <iostream>
#include <cmath>
#include "linesearch.h"
#include "forward_kinematics.h"


arma::dmat BFGS (double x, double y, double z, arma::vec X){

    LineSearch arm(57,365,430,0);
    arma::dmat S(1,5,arma::fill::zeros), P(1,5,arma::fill::zeros), Y(1,5,arma::fill::zeros), Bi(5,5,arma::fill::eye), Xp (1,5,arma::fill::zeros);
    
    double A;
    arm.set_goal(x,y,z);
    double mu = 1000;

    while (mu>0.000001){

       for (int i=0;i<10;i++){
            P= -(Bi)*(arm.cost_function(X,0,mu));
    
            Xp = arm.GoldenSearch(X, P, mu);


            Y = arm.cost_function_gradient(Xp, 0, mu) - arm.cost_function_gradient(X,0, mu) ;

            
            Bi += (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));

            X=Xp;
        }
        mu=mu* 0.9;  
    }
    return X;
}


int main (){
    ForwardKinematics fktoo;
    arma::vec yeet;
    yeet << 0 << arma::datum::pi/2 << 0 << -arma::datum::pi/2 << 0 << arma::endr;
    std::cout << fktoo.GetExtendedPositionVector(yeet) << std::endl;
    printf("hello\n");
    //BFGS(yeet);
    return 0;
}