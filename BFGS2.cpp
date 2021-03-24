#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <iostream>
#include <cmath>
#include "linesearch.h"
#include "forward_kinematics.h"

arma::vec BFGS (double x, double y, double z, arma::vec X){

    LineSearch arm(57,365,430,0);
    arma::dmat Bi(5,5,arma::fill::eye);
    arma::vec Xp(5, arma::fill::zeros), P(5,arma::fill::zeros), S(5,arma::fill::zeros), Y(5,arma::fill::zeros);
    double A;
    arm.set_goal(x,y,z);
    double mu = 1000;

    while (mu>0.000001){

       for (int i=0;i<10;i++){
            P= -(Bi)*(arm.cost_function_gradient(X,0,mu));
    
            Xp = arm.GoldenSearch(X, P, mu);
            S = Xp - X;

            arma::vec cost1 = arm.cost_function_gradient(Xp, 0, mu);
            arma::vec cost2 = arm.cost_function_gradient(X,0, mu);      
            Y = cost1 - cost2;

            // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm

            // aa = St.Y
            // ab = Yt.Bi
            // ac = ab.Y
            // ad = S.St
            // ae = aa + ac
            // a = ae.ad
            // a = (St.Y + Yt.Bi.Y).(S.St)

            // ba = Bi.Y
            // bb = ba.St
            // bc = S.Yt
            // bd = bc.Bi
            // b = bb + bd
            // b = (Bi.Y.St + S.Yt.Bi)

            // c = (St.Y)

            // da = c^2
            // db = a . dai
            // dc = b.ci
            // d = db - dc
            // d = (a . (c.c)i) - (b . ci) 

            // (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));

            arma::mat St = S.t();
            double aa = arma::as_scalar(S.t() * Y);
            //arma::mat Yt = Y.t();
            arma::mat ab = Y.t() * Bi;
            double ac = arma::as_scalar(ab * Y);
            arma::mat ad = S * S.t();
            double ae = aa + ac;
            arma::mat a = ae * ad;

            arma::mat ba = Bi * Y;
            arma::mat bb = ba * S.t();
            arma::mat bc = S * Y.t();
            arma::mat bd = bc * Bi;
            arma::mat b = bb + bd;

            double c = arma::as_scalar(S.t() * Y);

            double da = c * c;
            arma::mat db = a * (1/da);
            arma::mat dc = b * (1/c);
            arma::mat d = db - dc;
            Bi += d;
            X=Xp;
        }
        mu=mu* 0.9;  
    }
    return X;
}


int main (){
    ForwardKinematics fktoo;
    arma::vec yeet;
    yeet << 0 << 0 << 0 << 0 << 0 << arma::endr;
    std::cout << fktoo.GetExtendedPositionVector(yeet) << std::endl;

    arma::dmat mat = BFGS(320, 0, 65, yeet);
    std::cout << mat[0] << std::endl;    
    std::cout << mat[1] << std::endl;    
    std::cout << mat[2] << std::endl;
    std::cout << mat[3] << std::endl;
    std::cout << mat[4] << std::endl;

    std::cout << "Press Any Key to Continue..." << std::endl;    
    std::cin.get();
    return 0;
}