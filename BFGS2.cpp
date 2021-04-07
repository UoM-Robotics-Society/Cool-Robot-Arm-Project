#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <iostream>
#include <cmath>
#include "linesearch.h"
#include "forward_kinematics.h"

arma::vec BFGS (double x, double y, double z, arma::vec X){
    int count = 0;
    LineSearch arm(0.057,0.365,0.430,0);
    arma::dmat Bi(5,5,arma::fill::eye);
    arma::vec Xp(5, arma::fill::zeros), P(5,arma::fill::zeros), S(5,arma::fill::zeros), Y(5,arma::fill::zeros);
    double A;
    arm.set_goal(x,y,z);
    double mu = 0.0001;

    while (mu>0.000001){

       for (int i=0;i<10;i++){
            arma::vec costVec = arm.cost_function_gradient(X,0,mu);
            if (false) {
                std::cout << "---------------------" << std::endl;
                std::cout << "---------------------" << std::endl;
                std::cout << count << std::endl;
                std::cout << "---------------------" << std::endl;
                std::cout << "---------------------" << std::endl;

                std::cout << costVec[0] << std::endl;    
                std::cout << costVec[1] << std::endl;    
                std::cout << costVec[2] << std::endl;
                std::cout << costVec[3] << std::endl;
                std::cout << costVec[4] << std::endl;

                // Debug
                std::cout << "---------------------" << std::endl;
                std::cout << P[0] << std::endl;    
                std::cout << P[1] << std::endl;    
                std::cout << P[2] << std::endl;
                std::cout << P[3] << std::endl;
                std::cout << P[4] << std::endl;
            }
            
            P = -(Bi*costVec);

            Xp = arm.GoldenSearch(X, P, mu);
            S = Xp - X;

            arma::vec cost1 = arm.cost_function_gradient(Xp, 0, mu);
            arma::vec cost2 = arm.cost_function_gradient(X,0, mu);      
            if (false) {
                std::cout << "Cost1: ";
                std::cout << cost1[0] << " ";    
                std::cout << cost1[1] << " ";    
                std::cout << cost1[2] << " ";
                std::cout << cost1[3] << " ";
                std::cout << cost1[4] << std::endl; 

                std::cout << "Cost2: ";
                std::cout << cost2[0] << " ";    
                std::cout << cost2[1] << " ";    
                std::cout << cost2[2] << " ";
                std::cout << cost2[3] << " ";
                std::cout << cost2[4] << std::endl; 
            }
            Y = cost1 - cost2;

            // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
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

            count += 1;
        }
        mu=mu* 0.9;  
    }
    return X;
}


int main (){
    LineSearch ls(0.057,0.365,0.430,0);

    ForwardKinematics fktoo;
    arma::vec yeet;
    yeet << 0 << 0 << 0 << 0 << 0 << arma::endr;
    double x, y, z = 0;
    std::cout << fktoo.GetExtendedPositionVector(yeet) << std::endl;
    x = 0.02;
    y = 0;
    z = 0.12;
    for (int i = 0; i < 5; i++) {
        y += 0.05;
        std::cout << "---------------------" << std::endl;
        std::cout << x << ", " << y << ", " << z << std::endl;
        arma::dmat mat = BFGS(x, y, z, yeet);
        std::cout << "---------------------" << std::endl;
        std::cout << "Caluclated Motor Angles" << std::endl;
        std::cout << mat[0] << std::endl;    
        std::cout << mat[1] << std::endl;    
        std::cout << mat[2] << std::endl;
        std::cout << mat[3] << std::endl;
        std::cout << mat[4] << std::endl;
        arma::vec::fixed<5> vmat = mat.as_col();
       
        ls.set_goal(x,y,z);
        std::cout << "Final Cost = " << ls.cost_function(vmat, 0, 0.00001) << std::endl;
        std::cout << "Final Grad = " << ls.cost_function_gradient(vmat, 0, 0.00001) << std::endl;
        
        arma::vec pos = fktoo.GetExtendedPositionVector(vmat);
        std::cout << "---------------------" << std::endl;
        std::cout << "FK Calculated Position Based of Angles" << std::endl;
        std::cout << pos[0] << std::endl;    
        std::cout << pos[1] << std::endl;    
        std::cout << pos[2] << std::endl;
        std::cout << pos[3] << std::endl;
          
    }
    std::cout << "Press Enter Key to Continue..." << std::endl;     
    std::cin.get();
    return 0;
}