#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <iostream>
#include <cmath>
#include "linesearch.h"
#include "forward_kinematics.h"

arma::vec BFGS (double x, double y, double z, arma::vec X){

    int count = 0;
    LineSearch arm(0.057,0.365,0.430,0);
    arma::dmat Bii(5,5,arma::fill::eye);
    arma::dmat Bi = Bii;
    arma::vec Xp(5, arma::fill::zeros), P(5,arma::fill::zeros), S(5,arma::fill::zeros), Y(5,arma::fill::zeros);
    double A;
    arm.set_goal(x,y,z);
    double mu = 1000;
    ForwardKinematics fk = ForwardKinematics();
    while(mu > 0.000000000001){
        double costdiff = 100;
        double cost = 100;
        while (cost > 0.000001){
            arma::vec costVec = arm.cost_function_gradient(X,0,mu);
            cost = arm.cost_function(X,0,mu);
            if(false){
                std::cout << "Cost: " << cost << std::endl;
                std::cout << "MU: " << mu << std::endl;
            }
            if (false) {
                std::cout << "Bi: " << std::endl;
                print_mat(Bi);
            }
            if (false) {
                std::cout << "cost grad = ";
                std::cout << costVec[0] << " ";  
                std::cout << costVec[1] << " ";   
                std::cout << costVec[2] << " ";
                std::cout << costVec[3] << " ";
                std::cout << costVec[4] << std::endl;
            }
            P = -(Bi*costVec);
            if(std::isnan(P(0))){
                std::cout << "Error: X = Xp" << std::endl;
                return X;
            }
            if (false) {
                // Debug
                std::cout << "P = ";
                std::cout << P[0] << " ";    
                std::cout << P[1] << " ";  
                std::cout << P[2] << " ";
                std::cout << P[3] << " ";
                std::cout << P[4] << std::endl;
            }
            
            Xp = arm.GoldenSearch(X, P, mu);
            costdiff = abs(arm.cost_function(X,0,mu) - arm.cost_function(Xp,0,mu));
            S = Xp - X;
            if (false) {
                //std::cout << "---------------------" << std::endl;
                arma::vec pos = fk.GetExtendedPositionVector(X);
                std::cout << "";
                std::cout << pos[0] << ",";  
                std::cout << pos[1] << ","; 
                std::cout << pos[2] << std::endl;
            }
            if (false) {
                std::cout << "X = ";
                std::cout << X[0] << " ";    
                std::cout << X[1] << " ";  
                std::cout << X[2] << " ";
                std::cout << X[3] << " ";
                std::cout << X[4] << std::endl;
            }
            if (false) {
                std::cout << "Xp = ";
                std::cout << Xp[0] << " ";    
                std::cout << Xp[1] << " ";  
                std::cout << Xp[2] << " ";
                std::cout << Xp[3] << " ";
                std::cout << Xp[4] << std::endl;
            }

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
            if (false) {
                std::cout << "S = "; print_vec(S);
                std::cout << "Y = "; print_vec(Y);
            }

            double da = c * c;
            arma::mat db = a * (1/da);
            arma::mat dc = b * (1/c);
            arma::mat d = db - dc;
            Bi = Bi + d;
            X=Xp;
            count += 1;
            if (false) {
                std::cout << "db = "; print_mat(db);
                std::cout << "dc = "; print_mat(dc);
                std::cout << "d = "; print_mat(d);
            }
            if (false) {
                std::cout << "Bi: " << std::endl;
                print_mat(Bi);
            }
            if(count > 50){cost = 0;}
        }
        mu = mu*0.9;
    }
    //arma::vec good;
    //good << 0.540419489324968 << 0.334426881574699 << 0.658670308915048 << 0.600956360426415 << 0 << arma::endr;
    //std::cout << arm.cost_function(good,0,mu) << std::endl;
    //std::cout << arm.cost_function_gradient(good,0,mu) << std::endl;
    return X;
}


int main (){
    LineSearch ls(0.057,0.365,0.430,0);

    ForwardKinematics fktoo;
    arma::vec yeet;
    yeet << 0 << 0 << 0 << 0 << 0 << arma::endr;
    arma::vec start;
    start << 0.2 << 0.2 << 0.2 << 0.2 << 0 << arma::endr;
    double x, y, z = 0;

    x = 0.1;
    y = 0.1;
    z = 0.2;
    arma::vec position;
    position << x << y << z << arma::endr;
    std::cout << "---------------------" << std::endl;
    std::cout << "Goal: " << x << ", " << y << ", " << z << std::endl;
    std::cout << "Goal in-bounds test: " << ls.InBoundsPos(position) << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "Starting motor angles: ";
    print_vec(start);
    std::cout << "Startng Coordinates: ";
    print_vec(fktoo.GetExtendedPositionVector(start),3);
    std::cout << "Start in-bounds test: " << ls.InBounds(start) << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "Running BFGS Algorithm..." << std::endl;
    arma::dmat end = BFGS(x, y, z, start);
    std::cout << "---------------------" << std::endl;
    std::cout << "Caluclated Motor Angles: ";
    std::cout << end[0] << " ";    
    std::cout << end[1] << " ";    
    std::cout << end[2] << " ";
    std::cout << end[3] << " ";
    std::cout << end[4] << std::endl;
    arma::vec::fixed<5> vmat = end.as_col();
    
    ls.set_goal(x,y,z);
    std::cout << "Final Cost = " << ls.cost_function(vmat, 0, 0.00001) << std::endl;
    std::cout << "Final Grad = " << ls.cost_function_gradient(vmat, 0, 0.00001) << std::endl;
    
    arma::vec pos = fktoo.GetExtendedPositionVector(vmat);
    std::cout << "---------------------" << std::endl;
    std::cout << "FK Calculated Position Based off Angles: ";
    std::cout << pos[0] << " ";    
    std::cout << pos[1] << " ";    
    std::cout << pos[2] << " ";
    std::cout << "---------------------" << std::endl;
    std::cout << "Press Enter Key to Continue..." << std::endl;     
    std::cin.get();
    return 0;
}