#pragma once
// Standard Libraries
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <ctime>
// C.R.A.P Sub-Systems
#include "linesearch.h"
#include "forward_kinematics.h"
#include "la.h"


/*
Prime class that encapsulates all accesses to the arm with proper integration of all systems.
*/
class Arm {
    private:
        LineSearch ls = LineSearch(0.057,0.365,0.430,0);
        ForwardKinematics fk = ForwardKinematics();
        LA::vecd<5> current_motors = {0, 0, 0, 0, 0};

        LA::vecd<5> BFGS (double x, double y, double z) {
            LA::vecd<5> X = current_motors;
            int count, local_min_count = 0;
            LA::matd<5, 5> Bii = LA::matd<5, 5>();
            LA::matd<5, 5> Bi = Bii;
            LA::vecd<5> zeros(0.0), Xp(0.0), P(0.0), S(0.0), Y(0.0);
            double A;
            ls.set_goal(x,y,z);
            double mu = 1000;

            while(true){
                double costdiff = 100;
                double cost = 100;
                double count = 0;
                while (cost > 0.000001){
                    count++;
                    if(count > 100) break;
                    LA::vecd<5> costVec = ls.cost_function_gradient(X,0,mu);
                    cost = ls.cost_function(X,0,mu);
                    if(true){
                        std::cout << "Cost: " << cost << std::endl;
                        std::cout << "MU: " << mu << std::endl;
                    }
                    if (false) {
                        std::cout << "Bi: " << std::endl;
                        print(Bi);
                    }
                    if (false) {
                        std::cout << "cost grad: " << std::endl;
                        print(costVec);
                    }
                    P = -(Bi*costVec);
                    if (false) {
                        // Debug
                        std::cout << "P: " << std::endl;
                        print(P);
                    }
                    
                    if(std::isnan(P[0])){
                        if(local_min_count > 10){
                            std::cout << "Stuck in over 10 local minima" << std::endl;
                            return X;
                        } else {
                            local_min_count++;
                            std::cout << "Error: Local minima after " << local_min_count << " attempts" << std::endl;
                            float q[5];
                            std::cout << "Restarting from ";
                            for(int i = 0; i<5;i++){
                                q[i] = ls.min_angle[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(ls.max_angle[i]-ls.min_angle[i])));
                                X[i] = q[i];
                                std::cout << q[i] << " ";
                            }
                            std::cout << std::endl;
                            mu = 0;
                            Bi = Bii;
                            Xp, P, S, Y = zeros;
                            break;
                        }
                    }
                    Xp = ls.GoldenSearch(X, P, mu);
                    costdiff = abs(ls.cost_function(X,0,mu) - ls.cost_function(Xp,0,mu));
                    S = Xp - X;
                    if (false) {
                        //std::cout << "---------------------" << std::endl;
                        LA::vecd<3> pos = fk.GetExtendedPositionVector(X);
                        std::cout << "pos: " << std::endl;
                        LA::print(pos);
                    }
                    if (false) {
                        std::cout << "X: " << std::endl;
                        LA::print(X);
                    }
                    if (false) {
                        std::cout << "Xp = " << std::endl;
                        LA::print(Xp);
                    }

                    LA::vecd<5> cost1 = ls.cost_function_gradient(Xp, 0, mu);
                    LA::vecd<5> cost2 = ls.cost_function_gradient(X,0, mu);      
                    if (false) {
                        std::cout << "Cost1: " << std::endl;
                        LA::print(cost1);

                        std::cout << "Cost2: " << std::endl;
                        LA::print(cost2);
                    }
                    Y = cost1 - cost2;

                    // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
                    // (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));

                    LA::matd<1, 5> St = LA::transpose(S);
                    LA::matd<1, 5> Yt = LA::transpose(Y);
                    double aa = LA::as_scalar(St * Y);
                    
                    LA::matd<1,5> ab = LA::transpose(Y) * Bi;
                    double ac = LA::as_scalar(ab * Y);
                    LA::matd<5,5> ad = S * St;
                    double ae = aa + ac;
                    LA::matd<5, 5> a = ae * ad;

                    LA::vecd<5> ba = Bi * Y;
                    LA::matd<5, 5> bb = ba * St;
                    LA::matd<5, 5> bc = S * Yt;
                    LA::matd<5, 5> bd = (bc * Bi);
                    LA::matd<5, 5> b = bb + bd;

                    double c = LA::as_scalar(St * Y);
                    if (false) {
                        std::cout << "S = "; print(S);
                        std::cout << "Y = "; print(Y);
                    }

                    double da = c * c;
                    LA::matd<5, 5> db = a * (1/da);
                    LA::matd<5, 5> dc = b * (1/c);
                    LA::matd<5, 5> d = db - dc;
                    Bi = Bi + d;
                    X=Xp;
                    count += 1;
                    if (false) {
                        std::cout << "db = "; print(db);
                        std::cout << "dc = "; print(dc);
                        std::cout << "d = "; print(d);
                    }
                    if (false) {
                        std::cout << "Bi: " << std::endl;
                        print(Bi);
                    }
                    if(count > 50){cost = 0;}
                }
                if(ls.dist_to_goal(X,0) > 0.002){
                //if(mu>0.001){
                    mu = mu*0.9;
                }
                else{
                    return X;
                }
            }
            return X;
        }

    public:
        /*
        Calls arm to move to specified 3D co-ordinate immediately.
        @param x co-ordinate
        @param y co-ordinate
        @param z co-ordinate
        @param error code written int pointer for validation (OPTIONAL)
        @returns true if position was reached, false otherwise
        */
        bool MoveTo(double x, double y, double z, int* error = nullptr) {
            LA::vecd<5> res = BFGS(x, y, z);
            print(res);
        }

        /*
        Retrieves current arm position.
        @param x co-ordinate pointer
        @param y co-ordinate pointer
        @param z co-ordinate pointer
        @returns true if success, false otherwise
        */
        bool GetPosition(double* x, double* y, double* z) {
            LA::vecd<3> pos = fk.GetExtendedPositionVector(current_motors);
            *x = pos[0];
            *y = pos[1];
            *z = pos[2];
            return true;
        }
};