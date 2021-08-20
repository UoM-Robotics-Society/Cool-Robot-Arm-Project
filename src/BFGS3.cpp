#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include "la.h"
#include "linesearch.h"
#include "forward_kinematics.h"

#define GOAL_AXIS_MAX 0.3
#define GOAL_AXIS_MIN 0.1
#define DEBUG_ITERATION true
#define DEBUG_START_INFO true

#define MU_INIT 1000.0
#define MU_BROKEN 0.000002
#define COST_THRESHOLD 0.0005
#define DIST_THRESHOLD 0.002

LA::vecd<5> BFGS (double x, double y, double z, LA::vecd<5> current_motors) {
    LineSearch ls = LineSearch(0.057,0.365,0.430,0);
    ForwardKinematics fk = ForwardKinematics();

    LA::vecd<5> X = current_motors;
    int count, local_min_count = 0;
    LA::matd<5, 5> Bii = LA::matd<5, 5>();
    LA::matd<5, 5> Bi = Bii;
    LA::vecd<5> zeros(0.0), Xp(0.0), P(0.0), S(0.0), Y(0.0);
    double A;
    ls.set_goal(x,y,z);
    double mu = MU_INIT;
    while(true) {
        double costdiff = 100;
        double cost = 100;
        double count = 0;

        while (cost > COST_THRESHOLD){
            count++;
            if (count > 100)
                break;
            LA::vecd<5> costVec = ls.cost_function_gradient(X,0,mu);
            cost = ls.cost_function(X,0,mu);
            if (DEBUG_ITERATION) {
                std::cout << "Cost: " << cost << std::endl;
                std::cout << "MU: " << mu << std::endl;
            }
            if (false) {
                std::cout << "Bi: " << std::endl; print(Bi);
            }
            if (false) {
                LA::vecd<3> pos = fk.GetExtendedPositionVector(X);
                std::cout << "pos: "; print(pos);
            }
            if (false) {
                std::cout << "cost grad = "; print(costVec);
                std::cout << "P: "; print(P);
            }
            P = -(Bi*costVec);
            if (false) {
                std::cout << "P: "; print(P);
            }
            
            if(std::isnan(P[0]) || mu < MU_BROKEN) {
                if (mu < MU_BROKEN && ls.dist_to_goal(X,0) < DIST_THRESHOLD * 1.5 && ls.InBounds(X)) {
                    std::cout << "WARNING:\n\n\nALLOWED WITHIN EXTENDED THRESHOLD\n\n\n" << std::endl;
                    return X;
                }
                if(local_min_count > 10){
                    std::cout << "Stuck in over 10 local minima" << std::endl;
                    return X;
                } else {
                    local_min_count++;
                    std::cout << "Error: Local minima after " << local_min_count << " attempts" << std::endl;
                    
                    std::cout << "Restarting from ";
                    bool isInBounds = false;
                    LA::vecd<5> q;
                    while(!isInBounds) {
                        for(int i = 0; i<5;i++) {
                            q[i] = ls.min_angle[i] + static_cast<float>(rand()) / ( static_cast <float> (RAND_MAX / (ls.max_angle[i] - ls.min_angle[i])));
                        }
                        isInBounds = ls.InBounds(q);
                    }
                    X = q;
                    std::cout << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << std::endl;
                    mu = MU_INIT;
                    Bi = Bii;
                    Xp, P, S, Y = zeros;
                    break;
                }
            }
            Xp = ls.GoldenSearch(X, P, mu);
            costdiff = abs(ls.cost_function(X,0,mu) - ls.cost_function(Xp,0,mu));
            S = Xp - X;
            if (false) {
                LA::vecd<3> pos = fk.GetExtendedPositionVector(X);
                std::cout << "pos: "; print(pos);
            }
            if (false) {
                std::cout << "X: "; print(X);
            }
            if (false) {
                std::cout << "Xp: "; print(Xp);
            }

            LA::vecd<5> cost1 = ls.cost_function_gradient(Xp, 0, mu);
            LA::vecd<5> cost2 = ls.cost_function_gradient(X,0, mu);      
            if (false) {
                std::cout << "cost1: "; print(cost1);
                std::cout << "cost2: "; print(cost2);
            }
            Y = cost1 - cost2;

            // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
            // (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));

            LA::matd<1, 5> St = LA::transpose(S);
            LA::matd<1, 5> Yt = LA::transpose(Y);

            LA::matd<5, 5> a = (LA::as_scalar(St * Y) + LA::as_scalar((Yt * Bi) * Y)) * (S * St);
            LA::matd<5, 5> b = (Bi * Y) * St + (S * Yt) * Bi;
            double c = LA::as_scalar(St * Y);
            LA::matd<5, 5> d = (a * (1 / (c * c))) - (b * (1/c));

            if (false) {
                std::cout << "a: " << std::endl; 
                print(a);
                std::cout << "b: " << std::endl; 
                print(b);
                std::cout << "c: " << c << std::endl;
                std::cout << "d: " << std::endl; 
                print(d);
                std::cout << "S = "; print(S);
                std::cout << "Y = "; print(Y);
            }


            Bi = Bi + d;
            X=Xp;
            count += 1;
            if (false) {
                std::cout << "d: "; print(d);
                std::cout << "Bi: " << std::endl; print(Bi);
            }
            if (count > 50) {
                cost = 0;
            }
        }
        
        if (false) {
            LA::vecd<3> pos = fk.GetExtendedPositionVector(X);
            std::cout << "pos: "; print(pos, true);
        }
        if (ls.dist_to_goal(X,0) > DIST_THRESHOLD) {
            mu = mu * 0.9;
        } else {
            return X;
        }
    }
    
    return X;
}
 


int main () {
    LineSearch ls(0.057,0.365,0.430,0);
    ForwardKinematics fktoo;
    LA::vecd<5> yeet = LA::vecd<5>(0.0);
    LA::vecd<5> start = LA::vecd<5>(0.2);
    start[4] = 0.0;

    double x, y, z = 0;
    srand (static_cast <unsigned> (time(0)));
    
    LA::vecd<3> position;
    bool goalInBounds = false;
    while (!goalInBounds) {
        x = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            x = -x;
        y = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            y = -y;
        z = double(rand()) / double((RAND_MAX)) * (GOAL_AXIS_MAX - GOAL_AXIS_MIN) + GOAL_AXIS_MIN;
        if (rand() > 0.5)
            z = -z;
        // x = 0.16;
        // y = 0.15;
        // z = 0.2;
        ls.set_goal(x, y, z);
        position = { x, y, z };
        goalInBounds = ls.InBoundsPos(position);
    }
    
    
    if (DEBUG_START_INFO) {
        std::cout << "---------------------" << std::endl;
        std::cout << "Goal: " << x << ", " << y << ", " << z << std::endl;
        std::cout << "---------------------" << std::endl;
        std::cout << "Starting motor angles: ";
        print(start);
        std::cout << "Starting Coordinates: ";
        print(fktoo.GetExtendedPositionVector(start),3);
        std::cout << "Start in-bounds test: " << ls.InBounds(start) << std::endl;
        std::cout << "---------------------" << std::endl;
    }
    
    std::cout << "Running BFGS Algorithm..." << std::endl;
    LA::vecd<5> end = BFGS(x, y, z, start);
    std::cout << "---------------------" << std::endl;
    std::cout << "Caluclated Motor Angles: ";
    std::cout << end[0] << " " << end[1] << " " << end[2] << " " << end[3] << " " << end[4] << std::endl;
    LA::vecd<5> vmat = end;

    bool isResultInBounds = ls.InBounds(end);
    std::cout << "Result in Bounds: " << isResultInBounds << std::endl;
    
    ls.set_goal(x,y,z);
    double cost = ls.cost_function(vmat, 0, 0.00001);
    std::cout << "Final Cost = " << cost << std::endl;

    std::cout << "Final Grad = " << std::endl;
    LA::print(ls.cost_function_gradient(vmat, 0.0, 0.00001));
    
    LA::vecd<3> pos = fktoo.GetExtendedPositionVector(vmat);
    std::cout << "---------------------" << std::endl;
    std::cout << "Target Position: " << x << " " << y << " " << z << std::endl;
    std::cout << "FK Calculated Position Based off Angles: ";
    std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << "Press Enter Key to Continue..." << std::endl;     
    std::cin.get();
    return 0;
}