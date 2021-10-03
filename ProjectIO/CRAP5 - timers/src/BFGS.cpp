#include "la.h"
#include "linesearch.h"
#include "forwardkinematics.h"
#include "BFGS.h"

#define MU_BROKEN 0.000002
#define COST_THRESHOLD 0.0005
#define DIST_THRESHOLD 0.002
#define MU_MULTIPLIER 0.5

// dumps all algorithm variables to the console
// use for debugging
void BFGS::PrintState() {
    std::cout << "Bi: " << std::endl;
    print(Bi);
    std::cout << "pos: ";
    print(fk.GetExtendedPositionVector(X));
    std::cout << "cost grad = ";
    print(costVec);
    std::cout << "P: ";
    print(P);
}

void BFGS::PrintIteration() {
    Serial.print("\nCost: ");
    Serial.println(cost, DEC);
    Serial.print("Mu: ");
    Serial.println(mu, DEC);
    //std::cout << "X: ";
    //print(X, true);
}


void BFGS::PrintPos() {
    std::cout << "pos: ";
    print(fk.GetExtendedPositionVector(X));
}

bool BFGS::Run (double x, double y, double z, LA::vecd<5> current_motors, LA::vecd<5>& output_motors) {
    Reset();
    ft.Start();
    X = current_motors;
    ls.set_goal(x,y,z);
    int count= 0;
    
    while (true) {
        cost = 100;
        count = 0;

        while (cost > COST_THRESHOLD && count < 20) {
            costVec = ls.cost_function_gradient(X, 0, mu);
            cost = ls.cost_function(X, 0, mu);
            P = -(Bi * costVec);
            
            if (debugIteration)
            PrintIteration();

            if (mu < MU_BROKEN && ls.dist_to_goal(X,0) < DIST_THRESHOLD * 2 && ls.InBounds(X)) {
                Serial.println("WARNING:\n\nALLOWED WITHIN EXTENDED THRESHOLD\n\n");
                output_motors = X;
                return false;
            } else if (mu < MU_BROKEN) {
                Serial.println("ERROR: Could not reach goal within mu time frame");
                output_motors = X;
                return false;
            }
            if (std::isnan(P[0])) {
                Serial.println("ERROR: Stuck in local minima/singularity");
                output_motors = X;
                return false;
            }
            ft.Frame();
            Serial.print("\n\tBFGS before GS: ");
            Serial.println(ft.GetLastFrameElapsed(), DEC);
            Serial.print("\tTotal Elapsed: ");
            Serial.println(ft.GetTotalElapsed(), DEC);
            Xp = ls.GoldenSearch(X, P, mu);
            ft.Frame();                            
            Serial.print("\n\tGolden Search: "); 
            Serial.println(ft.GetLastFrameElapsed(), DEC);
            Serial.print("\tTotal Elapsed: ");
            Serial.println(ft.GetTotalElapsed(), DEC);
            //costDiff = abs(ls.cost_function(X,0,mu) - ls.cost_function(Xp,0,mu));
            S = Xp - X;
            LA::vecd<5> cost1 = ls.cost_function_gradient(Xp, 0, mu);
            LA::vecd<5> cost2 = ls.cost_function_gradient(X,0, mu);
            Y = cost1 - cost2;

            // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
            // (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));

            LA::matd<1, 5> St = LA::transpose(S);
            LA::matd<1, 5> Yt = LA::transpose(Y);
            LA::matd<5, 5> a = (LA::as_scalar(St * Y) + LA::as_scalar((Yt * Bi) * Y)) * (S * St);
            LA::matd<5, 5> b = (Bi * Y) * St + (S * Yt) * Bi;
            double c = LA::as_scalar(St * Y);
            LA::matd<5, 5> d = (a * (1 / (c * c))) - (b * (1/c));

            Bi = Bi + d;
            X = Xp;
            count += 1;
            if (debugAlgorithm)
                PrintState();

            // if (DEBUG_ALGORITHM) {
            //     std::cout << "a: " << std::endl; 
            //     print(a);
            //     std::cout << "b: " << std::endl; 
            //     print(b);
            //     std::cout << "c: " << c << std::endl;
            //     std::cout << "d: " << std::endl; 
            //     print(d);
            //     std::cout << "S: ";
            //     print(S);
            //     std::cout << "Y: ";
            //     print(Y);
            //     std::cout << "d: ";
            //     print(d);
            //     std::cout << "Bi: " << std::endl;
            //     print(Bi);
            // }
            ft.Frame();
            Serial.print("\n\tEnd of BFGS: ");
            Serial.println(ft.GetLastFrameElapsed(), DEC);
            Serial.print("\tTotal Elapsed: ");
            Serial.println(ft.GetTotalElapsed(), DEC);
        }

        if (ls.dist_to_goal(X, 0) > DIST_THRESHOLD) {
            mu = mu * MU_MULTIPLIER;
        } else {
            output_motors = X;
            return true;
        }
        
        ft.Frame();
        Serial.print("\nFinished 20 iterations: ");
        Serial.println(ft.GetLastFrameElapsed(), DEC);
        Serial.print("Total Elapsed: ");
        Serial.println(ft.GetTotalElapsed(), DEC);
    }
}

void BFGS::Reset() {
    Bii = LA::matd<5, 5>();
    Bi = Bii;
    zeros, Xp, P, S, Y = LA::vecd<5>(0.0);
    mu = MU_INIT;
    ft = MillisTimer();
}