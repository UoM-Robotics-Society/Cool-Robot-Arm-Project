#include "la.h"
#include "linesearch.h"
#include "forwardkinematics.h"
#include "frametimer.h"

#define MU_INIT 1.0

// kind of dumb class but helps debug the maths of the algorithm
class BFGS {
    private:
        // servo angles
        LA::vecd<5> X = LA::vecd<5>();
        LineSearch ls = LineSearch(0.057, 0.365, 0.430, 0);
        ForwardKinematics fk = ForwardKinematics();
        LA::vecd<5> zeros, Xp, P, S, Y = LA::vecd<5>();
        int count = 0;
        LA::matd<5, 5> Bii = LA::matd<5, 5>();
        LA::matd<5, 5> Bi = Bii;
        double mu = MU_INIT;
        LA::vecd<5> costVec;
        double cost = 100;

        FrameTimer ft = FrameTimer();

    public:
        bool debugIteration = false;
        bool debugAlgorithm = false;
    
        // dumps all algorithm variables to the console
        // use for debugging
        void PrintState();
        void PrintIteration();
        void PrintPos();
        
        // runs the algorithm itself
        // pass in pointer for output motors as it will fill vector with target motor positions
        // calls reset for you 
        bool Run (double x, double y, double z, LA::vecd<5> current_motors, LA::vecd<5>& output_motors);
        // resets all the algorithm variables so object can be reused
        void Reset();
};