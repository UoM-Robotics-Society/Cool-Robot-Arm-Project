#include "forward_kinematics.hpp"

// TODO documentation
// ? choose a different value
#define CRAP_MU 0.00001

class LineSearch {
    private:
    double goal_x, goal_y, goal_z;
    
    double radius_inner, radius_outer;
    double height_max, height_min;

    double max_angle[CRAP_NUM_REVOLUTE_FRAMES];
    double min_angle[CRAP_NUM_REVOLUTE_FRAMES];

    ForwardKinematics fk;

    public:
    /*
    Initialize line search.

    TODO improve documentation here
    @param the inner radius. The arm cannot get closer to the base than this.
    @param the outer radius. The arm can't stretch further than this.
    @param the highest the arm can go.
    @param the lowest the arm can go.
    */
    LineSearch(double r_inner, double r_outer, double h_max, double h_min);

    /*
    Set the goal coordinates for the arm's motion.

    @param the x coordinate of the goal.
    @param the y coordinate of the goal.
    @param the z coordinate of the goal.
    */
    void set_goal(double x, double y, double z);
    
    /*
    Calculate the value of the cost function.

    @param array of the angles of the 5 revolute joints.
    @param the position of the final prismatic joint.

    @returns the value of the cost function.
    */
    double cost_function(arma::vec5 q, double d);

    /*
    Calculate the gradient of the cost function with respect
    to the angles of the five revolute joints.

    @param array of the angles of the 5 revolute joints.
    @param the position of the final prismatic joint.

    @returns the gradient of the cost function as a vector.
    */
    arma::vec cost_function_gradient(arma::vec5 q, double d);
};