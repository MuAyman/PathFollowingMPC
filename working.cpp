#include <nlopt.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>

using namespace std;

// Weighting matricies for the MPC objective function
double Qpos = 10;  // Q for position (x,y)
double Qyaw = 0.1; // Q for yaw
double R = 2;      // R for v nad alphadot

struct States
{
     double x = 0;             // initially set to zeroes
     double y = 0;             // in meters
     double yaw = 0;           // in radians
     double steeringAngle = 0; // in radians

     // setter for steeringAngle to limit steering to pi/4
     void setsteeringAngle(double angle) // violation - this constriants should be set by MPC
     {
          if (angle > M_PI / 4)
               angle = M_PI / 4;
          else if (angle < -M_PI / 4)
               angle = -M_PI / 4;
          steeringAngle = angle;
     }
};

struct ControlInputs
{
     double velocity = 0;          // m/s
     double steeringAngleRate = 0; // rad/second
     ControlInputs() {}
     ControlInputs(double v, double s) : velocity(v), steeringAngleRate(s) {}
};

class Vehicle
{
public:
     States state;
     double L;            // wheelbase of the vehicle in m
     double dt;           // time step in seconds
     ControlInputs prevI; // previous control input (for derivatives)

     // Constructors
     Vehicle() : L(2.0), dt(0.1) {}

     Vehicle(double dt) : L(2.0), dt(dt) {}

     Vehicle(double _L, double dt) : L(_L), dt(dt) {}

     States dynamics(const ControlInputs input) const
     {
          States newState;

          // Store previous states for derivative calculations
          States prevS = state;

          // Update vehicle states
          newState.setsteeringAngle(prevS.steeringAngle + input.steeringAngleRate * dt);
          newState.yaw = prevS.yaw + (input.velocity / L * tan(prevS.steeringAngle) * dt);
          newState.y = prevS.y + (input.velocity * sin(prevS.yaw) * dt);
          newState.x = prevS.x + (input.velocity * cos(prevS.yaw) * dt);

          return newState;
     }

     void update(const ControlInputs input)
     {
          // Update vehicle states
          prevI = input;
          state = dynamics(prevI);

          // update previous control input values
     }
};

// container to pass for the object function
struct Container
{
     Vehicle vehicleObj;
     vector<double> trajectoryPoints; // reference fixed point (will be updated to trajectory points)

     Container() {}
     Container(vector<double> traj) : trajectoryPoints(traj) {}
};

double objFunc(unsigned int n, const double *inputs, double *gradient, void *ptr)
{
     Container *containerPtr = static_cast<Container *>(ptr);

     // filling the control input
     ControlInputs controlInput;
     controlInput.velocity = inputs[0];
     controlInput.steeringAngleRate = inputs[1];

     // getting vehicle states for inputs
     States newstate = containerPtr->vehicleObj.dynamics(controlInput);

     // objective function implementation
     double x = newstate.x;
     double y = newstate.y;
     double yaw = newstate.yaw;

     double dt = containerPtr->vehicleObj.dt; // time step
     double L = containerPtr->vehicleObj.L;   // wheelbase of the vehicle

     double v = inputs[0];
     double alphadot = inputs[1]; // steering angle rate

     // Reference trajectory points
     double x_ref = containerPtr->trajectoryPoints[0];
     double y_ref = containerPtr->trajectoryPoints[1];
     double yaw_ref = containerPtr->trajectoryPoints[2];

     // calculating the objective function value
     double objF = Qpos * (pow((x - x_ref), 2) + pow((y - y_ref), 2)) +
                   Qyaw * pow((yaw - yaw_ref), 2) +
                   R * (pow(v, 2) + pow(alphadot, 2));

     if (gradient != NULL)
     { // gradient df/dv, df/dalphadot
          gradient[0] = 2 * R * v +
                        2 * Qpos * ((x - x_ref) * cos(yaw) + (y - y_ref) * sin(yaw)) * dt + // (x_k - x_ref) * cos(yaw_k-1) ???
                        2 * Qyaw * (1.0 / L) * (yaw - yaw_ref) * tan(yaw) * dt;
          gradient[1] = 2 * R * alphadot +
                        2 * Qyaw * (v * dt / L) * (yaw - yaw_ref) * pow(1 / cos(yaw), 2) * dt;
     }

     return objF;
}

double *solve(Container v)
{
     nlopt_algorithm algorithmUsed = NLOPT_LD_SLSQP; // sequential quadratic programming (SQP) algorithm for nonlinearly constrained gradient-based optimization
     unsigned int n = 2;                             // problem dimensionality - the number of optimization parameters

     // lower bound
     double lb[2] = {-18, -1}; // v in m/s, alphadot in rad/s
     double ub[2] = {18, 1};   // v in m/s, alphadot in rad/s

     nlopt_opt opt = nlopt_create(algorithmUsed, n); // algorithm and dimensionality
     nlopt_set_lower_bounds(opt, lb);
     nlopt_set_upper_bounds(opt, ub);
     nlopt_set_min_objective(opt, objFunc, &v);

     double toleranceValue = 1e-6;
     int toleranceDuration = 5;
     nlopt_set_ftol_rel(opt, toleranceValue);   // tolerance for relative improvement between iterations in the objective function
     nlopt_set_xtol_rel(opt, toleranceValue);   // tolerance for relative improvement between iterations in the optimization parameters (control inputs)
     nlopt_set_maxtime(opt, toleranceDuration); // tolerance for relative improvement between iterations in the objective function

     double minf;                     // the minimum objective value, upon return
     double *x = new double[2]{0, 0}; // intital guess // dynamics allocation to return it
     int sx = nlopt_optimize(opt, x, &minf);
     if (sx < 0)
          printf("nlopt failed!\n");
     else
          printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
     x[4] = minf;
     return x;
}

int main()
{
     Vehicle car;
     Container v;
     v.trajectoryPoints = {10.0, 0.0, 0.0};

     ofstream outfile("trajectory.csv");
     outfile << "x,y,yaw,steeringAngle" << "\n";
     int i = 0;
     double tol = 10, prev = 10;
     while (prev > 1e-6)
     {
          double *in = solve(v);
          ControlInputs inputs(in[0], in[1]);
          v.vehicleObj.update(inputs);

          outfile << v.vehicleObj.state.x << "," << v.vehicleObj.state.y << "," << v.vehicleObj.state.yaw << "," << v.vehicleObj.state.steeringAngle << "\n";
          cout << "[i]: " << i << "\t";

          prev = in[4];
          ++i;
     }
     outfile.close();

     return 0;
}