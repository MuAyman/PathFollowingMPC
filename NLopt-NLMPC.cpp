#include <nlopt.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Weighting matricies for the MPC objective function
double Qpos = 1; // Q for position (x,y)
double Qyaw = 1; // Q for yaw
double R = 1;    // R for v nad alphadot

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
};

class Vehicle
{
public:
     States state;
     double L;            // wheelbase of the vehicle in m (defult of 2m)
     double dt;           // time step in seconds (defult of 0.1s)
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
          state = dynamics(prevI);

          // update previous control input values
          prevI = input;
     }

     // void simulate(int steps, ControlInputs input)
     // {
     //      std::ofstream outfile("trajectory.csv");
     //      outfile << "x,y,yaw,steeringAngle" << "\n";
     //      for (int i = 0; i < steps; ++i)
     //      {
     //           input.steeringAngleRate = sin(i / 9);
     //           update(input);
     //           outfile << state.x << "," << state.y << "," << state.yaw << "," << state.steeringAngle << "\n";

     //           // // Print vehicle states every step
     //           // cout << "\nStep " << i;
     //           // cout << ", x: " << state.x << ", y: " << state.y << ", yaw: "
     //           //      << state.yaw << ", steeringAngle: " << state.steeringAngle << "\n";
     //      }
     //      outfile.close();
     // }
};

// container to pass for the object function
struct Container
{
     Vehicle vehicleObj;
     vector<double> trajectoryPoints = {-5, 0}; // reference fixed point (will be updated to trajectory points)
};

// object function for the mpc
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
     double y_ref = containerPtr->trajectoryPoints[0];
     double yaw_ref = containerPtr->trajectoryPoints[1];

     // calculating the objective function value
     double objF = Qpos * (pow((x - x_ref), 2) + pow((y - y_ref), 2)) +
                   Qyaw * pow((yaw - yaw_ref), 2) +
                   R * (pow(v, 2) + pow(alphadot, 2));

     if (gradient != NULL)
     {
          // gradient df/dv, df/dalphadot
          gradient[0] = 2 * R * v +
                        2 * Qpos * ((x - x_ref) * cos(yaw) + (y - y_ref) * sin(yaw)) * dt + // (x_k - x_ref) * cos(yaw_k-1) ???
                        2 * Qyaw * (1.0 / L) * (yaw - yaw_ref) * tan(yaw) * dt;
          gradient[1] = 2 * R * alphadot +
                        2 * Qyaw * (v * dt / L) * (yaw - yaw_ref) * pow(1 / cos(yaw), 2) * dt;
     }

     return objF;
}

class NLMPC
{
public:
     // Container v;
     unsigned int n;                // problem dimensionality - the number of optimization parameters
     nlopt_algorithm algorithmUsed; // sequential quadratic programming (SQP) algorithm for nonlinearly constrained gradient-based optimization
     nlopt_opt opt;                 // optimization slover ptr
     double toleranceValue = 1e-6;  // tolerance for relative improvement between iterations
     int toleranceDuration = 5;     // Duration tolerance for optimization solver (5 seconds timeout per iteration)

     double *lb = new double[n]; // lower bound
     double *ub = new double[n]; // upper bound

     NLMPC(int n, nlopt_algorithm a = NLOPT_LD_SLSQP) : n(n), algorithmUsed(a)
     {
          opt = nlopt_create(algorithmUsed, n);
     }

     void setBounds(const double *lower, const double *upper)
     {
          for (int i = 0; i < n; ++i)
          {
               lb[i] = lower[i];
               ub[i] = upper[i];
          }
          nlopt_set_lower_bounds(opt, lb);
          nlopt_set_upper_bounds(opt, ub);
     }

     void setTolerance(double tolValue, int tolDuration)
     {
          toleranceValue = tolValue;
          toleranceDuration = tolDuration;
          nlopt_set_ftol_rel(opt, toleranceValue);   // tolerance for relative improvement between iterations in the objective function
          nlopt_set_xtol_rel(opt, toleranceValue);   // tolerance for relative improvement between iterations in the optimization parameters (control inputs)
          nlopt_set_maxtime(opt, toleranceDuration); // tolerance for relative improvement between iterations in the objective function
     };

     void run()
     {
          Container container;
          nlopt_set_min_objective(opt, objFunc, &container);

          double x[2] = {1.0, 0.5}; // some initial guess
          double minf;              // the minimum objective value, upon return
          if (nlopt_optimize(opt, x, &minf) < 0)
          {
               printf("nlopt failed!\n");
               cout << "\nthe number of evaluations: " << nlopt_get_numevals(opt) << "\n\n";
          }
          else
          {
               printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
               cout << "\nthe number of evaluations: " << nlopt_get_numevals(opt) << "\n\n";
          }
     }

     ~NLMPC()
     {
          if (lb != nullptr)
               delete[] lb;
          if (ub != nullptr)
               delete[] ub;
     }
};

int main()
{

     int n = 2;
     double lb[2] = {-18, -1}; // v in m/s, alphadot in rad/s
     double ub[2] = {18, 1};   // v in m/s, alphadot in rad/s

     NLMPC nlmpc(n);
     nlmpc.setBounds(lb, ub);
     nlmpc.run();

     return 0;
}