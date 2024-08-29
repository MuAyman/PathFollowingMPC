#include <nlopt.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "model.hpp"

using namespace std;

// Weighting matrices for the MPC objective function
const double Qx = 10.0;  // Q for x position
const double Qy = 20.0;  // Q for y position
const double Qf = 30.0;  // Q for Euclidean x, y position
const double Qyaw = 3.0; // Q for yaw
const double R = 0.1;    // R for velocity and steering rate

// Reference trajectory point (example)
const double xx_ref = -100;
const double yx_ref = -500;
const vector<double> trajectory = {xx_ref, yx_ref, atan(yx_ref / xx_ref)};

// Parameters for simulation
const int runs = 500;
const unsigned int predictionHorizon = 50; // Nc = Np

// Constraints for velocity and steering rate
const double maxVelocity = 20.0;     // Maximum allowable velocity (m/s)
const double minVelocity = -20.0;    // Minimum allowable velocity (m/s)
const double maxSteeringRate = 1.0;  // Maximum allowable steering rate (rad/s)
const double minSteeringRate = -1.0; // Minimum allowable steering rate (rad/s)

const double toleranceValue = 1e-6;

struct Container
{
     Vehicle vehicleObj;
     Vehicle predictionvehicleObj;
     vector<double> trajectoryPoints;

     Container() {}
     Container(vector<double> traj) : trajectoryPoints(traj) {}
};

class NLMPC
{
public:
     Container &container;
     unsigned int n;
     nlopt_algorithm algorithmUsed = NLOPT_LD_SLSQP; // sequential quadratic programming (SQP) algorithm for nonlinearly constrained gradient-based optimization
     nlopt_opt opt = nlopt_create(algorithmUsed, n);

     NLMPC(Container &v) : container(v), n(2 * predictionHorizon)
     {
          setBounds();
          initOpt();
     }

     double *solve()
     {
          double *x = new double[n]{}; // Initial guess

          double minf;
          if (nlopt_optimize(opt, x, &minf) < 0)
               printf("nlopt failed!\n");
          else
               printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);

          x[4] = minf;
          return x;
     }

     static double objFunc(unsigned int n, const double *inputs, double *gradient, void *ptr)
     {
          Container *containerPtr = static_cast<Container *>(ptr);
          double totalCost = 0.0;

          containerPtr->predictionvehicleObj.state = containerPtr->vehicleObj.state;
          States predictedState = containerPtr->predictionvehicleObj.state;

          for (unsigned int k = 0; k < predictionHorizon; ++k)
          {
               ControlInputs controlInput = {inputs[2 * k], inputs[2 * k + 1]};
               containerPtr->predictionvehicleObj.update(controlInput);
               predictedState = containerPtr->predictionvehicleObj.state;

               double x = predictedState.x;
               double y = predictedState.y;
               double yaw = predictedState.yaw;
               double steeringAngle = predictedState.steeringAngle;

               double dt = containerPtr->vehicleObj.dt;
               double L = containerPtr->vehicleObj.L;

               double v = controlInput.velocity;
               double alphadot = controlInput.steeringAngleRate;

               double x_ref = containerPtr->trajectoryPoints[0];
               double y_ref = containerPtr->trajectoryPoints[1];
               double yaw_ref = containerPtr->trajectoryPoints[2];

               double objF = Qx * pow((x - x_ref), 2) +
                             Qy * pow((y - y_ref), 2) +
                             Qyaw * pow((yaw - yaw_ref), 2) +
                             Qf * (pow((x - x_ref), 2) +
                                   pow((y - y_ref), 2)) +
                             R * (pow(v, 2) + pow(alphadot, 2));

               totalCost += objF;

               if (gradient != NULL)
               {
                    gradient[2 * k] = 2 * R * v +
                                      2 * Qx * (x - x_ref) * (cos(yaw) * dt) +
                                      2 * Qy * (y - y_ref) * (sin(yaw) * dt) +
                                      2 * Qyaw * (yaw - yaw_ref) * (1.0 / L) * (tan(steeringAngle) * dt);
                    gradient[2 * k + 1] = 2 * R * alphadot +
                                          2 * Qyaw * (v * dt / L) * (yaw - yaw_ref) * pow(1 / cos(steeringAngle), 2) * dt;
               }
          }
          return totalCost;
     }

     void setBounds()
     {
          double lb[n], ub[n];
          for (unsigned int i = 0; i < n; i += 2)
          {
               lb[i] = minVelocity;         // v in m/s
               ub[i] = maxVelocity;         // v in m/s
               lb[i + 1] = minSteeringRate; // alphadot in rad/s
               ub[i + 1] = maxSteeringRate; // alphadot in rad/s
          }

          nlopt_set_lower_bounds(opt, lb);
          nlopt_set_upper_bounds(opt, ub);
     }

     void initOpt()
     {

          nlopt_set_min_objective(opt, objFunc, &container);
          nlopt_set_ftol_rel(opt, toleranceValue);
          nlopt_set_xtol_rel(opt, toleranceValue);
          nlopt_set_maxtime(opt, 5); // 5 seconds timeout per iteration
     }
};

int main()
{
     Container v(trajectory);

     ofstream outfile("trajectory.csv");
     outfile << "x,y,yaw,steeringAngle,velocity,srate" << "\n";

     NLMPC nlmpc(v);

     for (int i = 0; i < runs; ++i)
     {
          double *in = nlmpc.solve();
          ControlInputs inputs(in[0], in[1]); // Apply only the first control input
          v.vehicleObj.update(inputs);

          outfile << v.vehicleObj.state.x << "," << v.vehicleObj.state.y << "," << v.vehicleObj.state.yaw
                  << "," << v.vehicleObj.state.steeringAngle << "," << inputs.velocity << "," << inputs.steeringAngleRate << "\n";
          cout << "[i]: " << i << "\t";
          cout << "[f]: " << in[4] << "\t";

          delete[] in; // Free the dynamically allocated memory
     }

     outfile.close();

     return 0;
}
