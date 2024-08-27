#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;

struct States
{
     double x = 0;             // initially set to zeroes
     double y = 0;             // in meters
     double yaw = 0;           // in radians
     double steeringAngle = 0; // in radians

     // setter for steeringAngle to limit steering to pi/4
     // if above pi/4, set to pi/4, if less than -pi/4, set to -pi/4
     void setSteeringAngle(double angle)
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
     double steeringAngleRate = 0; // rad/second
     double velocity = 0;          // m/s
};

class Vehicle
{
public:
     States state_exact;
     States state_approx;
     double L;            // wheelbase of the vehicle in m
     double wheel_radius; // wheel radius in m
     double dt;           // time step in seconds

     // Constructors
     Vehicle(double dt) : L(2.0), wheel_radius(0.2), dt(dt) {}

     Vehicle(double _L, double _wheel_radius, double dt) : L(_L), wheel_radius(_wheel_radius), dt(dt) {}

     void updateExact(ControlInputs input)
     {
          // Update vehicle states using the exact model
          state_exact.setSteeringAngle(state_exact.steeringAngle + input.steeringAngleRate * dt);
          state_exact.yaw += input.velocity / L * tan(state_exact.steeringAngle) * dt;
          state_exact.y += input.velocity * sin(state_exact.yaw) * dt;
          state_exact.x += input.velocity * cos(state_exact.yaw) * dt;
     }

     void updateApprox(ControlInputs input)
     {
          // Store previous states and inputs for the approximate model
          States prevS = state_approx;
          ControlInputs prevI = input;

          // Update vehicle states using the approximate model
          state_approx.setSteeringAngle(prevS.steeringAngle + input.steeringAngleRate * dt);
          state_approx.yaw = prevS.yaw + ((prevI.velocity / L * tan(prevS.steeringAngle)) +
                                          (prevI.velocity / (L * pow(cos(prevS.steeringAngle), 2))) *
                                              (state_approx.steeringAngle - prevS.steeringAngle) +
                                          (tan(prevS.steeringAngle) / L * (input.steeringAngleRate - prevI.steeringAngleRate))) *
                                             dt;
          state_approx.y = prevS.y + ((prevI.velocity * sin(prevS.yaw)) +
                                      (prevI.velocity * cos(prevS.yaw)) * (state_approx.yaw - prevS.yaw) +
                                      (sin(prevS.yaw) * (input.velocity - prevI.velocity))) *
                                         dt;
          state_approx.x = prevS.x + ((prevI.velocity * cos(prevS.yaw)) +
                                      (-prevI.velocity * sin(prevS.yaw)) * (state_approx.yaw - prevS.yaw) +
                                      (cos(prevS.yaw) * (input.velocity - prevI.velocity))) *
                                         dt;
     }

     double calculateError()
     {
          // Calculate the Euclidean distance error between exact and approximate states
          double dx = abs(state_exact.x - state_approx.x);
          double dy = abs(state_exact.y - state_approx.y);

          double avg = (dx + dy) / 2;
          return avg;
     }
};

int main()
{
     double dt = 0.05; // seconds
     int steps = 500;

     Vehicle vehicle(dt);

     ControlInputs input = {0.15, 15};

     std::ofstream outfile("trajectory_comparison.csv");
     outfile << "step,x_exact,y_exact,yaw_exact,steeringAngle_exact,x_approx,y_approx,yaw_approx,steeringAngle_approx,error" << "\n";

     double biggest_error = 0.0;

     // Simulation loop
     for (int i = 0; i < steps; ++i)
     {
          input.steeringAngleRate = sin(i / 9);

          vehicle.updateExact(input);
          vehicle.updateApprox(input);

          double error = vehicle.calculateError();
          biggest_error = max(biggest_error, error);

          outfile << i << ","
                  << vehicle.state_exact.x << "," << vehicle.state_exact.y << "," << vehicle.state_exact.yaw << "," << vehicle.state_exact.steeringAngle << ","
                  << vehicle.state_approx.x << "," << vehicle.state_approx.y << "," << vehicle.state_approx.yaw << "," << vehicle.state_approx.steeringAngle << ","
                  << error << "\n";
     }
     outfile.close();

     // Print the overall error metric
     cout << "Largest Error: " << biggest_error << endl;

     return 0;
}
