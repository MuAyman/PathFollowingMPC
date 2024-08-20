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
};

struct ControlInput
{
     double steeringAngleRate = 0; // rad/second
     double velocity = 0;          // m/s
};

class Vehicle
{
public:
     States state;
     double L;            // wheelbase of the vehicle in m
     double wheel_radius; // wheel radius in m
     double dt;           // time step in seconds

     // Constructors
     Vehicle(double dt) : L(2.0), wheel_radius(0.2), dt(dt)
     {
     }

     Vehicle(double _L, double _wheel_radius, double dt) : L(_L), wheel_radius(_wheel_radius), dt(dt)
     {
     }

     void update(ControlInput input) // alpha & v
     {
          // Update vehicle states
          state.steeringAngle += input.steeringAngleRate * dt;
          state.yaw += input.velocity / L * tan(state.steeringAngle) * dt;
          state.y += input.velocity * sin(state.yaw) * dt;
          state.x += input.velocity * cos(state.yaw) * dt;
     }

     void simulate(int steps, ControlInput input)
     {
          std::ofstream outfile("trajectory.csv");
          outfile << "x,y,yaw,steeringAngle" << "\n";
          for (int i = 0; i < steps; ++i)
          {
               update(input);
               outfile << state.x << "," << state.y << "," << state.yaw << "," << state.steeringAngle << "\n";

               // // Print vehicle states every step
               // cout << "\nStep " << i;
               // cout << ", x: " << state.x << ", y: " << state.y << ", yaw: "
               //      << state.yaw << ", steeringAngle: " << state.steeringAngle << "\n";
          }
          outfile.close();
     }
};

int main()
{
     double dt = 0.1; // seconds
     int steps = 100;

     Vehicle vehicle(dt);
     vehicle.simulate(steps, {0.1, 5});

     return 0;
}
