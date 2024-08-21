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
     void setsteeringAngle(double angle)
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

     void update(ControlInputs input)
     {
          // Update vehicle states
          state.setsteeringAngle(state.steeringAngle + input.steeringAngleRate * dt);
          state.yaw += input.velocity / L * tan(state.steeringAngle) * dt;
          state.y += input.velocity * sin(state.yaw) * dt;
          state.x += input.velocity * cos(state.yaw) * dt;
     }
};

int main()
{
     double dt = 0.05; // seconds
     int steps = 500;

     Vehicle vehicle(dt);

     ControlInputs input = {0.1, 10};

     std::ofstream outfile("trajectory.csv");
     outfile << "x,y,yaw,steeringAngle" << "\n";

     // simulation loop
     for (int i = 0; i < steps; ++i)
     {
          input.steeringAngleRate = sin(i / 9);
          vehicle.update(input);
          outfile << vehicle.state.x << "," << vehicle.state.y << "," << vehicle.state.yaw << "," << vehicle.state.steeringAngle << "\n";
     }
     outfile.close();

     return 0;
}
