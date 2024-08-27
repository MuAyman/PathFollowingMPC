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
     double velocity = 0;          // m/s
     double steeringAngleRate = 0; // rad/second
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

int main()
{
     double dt = 0.1; // seconds
     int steps = 1000;

     Vehicle vehicle(dt);

     ControlInputs input = {0.49505, 1.807e-13};

     std::ofstream outfile("trajectory.csv");
     outfile << "x,y,yaw,steeringAngle" << "\n";

     // simulation loop
     for (int i = 0; i < steps; ++i)
     {
          input.steeringAngleRate = sin(i / 9);
          vehicle.update(input);
          outfile << vehicle.state.x << "," << vehicle.state.y << "," << vehicle.state.yaw << "," << vehicle.state.steeringAngle << "\n";

          // // Print vehicle states every step
          // cout << "\nStep " << i;
          // cout << ", x: " << vehicle.state.x << ", y: " << vehicle.state.y << ", yaw: "
          //      << vehicle.state.yaw << ", steeringAngle: " << vehicle.state.steeringAngle << "\n";
     }
     outfile.close();

     // vehicle.simulate(steps, {0.1, 5});

     return 0;
}
