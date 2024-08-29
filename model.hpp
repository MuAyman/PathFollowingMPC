#include <iostream>
#include <cmath>

const double timestep = 0.1; // time step in seconds

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
     States prevS;

     // Constructors
     Vehicle() : L(2.0), dt(timestep) {}

     Vehicle(double dt) : L(2.0), dt(dt) {}

     Vehicle(double _L, double dt) : L(_L), dt(dt) {}

     States dynamics(const ControlInputs input) const
     {
          States predictedState;

          // Store previous states for derivative calculations
          States prevS = state;

          // Update vehicle states
          predictedState.setsteeringAngle(prevS.steeringAngle + input.steeringAngleRate * dt);
          predictedState.yaw = prevS.yaw + (input.velocity / L * tan(prevS.steeringAngle) * dt);
          predictedState.y = prevS.y + (input.velocity * sin(prevS.yaw) * dt);
          predictedState.x = prevS.x + (input.velocity * cos(prevS.yaw) * dt);

          return predictedState;
     }

     void update(const ControlInputs input)
     {
          // Update vehicle states
          prevI = input;
          prevS = state;
          state = dynamics(prevI);

          // update previous control input values
     }
};