#define HAVE_CSTDDEF
#include <iostream>                    // Include the standard I/O library for printing output
#include <vector>                      // Include the vector library for dynamic array management
#include "coin/IpIpoptApplication.hpp" // Include IPOPT application-specific classes and methods
#include "coin/IpTNLP.hpp"             // Include the TNLP (nonlinear programming problem) base class from IPOPT
#include <cmath>                       // Include the cmath library for mathematical functions like pow and cos

using namespace Ipopt; // Use the Ipopt namespace for convenience, so you don't have to prefix Ipopt classes with 'Ipopt::'

static int iterationNumber = 0;

// Define the MPC problem class inheriting from IPOPT's TNLP class
class BicycleMPC : public TNLP
{
public:
     BicycleMPC() {}          // Constructor (empty for now)
     virtual ~BicycleMPC() {} // Destructor (also empty)

     // Method to specify the structure of the problem (number of variables, constraints, etc.)
     virtual bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag, IndexStyleEnum &index_style) override
     {
          n = 4 * N + 2 * N;           // Total number of variables (4 states and 2 inputs per time step over N steps)
          m = 4 * N;                   // Number of constraints (4 dynamic equations per time step over N steps)
          nnz_jac_g = 4 * N * 6;       // Number of non-zero elements in the Jacobian matrix (each constraint has 6 non-zero elements)
          nnz_h_lag = n;               // Number of non-zero elements in the Hessian (assuming a diagonal structure for simplicity)
          index_style = TNLP::C_STYLE; // Specify the indexing style (C-style: 0-based index)
          return true;
     }

     // Method to specify bounds for variables and constraints
     virtual bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) override
     {
          // Loop over all state variables
          for (Index i = 0; i < 4 * N; ++i)
          {
               x_l[i] = -1e20; // No lower bound for states (effectively unbounded)
               x_u[i] = 1e20;  // No upper bound for states (effectively unbounded)
          }

          // Loop over all input variables
          for (Index i = 4 * N; i < n; ++i)
          {
               if ((i - 4 * N) % 2 == 0)
               {
                    x_l[i] = v_min; // Lower bound for velocity
                    x_u[i] = v_max; // Upper bound for velocity
               }
               else
               {
                    x_l[i] = -alpha_max; // Lower bound for steering angle
                    x_u[i] = alpha_max;  // Upper bound for steering angle
               }
          }

          // All constraints are equality constraints, hence the lower and upper bounds are zero
          for (Index i = 0; i < m; ++i)
          {
               g_l[i] = g_u[i] = 0.0;
          }

          return true;
     }

     // Method to provide an initial guess for the variables
     virtual bool get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U, Index m, bool init_lambda, Number *lambda) override
     {
          if (init_x)
          {
               // Initialize all variables with zero
               for (Index i = 0; i < n; ++i)
               {
                    x[i] = 0.0;
               }
          }
          return true;
     }

     // Method to compute the objective function value
     virtual bool eval_f(Index n, const Number *x, bool new_x, Number &obj_value) override
     {
          obj_value = 0.0; // Initialize the objective value to zero

          // Loop over all time steps
          for (Index k = 0; k < N; ++k)
          {
               Index idx_x = 4 * k;                                   // Index for state variables at time step k
               Index idx_u = 4 * N + 2 * k;                           // Index for control variables at time step k
               Number x_ref = 1.0, y_ref = 1.0, theta_ref = 0.0;      // Reference values for x, y, and theta
               Number Qe = 1.0, Qtheta = 1.0, Rv = 0.1, Ralpha = 0.1; // Weights for the cost function

               // Add the squared error for states and control inputs to the objective function
               obj_value += Qe * (pow(x[idx_x] - x_ref, 2) + pow(x[idx_x + 1] - y_ref, 2)) +
                            Qtheta * pow(x[idx_x + 2] - theta_ref, 2) +
                            Rv * pow(x[idx_u], 2) +
                            Ralpha * pow(x[idx_u + 1], 2);
          }

          return true;
     }

     // Method to compute the gradient of the objective function
     virtual bool eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) override
     {
          // Loop over all time steps
          for (Index k = 0; k < N; ++k)
          {
               Index idx_x = 4 * k;                                   // Index for state variables at time step k
               Index idx_u = 4 * N + 2 * k;                           // Index for control variables at time step k
               Number x_ref = 1.0, y_ref = 1.0, theta_ref = 0.0;      // Reference values for x, y, and theta
               Number Qe = 4.0, Qtheta = 1.0, Rv = 0.1, Ralpha = 0.1; // Weights for the gradient calculation

               // Compute the gradient for the state variables
               grad_f[idx_x] = 2.0 * Qe * (x[idx_x] - x_ref);                 // Gradient w.r.t x position
               grad_f[idx_x + 1] = 2.0 * Qe * (x[idx_x + 1] - y_ref);         // Gradient w.r.t y position
               grad_f[idx_x + 2] = 2.0 * Qtheta * (x[idx_x + 2] - theta_ref); // Gradient w.r.t theta
               grad_f[idx_x + 3] = 0.0;                                       // No gradient w.r.t steering angle

               // Compute the gradient for the control variables
               grad_f[idx_u] = 2.0 * Rv * x[idx_u];             // Gradient w.r.t velocity
               grad_f[idx_u + 1] = 2.0 * Ralpha * x[idx_u + 1]; // Gradient w.r.t steering angle rate
          }

          return true;
     }

     // Method to compute the constraint values (dynamics equations)
     virtual bool eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) override
     {
          // Loop over all time steps
          for (Index k = 0; k < N; ++k)
          {
               Index idx_x = 4 * k;         // Index for state variables at time step k
               Index idx_u = 4 * N + 2 * k; // Index for control variables at time step k

               if (k < N - 1) // Skip the last step (no next step for dynamics)
               {
                    // Compute the dynamics equations (discrete-time)
                    g[idx_x] = 2 * x[idx_x + 4] - (x[idx_x] + x[idx_u] * cos(x[idx_x + 2]));           // x position update equation
                    g[idx_x + 1] = x[idx_x + 5] - (x[idx_x + 1] + x[idx_u] * sin(x[idx_x + 2]));       // y position update equation
                    g[idx_x + 2] = x[idx_x + 6] - (x[idx_x + 2] + (x[idx_u] / L) * tan(x[idx_x + 3])); // theta update equation
                    g[idx_x + 3] = x[idx_x + 7] - (x[idx_x + 3] + x[idx_u + 1]);                       // steering angle update equation
               }
          }

          return true;
     }

     virtual bool eval_jac_g(Index n, const Number *x, bool new_x, Index m, Index nele_jac, Index *iRow, Index *jCol, Number *values)
     {
          if (values == nullptr)
          {
               // Return the structure of the Jacobian (i.e., the sparsity pattern)
               Index idx = 0;

               // g1 with respect to [x, y, theta, alpha, v]
               iRow[idx] = 0;
               jCol[idx] = 2;
               idx++; // d(g1)/d(theta)
               iRow[idx] = 0;
               jCol[idx] = 4;
               idx++; // d(g1)/d(v)

               // g2 with respect to [x, y, theta, alpha, v]
               iRow[idx] = 1;
               jCol[idx] = 2;
               idx++; // d(g2)/d(theta)
               iRow[idx] = 1;
               jCol[idx] = 4;
               idx++; // d(g2)/d(v)

               // g3 with respect to [x, y, theta, alpha, v]
               iRow[idx] = 2;
               jCol[idx] = 3;
               idx++; // d(g3)/d(alpha)
               iRow[idx] = 2;
               jCol[idx] = 4;
               idx++; // d(g3)/d(v)

               // g4 with respect to [dot_alpha]
               iRow[idx] = 3;
               jCol[idx] = 5;
               idx++; // d(g4)/d(dot_alpha)
          }
          else
          {
               // Return the values of the Jacobian
               Index idx = 0;

               // g1 partial derivatives
               values[idx++] = -x[4] * sin(x[2]); // d(g1)/d(theta)
               values[idx++] = cos(x[2]);         // d(g1)/d(v)

               // g2 partial derivatives
               values[idx++] = x[4] * cos(x[2]); // d(g2)/d(theta)
               values[idx++] = sin(x[2]);        // d(g2)/d(v)

               // g3 partial derivatives
               values[idx++] = -(x[4] / L) * (1 / pow(cos(x[3]), 2)); // d(g3)/d(alpha)
               values[idx++] = (1 / L) * tan(x[3]);                   // d(g3)/d(v)

               // g4 partial derivative (identity)
               values[idx++] = 1.0; // d(g4)/d(dot_alpha)
          }
          return true;
     }

     // Method to compute the Hessian matrix (second derivatives of the Lagrangian)
     virtual bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor,
                         Index m, const Number *lambda, bool new_lambda,
                         Index nele_hess, Index *iRow, Index *jCol, Number *values)
     {
          if (values == nullptr)
          {
               // Return the structure of the Hessian (i.e., the sparsity pattern)
               Index idx = 0;

               // Diagonal terms (assuming objective function is quadratic)
               for (Index i = 0; i < n; ++i)
               {
                    iRow[idx] = i;
                    jCol[idx] = i;
                    idx++;
               }

               // Off-diagonal terms if any (depending on cross-terms in constraints)
               // Example for simplicity, let's assume no cross terms in constraints
          }
          else
          {
               // Return the values of the Hessian
               Index idx = 0;

               // Quadratic terms for the objective
               // Assuming a simple form where Q and R are identity for illustration
               for (Index i = 0; i < n; ++i)
               {
                    values[idx++] = obj_factor; // Hessian w.r.t. x_i^2 or u_i^2
               }

               // Off-diagonal terms
               // Here you would include terms involving lambda if constraints are nonlinear and contribute to the Hessian
          }
          return true;
     }

     // Method to finalize the solution (print or store the results)
     virtual void finalize_solution(SolverReturn status, Index n, const Number *x, const Number *z_L, const Number *z_U, Index m, const Number *g, const Number *lambda, Number obj_value, const IpoptData *ip_data, IpoptCalculatedQuantities *ip_cq) override
     {
          std::cout << "Optimal solution:" << std::endl; // Print the optimal solution to the console

          for (Index i = 0; i < n; ++i)
          {
               std::cout << "x[" << i << "] = " << x[i] << std::endl; // Print each variable value
          }
     }

private:
     Index N = 10;           // Prediction horizon (number of time steps)
     Number v_min = -1.0;    // Minimum velocity
     Number v_max = 1.0;     // Maximum velocity
     Number alpha_max = 0.5; // Maximum steering angle rate
     Number L = 2.0;         // Length of the bicycle model
};

// Main function to set up and solve the MPC problem
int main()
{
     // Create an instance of the MPC problem
     SmartPtr<TNLP> mpc_problem = new BicycleMPC();

     // Create an instance of the Ipopt application
     SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

     // Initialize the Ipopt application and check for success
     ApplicationReturnStatus status = app->Initialize();
     if (status != Solve_Succeeded)
     {
          std::cout << std::endl
                    << std::endl
                    << "*** Error during initialization!" << std::endl;
          return (int)status;
     }

     // Optimize the problem using Ipopt and check for success
     status = app->OptimizeTNLP(mpc_problem);
     if (status == Solve_Succeeded)
     {
          std::cout << std::endl
                    << std::endl
                    << "*** The problem solved!" << std::endl;
     }
     else
     {
          std::cout << std::endl
                    << std::endl
                    << "*** The problem FAILED!" << std::endl;
     }

     return (int)status;
}
