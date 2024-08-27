#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <iostream>
#include <cstdlib>

// Define the system matrices
const double dt = 0.1;
Eigen::Matrix2d A_d;
Eigen::Vector2d B_d;
Eigen::MatrixXd Q(2, 2);
Eigen::MatrixXd R(1, 1);
int N = 10; // Prediction horizon

void setup_system()
{
     // Discrete-time system matrices for a double integrator
     A_d << 1, dt,
         0, 1;
     B_d << 0.5 * dt * dt, dt;

     // Cost matrices
     Q << 1, 0,
         0, 1;
     R << 1;
}

void mpc_controller(const Eigen::Vector2d &x0, Eigen::VectorXd &u_opt)
{
     // Define the problem sizes
     int n = 2 * N;    // Number of states (x)
     int m = N;        // Number of controls (u)
     int n_eq = 2 * N; // Equality constraints

     // Build the matrices for the QP problem
     Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n + m, n + m);
     Eigen::VectorXd q = Eigen::VectorXd::Zero(n + m);
     Eigen::MatrixXd A_eq = Eigen::MatrixXd::Zero(n_eq, n + m);
     Eigen::VectorXd l_eq = Eigen::VectorXd::Zero(n_eq);
     Eigen::VectorXd u_eq = Eigen::VectorXd::Zero(n_eq);

     // Populate the QP matrices
     for (int i = 0; i < N; ++i)
     {
          P.block<2, 2>(2 * i, 2 * i) = Q;
          P(2 * N + i, 2 * N + i) = R(0, 0);

          if (i < N - 1)
          {
               A_eq.block<2, 2>(2 * (i + 1), 2 * i) = -A_d;
               A_eq.block<2, 1>(2 * (i + 1), 2 * N + i) = -B_d;
          }
     }
     A_eq.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
     l_eq.head<2>() = x0;
     u_eq.head<2>() = x0;

     // Convert Eigen matrices to raw arrays compatible with OSQP
     OSQPFloat *P_x = (OSQPFloat *)malloc((2 * 2 * N + N) * sizeof(OSQPFloat));
     OSQPInt *P_i = (OSQPInt *)malloc((2 * 2 * N + N) * sizeof(OSQPInt));
     OSQPInt *P_p = (OSQPInt *)malloc((n + m + 1) * sizeof(OSQPInt));

     OSQPFloat *q_data = (OSQPFloat *)malloc((n + m) * sizeof(OSQPFloat));

     OSQPFloat *A_x = (OSQPFloat *)malloc((2 * 2 * N) * sizeof(OSQPFloat));
     OSQPInt *A_i = (OSQPInt *)malloc((2 * 2 * N) * sizeof(OSQPInt));
     OSQPInt *A_p = (OSQPInt *)malloc((n + m + 1) * sizeof(OSQPInt));

     OSQPFloat *l_data = (OSQPFloat *)malloc(n_eq * sizeof(OSQPFloat));
     OSQPFloat *u_data = (OSQPFloat *)malloc(n_eq * sizeof(OSQPFloat));

     if (!P_x || !P_i || !P_p || !q_data || !A_x || !A_i || !A_p || !l_data || !u_data)
     {
          std::cerr << "Memory allocation failed!" << std::endl;
          return;
     }

     int idx = 0;
     int p_idx = 0;
     for (int i = 0; i < n + m; i++)
     {
          P_p[i] = idx;
          for (int j = 0; j < n + m; j++)
          {
               if (P(j, i) != 0.0)
               {
                    P_x[idx] = P(j, i);
                    P_i[idx] = j;
                    idx++;
               }
          }
     }
     P_p[n + m] = idx;

     for (int i = 0; i < n + m; i++)
     {
          q_data[i] = q(i);
     }

     idx = 0;
     for (int i = 0; i < n + m; i++)
     {
          A_p[i] = idx;
          for (int j = 0; j < n_eq; j++)
          {
               if (A_eq(j, i) != 0.0)
               {
                    A_x[idx] = A_eq(j, i);
                    A_i[idx] = j;
                    idx++;
               }
          }
     }
     A_p[n + m] = idx;

     for (int i = 0; i < n_eq; i++)
     {
          l_data[i] = l_eq(i);
          u_data[i] = u_eq(i);
     }

     // Setup OSQP matrices
     OSQPCscMatrix P_matrix, A_matrix;

     P_matrix.n = n + m;
     P_matrix.m = n + m;
     P_matrix.nz = idx;
     P_matrix.p = P_p;
     P_matrix.i = P_i;
     P_matrix.x = P_x;

     A_matrix.n = n + m;
     A_matrix.m = n_eq;
     A_matrix.nz = idx;
     A_matrix.p = A_p;
     A_matrix.i = A_i;
     A_matrix.x = A_x;

     // Setup OSQP settings
     OSQPSettings settings;
     osqp_set_default_settings(&settings);
     settings.alpha = 1.0;

     // Initialize the solver
     OSQPSolver *solver = nullptr;
     OSQPInt exitflag = osqp_setup(&solver, &P_matrix, q_data, &A_matrix, l_data, u_data, n_eq, n + m, &settings);
     if (exitflag != 0)
     {
          std::cerr << "OSQP setup failed!" << std::endl;
          return;
     }

     // Solve the problem
     osqp_solve(solver);

     // Extract the control input
     u_opt = Eigen::VectorXd::Zero(N);
     for (int i = 0; i < N; i++)
     {
          u_opt(i) = solver->solution->x[2 * N + i];
     }

     // Output the first control input
     std::cout << "Optimal control input: " << u_opt(0) << std::endl;

     // Cleanup
     free(P_x);
     free(P_i);
     free(P_p);
     free(q_data);
     free(A_x);
     free(A_i);
     free(A_p);
     free(l_data);
     free(u_data);

     osqp_cleanup(solver);
}

int main()
{
     // Initial state
     Eigen::Vector2d x0(0, 0); // Initial position and velocity

     // Setup the system
     setup_system();

     // Solve the MPC problem
     Eigen::VectorXd u_opt(N);
     mpc_controller(x0, u_opt);

     return 0;
}
