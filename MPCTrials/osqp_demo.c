// #include <Eigen/Dense>
// #include "Eigen/Sparse"
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include <osqp/osqp.h>
#include <vector>
#include <iostream>

int main()
{
  // Discrete time model of a quadcopter
  Eigen::SparseMatrix<double> Ad(12, 12);
  Ad.insert(0, 0) = 1.0;
  Ad.insert(0, 6) = 0.1;
  Ad.insert(1, 1) = 1.0;
  Ad.insert(1, 7) = 0.1;
  Ad.insert(2, 2) = 1.0;
  Ad.insert(2, 8) = 0.1;
  Ad.insert(3, 0) = 0.0488;
  Ad.insert(3, 3) = 1.0;
  Ad.insert(3, 6) = 0.0016;
  Ad.insert(3, 9) = 0.0992;
  Ad.insert(4, 1) = -0.0488;
  Ad.insert(4, 4) = 1.0;
  Ad.insert(4, 7) = -0.0016;
  Ad.insert(4, 10) = 0.0992;
  Ad.insert(5, 5) = 1.0;
  Ad.insert(5, 11) = 0.0992;
  Ad.insert(6, 6) = 1.0;
  Ad.insert(7, 7) = 1.0;
  Ad.insert(8, 8) = 1.0;
  Ad.insert(9, 0) = 0.9734;
  Ad.insert(9, 6) = 0.0488;
  Ad.insert(9, 9) = 0.9846;
  Ad.insert(10, 1) = -0.9734;
  Ad.insert(10, 7) = -0.0488;
  Ad.insert(10, 10) = 0.9846;
  Ad.insert(11, 11) = 0.9846;

  Eigen::SparseMatrix<double> Bd(12, 4);
  Bd.insert(0, 1) = -0.0726;
  Bd.insert(0, 3) = 0.0726;
  Bd.insert(1, 0) = -0.0726;
  Bd.insert(1, 2) = 0.0726;
  Bd.insert(2, 0) = -0.0152;
  Bd.insert(2, 1) = 0.0152;
  Bd.insert(2, 2) = -0.0152;
  Bd.insert(2, 3) = 0.0152;
  Bd.insert(3, 3) = 0.0006;
  Bd.insert(4, 0) = 0.0006;
  Bd.insert(4, 2) = -0.0006;
  Bd.insert(5, 0) = 0.0106;
  Bd.insert(5, 1) = 0.0106;
  Bd.insert(5, 2) = 0.0106;
  Bd.insert(5, 3) = 0.0106;
  Bd.insert(6, 1) = -1.4512;
  Bd.insert(6, 3) = 1.4512;
  Bd.insert(7, 0) = -1.4512;
  Bd.insert(7, 2) = 1.4512;
  Bd.insert(8, 0) = -0.3049;
  Bd.insert(8, 1) = 0.3049;
  Bd.insert(8, 2) = -0.3049;
  Bd.insert(8, 3) = 0.3049;
  Bd.insert(9, 3) = 0.0236;
  Bd.insert(10, 0) = 0.0236;
  Bd.insert(10, 2) = -0.0236;
  Bd.insert(11, 0) = 0.2107;
  Bd.insert(11, 1) = 0.2107;
  Bd.insert(11, 2) = 0.2107;
  Bd.insert(11, 3) = 0.2107;

  int nx = Bd.rows(), nu = Bd.cols();

  // Constraints
  double u0 = 10.5916;
  Eigen::Vector4d umin = Eigen::Vector4d(9.6, 9.6, 9.6, 9.6) - Eigen::Vector4d::Ones() * u0;
  Eigen::Vector4d umax = Eigen::Vector4d(13., 13., 13., 13.) - Eigen::Vector4d::Ones() * u0;
  Eigen::VectorXd xmin(12), xmax(12);
  xmin << -M_PI / 6, -M_PI / 6, -INFINITY, -INFINITY, -INFINITY, -1., -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY;
  xmax << M_PI / 6, M_PI / 6, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY;

  // Objective function
  Eigen::SparseMatrix<double> Q(12, 12);
  Q.insert(2, 2) = 10.0;
  Q.insert(3, 3) = 10.0;
  Q.insert(4, 4) = 10.0;
  Q.insert(5, 5) = 10.0;
  Q.insert(9, 9) = 5.0;
  Q.insert(10, 10) = 5.0;
  Q.insert(11, 11) = 5.0;
  Eigen::SparseMatrix<double> QN = Q;
  Eigen::SparseMatrix<double> R = 0.1 * Eigen::MatrixXd::Identity(4, 4).sparseView();

  // Initial and reference states
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd xr(12);
  xr << 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.;

  // Prediction horizon
  int N = 10;

  // Cast MPC problem to a QP
  Eigen::SparseMatrix<double> P = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(N, N), Q).sparseView();
  P = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(N + 1, N + 1), Q).sparseView();
  Eigen::VectorXd q = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(N), -Q * xr);
  q.conservativeResize(q.size() + 12);
  q.tail(12) = -QN * xr;

  // Linear dynamics
  Eigen::SparseMatrix<double> Ax = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(N + 1, N + 1), -Eigen::MatrixXd::Identity(nx, nx)).sparseView();
  Ax += Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(N + 1, N + 1, -1), Ad).sparseView();
  Eigen::SparseMatrix<double> Bu = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(N, N), Bd).sparseView();
  Bu.conservativeResize(Ax.rows(), Ax.cols());

  Eigen::SparseMatrix<double> Aeq = Eigen::hstack({Ax, Bu});
  Eigen::VectorXd leq = -x0;
  leq.conservativeResize(Aeq.rows());
  Eigen::VectorXd ueq = leq;

  // Input and state constraints
  Eigen::SparseMatrix<double> Aineq = Eigen::MatrixXd::Identity((N + 1) * nx + N * nu, (N + 1) * nx + N * nu).sparseView();
  Eigen::VectorXd lineq = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(N + 1), xmin);
  Eigen::VectorXd uineq = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(N + 1), xmax);

  // OSQP constraints
  Eigen::SparseMatrix<double> A = Eigen::vstack({Aeq, Aineq});
  Eigen::VectorXd l = leq;
  l.conservativeResize(A.rows());
  Eigen::VectorXd u = ueq;
  u.conservativeResize(A.rows());

  // Create an OSQP object
  OSQPWorkspace *work;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  data->n = P.cols();
  data->m = A.rows();
  data->P = csc_matrix(P.rows(), P.cols(), P.nonZeros(), P.valuePtr(), P.innerIndexPtr(), P.outerIndexPtr());
  data->q = q.data();
  data->A = csc_matrix(A.rows(), A.cols(), A.nonZeros(), A.valuePtr(), A.innerIndexPtr(), A.outerIndexPtr());
  data->l = l.data();
  data->u = u.data();

  // Define settings
  osqp_set_default_settings(settings);
  settings->warm_start = 1;

  // Setup workspace
  work = osqp_setup(data, settings);

  // Simulate in closed loop
  int nsim = 15;
  for (int i = 0; i < nsim; ++i)
  {
    // Solve
    osqp_solve(work);

    // Check solver status
    if (work->info->status_val != OSQP_SOLVED)
    {
      throw std::runtime_error("OSQP did not solve the problem!");
    }

    // Apply first control input to the plant
    Eigen::VectorXd ctrl = Eigen::Map<Eigen::VectorXd>(work->solution->x + N * nx, nu);
    x0 = Ad * x0 + Bd * ctrl;

    // Update initial state
    l.head(nx) = -x0;
    u.head(nx) = -x0;
    osqp_update_lin_cost(work, q.data());
    osqp_update_bounds(work, l.data(), u.data());
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return 0;
}
