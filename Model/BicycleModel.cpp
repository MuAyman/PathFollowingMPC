#include <iostream>    // Include standard input/output stream library for console I/O
#include <Eigen/Dense> // Include Eigen library for matrix operations
#include <vector>      // Include vector library for dynamic arrays
// #include <osqp/osqp.h> // Include OSQP header for the quadratic programming solver

using namespace Eigen; // Use Eigen namespace to simplify Eigen type names
using namespace std;   // Use standard namespace to simplify standard type names

int main()
{
     // Define state-space matrices using Eigen library
     MatrixXd A_eigen(4, 4); // A 4x4 matrix A_eigen for state transition
     MatrixXd B_eigen(4, 2); // A 4x2 matrix B_eigen for control input
     MatrixXd C_eigen(1, 4); // A 1x4 matrix C_eigen for output

     // Initialize the matrices with example values
     A_eigen << 1, 0.1, 0, 0, // Fill A_eigen with example data
         0, 1, 0, 0,
         0, 0, 1, 0.1,
         0, 0, 0, 1;

     B_eigen << 0, 0, // Fill B_eigen with example data
         0.1, 0,
         0, 0,
         0, 0.1;

     C_eigen << 1, 0, 0, 0, // Fill C_eigen with example data
         0, 1, 0, 0;
}