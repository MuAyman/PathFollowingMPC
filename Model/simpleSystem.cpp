#include "osqp/osqp.h"
#include <stdlib.h>
#include <stdio.h>

int main()
{

     /* Load problem data */
     OSQPFloat P_x[3] = {
         4.0,
         1.0,
         2.0,
     };
     OSQPInt P_nnz = 3;
     OSQPInt P_i[3] = {
         0,
         0,
         1,
     };
     OSQPInt P_p[3] = {
         0,
         1,
         3,
     };
     OSQPFloat q[2] = {
         1.0,
         1.0,
     };
     OSQPFloat A_x[4] = {
         1.0,
         1.0,
         1.0,
         1.0,
     };
     OSQPInt A_nnz = 4;
     OSQPInt A_i[4] = {
         0,
         1,
         0,
         2,
     };
     OSQPInt A_p[3] = {
         0,
         2,
         4,
     };
     OSQPFloat l[3] = {
         1.0,
         0.0,
         0.0,
     };
     OSQPFloat u[3] = {
         1.0,
         0.7,
         0.7,
     };
     OSQPInt n = 2;
     OSQPInt m = 3;

     /* Exitflag */
     OSQPInt exitflag = 0;

     /* Solver, settings, matrices */
     OSQPSolver *solver = NULL;
     OSQPSettings *settings = NULL;
     OSQPCscMatrix P, A; // OSQP matrices for problem setup

     P.n = n;      // Number of variables
     P.m = n;      // Number of variables (same as P.n for quadratic term)
     P.nz = P_nnz; // Number of non-zero elements
     P.p = P_p;    // Column pointers
     P.i = P_i;    // Row indices
     P.x = P_x;    // Values

     A.n = n;      // Number of variables
     A.m = m;      // Number of constraints
     A.nz = A_nnz; // Number of non-zero elements
     A.p = A_p;    // Column pointers
     A.i = A_i;    // Row indices
     A.x = A_x;    // Values

     /* Set default settings */
     settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
     if (settings)
     {
          osqp_set_default_settings(settings);
          settings->polishing = 1;

          // settings->linsys_solver = OSQP_DIRECT_SOLVER;
          // settings->linsys_solver = OSQP_INDIRECT_SOLVER;
     }

     /* Setup solver */
     exitflag = osqp_setup(&solver, &P, q, &A, l, u, m, n, settings);

     if (exitflag)
     {
          printf("  OSQP errored during setup: %s\n", osqp_error_message(exitflag));
          return exitflag;
     }

     /* Test codegen */
     OSQPCodegenDefines *defs = (OSQPCodegenDefines *)calloc(1, sizeof(OSQPCodegenDefines));

     /* Get the default codegen options */
     osqp_set_default_codegen_defines(defs);

     if (exitflag)
     {
          printf("  OSQP errored during vector code genreation: %s\n", osqp_error_message(exitflag));
          return exitflag;
     }

     if (exitflag)
     {
          printf("  OSQP errored during matrix code genreation: %s\n", osqp_error_message(exitflag));
          return exitflag;
     }

     /* Solve problem */
     exitflag = osqp_solve(solver);

     if (exitflag)
     {
          printf("  OSQP errored during solve: %s\n", osqp_error_message(exitflag));
          return exitflag;
     }

     /* Cleanup */
     osqp_cleanup(solver);

     if (settings)
          free(settings);

     return (int)exitflag;
}
