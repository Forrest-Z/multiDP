#include <stdio.h>
#include "osqp.h"
void basc_qp_test() {
  // Load problem data
  c_float P_x[4] = {
      2.0,
      0.2,
      -1.0,
      2.0,
  };
  c_int P_nnz = 4;
  c_int P_i[4] = {
      0,
      1,
      0,
      2,
  };
  c_int P_p[4] = {
      0,
      1,
      2,
      4,
  };
  c_float q[3] = {
      0.0,
      -1.0,
      0.0,
  };
  c_float A_x[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  c_int A_nnz = 6;
  c_int A_i[6] = {
      0, 1, 0, 2, 0, 3,
  };
  c_int A_p[4] = {
      0,
      2,
      4,
      6,
  };
  c_float l[4] = {1.0, 0.0, 0.0, 0.0};
  c_float u[4] = {
      OSQP_INFTY,
      OSQP_INFTY,
      OSQP_INFTY,
      OSQP_INFTY,
  };
  c_int n = 3;
  c_int m = 4;

  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
  }

  // Define solver settings as default
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.0;  // Change alpha parameter
  }

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);
  if (exitflag != 0) printf("setup error\n");
  // Solve Problem
  osqp_solve(work);
  // solution
  OSQPSolution *sol_data = work->solution;
  switch (work->info->status_val) {
    case OSQP_SOLVED:
      // print solution
      printf("Optimal primal solution: \n");
      for (int i = 0; i != 3; ++i) printf("x[%d]: %f\n", i, sol_data->x[i]);
      break;
    case OSQP_SOLVED_INACCURATE:
      printf("SOLVED_INACCURATE\n");
      break;
    case OSQP_MAX_ITER_REACHED:
      printf("OSQP_MAX_ITER_REACHED\n");
      break;
    case OSQP_PRIMAL_INFEASIBLE:
      printf("OSQP_PRIMAL_INFEASIBLE\n");
      break;
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
      printf("OSQP_PRIMAL_INFEASIBLE_INACCURATE\n");
      break;
    case OSQP_DUAL_INFEASIBLE:
      printf("OSQP_DUAL_INFEASIBLE\n");
      break;
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      printf("OSQP_DUAL_INFEASIBLE_INACCURATE\n");
      break;
    case OSQP_SIGINT:
      printf("OSQP_SIGINT\n");
      break;
    case OSQP_TIME_LIMIT_REACHED:
      printf("OSQP_TIME_LIMIT_REACHED\n");
      break;
    case OSQP_UNSOLVED:
      printf("OSQP_UNSOLVED\n");
      break;
    case OSQP_NON_CVX:
      printf("OSQP_NON_CVX\n");
      break;
    default:
      printf("unknown\n");
      break;
  }

  // Clean workspace
  osqp_cleanup(work);

  // Cleanup
  if (data) {
    if (data->P) c_free(data->P);
    if (data->A) c_free(data->A);
    c_free(data);
  }
  if (settings) c_free(settings);
}  // basc_qp_test

void update_qp_test() {
  constexpr c_int X_n = 3;    // dimension of variable (n*1)
  constexpr c_int P_nnz = 4;  // # of non-zero elements in P
  constexpr c_int A_nnz = 6;  // # of non-zero elements in A
  constexpr c_int m = 4;      // dimension of A (m*n)

  // Load problem data
  c_float P_x[P_nnz] = {2.0, 0.2, -1.0, 2.0};
  c_int P_i[P_nnz] = {0, 1, 0, 2};
  c_int P_p[X_n + 1] = {0, 1, 2, 4};
  c_float q[X_n] = {0.0, -1.0, 0.0};
  c_float A_x[A_nnz] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  c_int A_i[A_nnz] = {0, 1, 0, 2, 0, 3};
  c_int A_p[X_n + 1] = {0, 2, 4, 6};
  c_float l[m] = {1.0, 0.0, 0.0, 0.0};
  c_float u[m] = {OSQP_INFTY, OSQP_INFTY, OSQP_INFTY, OSQP_INFTY};

  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    data->n = X_n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
  }

  // Define solver settings as default
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.0;  // Change alpha parameter
  }

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  // Solve Problem
  osqp_solve(work);

  // solution
  OSQPSolution *sol_data = work->solution;
  printf("Optimal primal solution: \n");
  for (int i = 0; i != X_n; ++i) printf("x[%d]: %f\n", i, sol_data->x[i]);
  printf("\n");

  // Update problem
  c_float P_x_new[P_nnz] = {1.0, 2.0, -1.0, 4.0};
  c_float q_new[X_n] = {2.0, -3.0, 1.0};
  c_float A_x_new[A_nnz] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  c_float l_new[m] = {0.5, 0.0, 0.0, 0.0};
  c_float u_new[m] = {0.5, 1.0, 1.0, 1.0};

  osqp_update_P(work, P_x_new, OSQP_NULL,
                P_nnz);  // Update only upper triangular part of P
  osqp_update_A(work, A_x_new, OSQP_NULL, A_nnz);
  osqp_update_lin_cost(work, q_new);
  osqp_update_bounds(work, l_new, u_new);

  // Solve Problem again
  osqp_solve(work);

  // solution
  OSQPSolution *new_sol_data = work->solution;
  printf("New Optimal primal solution: \n");
  for (int i = 0; i != X_n; ++i) printf("x[%d]: %f\n", i, new_sol_data->x[i]);
  printf("\n");

  // Clean workspace
  osqp_cleanup(work);

  // Cleanup
  if (data) {
    if (data->P) c_free(data->P);
    if (data->A) c_free(data->A);
    c_free(data);
  }
  if (settings) c_free(settings);
}  // update_qp_test

int main() {
  basc_qp_test();
  // update_qp_test();
}