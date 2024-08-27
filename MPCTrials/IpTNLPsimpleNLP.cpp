#define HAVE_CSTDDEF
#include <iostream>
#include <vector>
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpTNLP.hpp"
#include <cmath>

#undef HAVE_CSTDDEF

using namespace Ipopt;

// Nonlinear MPC Problem definition
class MyNLP : public TNLP
{
public:
    MyNLP() {}
    virtual ~MyNLP() {}

    virtual bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag,
                              IndexStyleEnum &index_style) override
    {
        n = 4;                       // Number of variables
        m = 0;                       // Number of constraints
        nnz_jac_g = 0;               // Jacobian is empty since there are no constraints
        nnz_h_lag = 10;              // Number of non-zero elements in the Hessian (lower triangular)
        index_style = TNLP::C_STYLE; // Use C-style indexing (0-based)
        return true;
    }

    virtual bool get_bounds_info(Index n, Number *x_l, Number *x_u,
                                 Index m, Number *g_l, Number *g_u) override
    {
        // Setting bounds on the variables
        for (Index i = 0; i < n; ++i)
        {
            x_l[i] = 7; // effectively making them unbounded.
            x_u[i] = 1e20;
        }
        return true;
    }

    virtual bool get_starting_point(Index n, bool init_x, Number *x,
                                    bool init_z, Number *z_L, Number *z_U,
                                    Index m, bool init_lambda,
                                    Number *lambda) override
    {
        if (init_x)
        {
            for (Index i = 0; i < n; ++i)
            {
                x[i] = 8; // Initial guess for variables
            }
        }
        return true;
    }

    virtual bool eval_f(Index n, const Number *x, bool new_x, Number &obj_value) override
    {
        obj_value = 0.0;
        for (Index i = 0; i < n; ++i)
        {
            obj_value += x[i] * x[i]; // Simple quadratic objective
        }
        return true;
    }

    virtual bool eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) override
    {
        for (Index i = 0; i < n; ++i)
        {
            grad_f[i] = 2.0 * x[i]; // Gradient of the quadratic objective
        }
        return true;
    }

    virtual bool eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) override
    {
        // No constraints to evaluate
        return true;
    }

    virtual bool eval_jac_g(Index n, const Number *x, bool new_x,
                            Index m, Index nele_jac, Index *iRow, Index *jCol,
                            Number *values) override
    {
        // No constraints, no Jacobian
        return true;
    }

    virtual bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m,
                        const Number *lambda, bool new_lambda, Index nele_hess,
                        Index *iRow, Index *jCol, Number *values) override
    {
        if (values == nullptr)
        {
            // Structure of the Hessian (lower triangular part)
            Index idx = 0;
            for (Index i = 0; i < n; ++i)
            {
                for (Index j = 0; j <= i; ++j)
                {
                    iRow[idx] = i;
                    jCol[idx] = j;
                    ++idx;
                }
            }
        }
        else
        {
            // Values of the Hessian
            for (Index idx = 0; idx < nele_hess; ++idx)
            {
                values[idx] = obj_factor * 2.0; // Assuming a simple quadratic form for the objective
            }
        }
        return true;
    }

    virtual void finalize_solution(SolverReturn status,
                                   Index n, const Number *x, const Number *z_L, const Number *z_U,
                                   Index m, const Number *g, const Number *lambda,
                                   Number obj_value,
                                   const IpoptData *ip_data,
                                   IpoptCalculatedQuantities *ip_cq) override
    {
        std::cout << "Solution found!\n";
        for (Index i = 0; i < n; ++i)
        {
            std::cout << "x[" << i << "] = " << x[i] << std::endl;
        }
        std::cout << "Objective value = " << obj_value << std::endl;
    }
};

int main()
{
    // Create an instance of the IpoptApplication
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status = app->Initialize();
    if (status != Solve_Succeeded)
    {
        std::cout << "Error during initialization!\n";
        return (int)status;
    }

    // Create an instance of your NLP
    SmartPtr<TNLP> mynlp = new MyNLP();

    // Optimize the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Solve_Succeeded)
    {
        std::cout << "\n\n*** The problem solved successfully!\n";
    }
    else
    {
        std::cout << "\n\n*** The problem failed!\n";
    }

    return (int)status;
}
