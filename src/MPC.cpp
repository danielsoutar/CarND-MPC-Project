#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;

size_t delta_start = epsi_start + N-1;
size_t acc_start = delta_start + N-1;


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Reference velocity - as fast as we can go?
const double reference_velocity = 65.0;

// Weights for the cost terms
const double cost_cte_sq = 2100;
const double cost_epsi_sq = 2100;
const double cost_vref_sq = 1;

const double cost_del_sq = 100;
const double cost_acc_sq = 100;

const double cost_del_del_sq = 40;
const double cost_acc_del_sq = 10;


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    fg[0] = 0.0;

    // Compute cost function...
    for(int t = 0; t < N; ++t) {
        fg[0] += cost_cte_sq * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += cost_epsi_sq * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += cost_vref_sq * CppAD::pow(vars[v_start + t] - reference_velocity, 2);
    }

    for(int t = 0; t < N - 1; ++t) {
        fg[0] += cost_del_sq * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += cost_acc_sq * CppAD::pow(vars[acc_start + t], 2);
    }

    for (int t = 0; t < N - 2; ++t) {
        fg[0] += cost_del_del_sq * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += cost_acc_del_sq * CppAD::pow(vars[acc_start + t + 1] - vars[acc_start + t], 2);
    }

    // Set up constraints (initial vals) ...
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    AD<double> x1;
    AD<double> y1;
    AD<double> psi1;
    AD<double> v1;
    AD<double> cte1;
    AD<double> epsi1;
    
    AD<double> x0;
    AD<double> y0;
    AD<double> psi0;
    AD<double> v0;
    AD<double> cte0;
    AD<double> epsi0;

    AD<double> delta0;
    AD<double> acc0;

    AD<double> f0;
    AD<double> psi_desired;

    // Loop through timesteps (bar timestep 0, the initial state) and specify constraints
    for (int t = 1; t < N; ++t) {
        // Unpleasant duplication, but avoids creating a new AD<double> each time,
        // reuses it instead.
        x1 = vars[x_start + t];
        y1 = vars[y_start + t];
        psi1 = vars[psi_start + t];
        v1 = vars[v_start + t];
        cte1 = vars[cte_start + t];
        epsi1 = vars[epsi_start + t];

        x0 = vars[x_start + t - 1];
        y0 = vars[y_start + t - 1];
        psi0 = vars[psi_start + t - 1];
        v0 = vars[v_start + t - 1];
        cte0 = vars[cte_start + t - 1];
        epsi0 = vars[epsi_start + t - 1];

        delta0 = vars[delta_start + t - 1];
        acc0 = vars[acc_start + t - 1];

        // f0 = polyeval(coeffs, x0);
        if(coeffs.size() == 4) {  // third-order, a quadratic derivative (ax^3 + bx^2 + cx + d) => 3ax^2 + 2bx + c
            f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * (x0 * x0) + coeffs[3] * (x0 * x0 * x0);
            psi_desired = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * (x0 * x0)); 
        }
        else if(coeffs.size() == 3) {  // second-order, a linear derivative (ax^2 + bx + c) => 2ax + b
            f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * (x0 * x0);
            psi_desired = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);
        }
        else if(coeffs.size() == 2) {  // first-order, a constant derivative (ax + b) => a
            f0 = coeffs[0] + coeffs[1] * x0;
            psi_desired = CppAD::atan(coeffs[1]);
        }

        // x1 - (x0 + v0 * cos(psi0) * dt) == 0;
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        // y1 - (y0 + v0 * sin(psi0) * dt) == 0
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        // psi1 - (psi0 + (v / Lf) * delta0 * dt) == 0
        fg[1 + psi_start + t] = psi1 - (psi0 - (v0 / Lf) * delta0 * dt);
        // v1 - (v0 * acc0 * dt) == 0
        fg[1 + v_start + t] = v1 - (v0 + acc0 * dt);

        // cte1 - ((f0 - y0) + v0 * sin(epsi0) * dt) == 0
        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
        // epsi1 - ((psi0 - psi_desired) + (v0 / Lf) * delta0 * dt) == 0
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_desired) - (v0 / Lf) * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Set the number of model variables (includes both states and inputs).
    // We have x, y, psi, v, cte, and epsi as our state (6 vars), and delta,
    // acceleration (2 vars). So we have:
    size_t n_vars = N * state_size + (N - 1) * actuators_size;
    // Set the number of constraints - we are constraining our state
    // variables (state_size) multiplied by the no. of timesteps (N). 
    size_t n_constraints = N * state_size;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set lower and upper limits for variables.
    for (int i = 0; i < delta_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    for (int i = delta_start; i < acc_start; ++i) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    for (int i = acc_start; i < n_vars; ++i) {
        vars_lowerbound[i] = -1;
        vars_upperbound[i] = 1;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    std::vector<double> res;

    res.push_back(solution.x[delta_start]);
    res.push_back(solution.x[acc_start]);

    for (int t = 0; t < N; ++t) {
        res.push_back(solution.x[x_start + t]);
        res.push_back(solution.x[y_start + t]);
    }

    return res;
}









