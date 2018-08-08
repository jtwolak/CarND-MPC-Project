#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

/* 
 * Set the timestep length and duration
 */
size_t N = MPC_SIZE_N;
double dt = MPC_DT;

double latency = 0.1;

/* 
 * Set the length from front to CoG that has a similar radius.
 */
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
double ref_v = MPC_SPEED;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
	
	/*
	 * Setup Constraints
	 */

	/* Cost function */
	for (unsigned int t = 0; t < N; t++) 
	{
		fg[0] += CppAD::pow(vars[cte_start + t], 2);
		fg[0] += CppAD::pow(vars[epsi_start + t], 2);
		fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
	}

	/* Minimize the use of actuators. */
	for (unsigned int t = 0; t < N - 1; t++) 
	{
		fg[0] += CppAD::pow(vars[delta_start + t], 2);
		fg[0] += CppAD::pow(vars[a_start + t], 2);
	}

	/* Minimize the value gap between sequential actuations. */
	for (unsigned int t = 0; t < N - 2; t++) 
	{
		fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}

	/* Initial constraints */
	fg[1 + x_start] = vars[x_start];
	fg[1 + y_start] = vars[y_start];
	fg[1 + psi_start] = vars[psi_start];
	fg[1 + v_start] = vars[v_start];
	fg[1 + cte_start] = vars[cte_start];
	fg[1 + epsi_start] = vars[epsi_start];

	/* The rest of the constraints */
	for (unsigned int t = 1; t < N; t++)
	{
		/* The state at time t+1 . */
		AD<double> x1 = vars[x_start + t];
		AD<double> y1 = vars[y_start + t];
		AD<double> psi1 = vars[psi_start + t];
		AD<double> v1 = vars[v_start + t];
		AD<double> cte1 = vars[cte_start + t];
		AD<double> epsi1 = vars[epsi_start + t];

		/* The state at time t. */
		AD<double> x0 = vars[x_start + t - 1];
		AD<double> y0 = vars[y_start + t - 1];
		AD<double> psi0 = vars[psi_start + t - 1];
		AD<double> v0 = vars[v_start + t - 1];
		AD<double> cte0 = vars[cte_start + t - 1];
		AD<double> epsi0 = vars[epsi_start + t - 1];

		/* Only consider the actuation at time t. */
		AD<double> delta0 = vars[delta_start + t - 1];
		AD<double> a0 = vars[a_start + t - 1];

		AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 + coeffs[3] * x0*x0*x0;
		AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0*x0);


		/*
		 * Model Equations:
		 *
		 * x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
		 * y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
		 * psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
		 * v_[t] = v[t-1] + a[t-1] * dt
		 * cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
		 * epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
		 */
		fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
		fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
		fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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

  //definie the timestep to constraint and variables to save actuator values
  size_t fixed_steps = latency / dt;
  static double prev_a = 0;
  static double prev_delta = 0;

  /* 
   * Set the number of model variables (includes both states and inputs).
   */
  size_t n_vars = N * 6 + (N - 1) * 2;

  /*
   * Set the number of constraints
   */
  size_t n_constraints = N * 6;

  /*
   * Initial value of the independent variables.
   * SHOULD BE 0 besides initial state.
   */
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) 
  {
	vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for state
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  /*
   * Set all non-actuators upper and lowerlimits
   * to the max negative and positive values.
   */
  for (unsigned int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  /* The upper and lower limits of delta are set to -25 and 25
   * degrees (values in radians).
   */
  for (unsigned int i = delta_start; i < a_start; i++) 
  {
	  vars_lowerbound[i] = -0.436332;
	  vars_upperbound[i] = 0.436332;
  }

  /* Acceleration/decceleration upper and lower limits. */
  for (unsigned int i = a_start; i < n_vars; i++) 
  {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] = 1.0;
  }

  // set constraints in MPC:Solve() to deal with the latency
  for (unsigned int i = delta_start; i < delta_start + fixed_steps; i++)
  {
	  vars_lowerbound[i] = prev_delta;
	  vars_upperbound[i] = prev_delta;
  }

  for (unsigned int i = a_start; i < a_start + fixed_steps; i++)
  {
	  vars_lowerbound[i] = prev_a;
	  vars_upperbound[i] = prev_a;
  }

  /* Lower and upper limits for constraints
   * All of these should be 0 except the initial
   * state indices.
   */
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) 
  {
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

  /*
   * object that computes objective and constraints
   */
  FG_eval fg_eval(coeffs);

  /*
   * options for IPOPT solver: don't have to worry about these
   */
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  //options += "Numeric max_cpu_time 0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  /* 
   * solve the problem
   */
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /*
   * Return the solution result. 
   * The variables can be accessed with
   */
  vector<double> result;

  for (unsigned int i = 0; i < N - 1; i++) 
  {
	  result.push_back(solution.x[x_start + i]);
	  result.push_back(solution.x[y_start + i]);
	  result.push_back(solution.x[delta_start + i]);
	  result.push_back(solution.x[a_start + i]);
  }

  // save values after solving MPC
  prev_delta = solution.x[delta_start + fixed_steps];
  prev_a = solution.x[a_start + fixed_steps];

  return result;
}

