#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "MPC.hpp"
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

namespace model_predictive_control_project {

// TODO: Set the timestep length and duration
// I've choose N and dt so that the prediction horizon T = N*dt is as big as
// possible but not exceeding a few seconds. I also tried to keep dt as small
// as possible. Other configurations I tried but were not working for me:
// N=30, dt=0.1 ; N=20, dt=0.1; N=20, dt=0.05
constexpr size_t N = 10;
constexpr double dt = 0.1;
constexpr double ref_cte = 0.0;
constexpr double ref_epsi = 0.0;
constexpr double ref_v = 80.0;
constexpr size_t n_vars = N * 6 + (N - 1) * 2; // in N timesteps there is N-1 actuations
constexpr size_t n_constraints = N * 6;

// state varaibles indexes:
constexpr size_t x_start = 0;
constexpr size_t y_start = x_start + N;
constexpr size_t psi_start = y_start + N;
constexpr size_t v_start = psi_start + N;
constexpr size_t cte_start = v_start + N;
constexpr size_t epsi_start = cte_start + N;

// actuators indexes
constexpr size_t delta_start = epsi_start + N;
constexpr size_t a_start = delta_start + N - 1;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
constexpr double Lf = 2.67;

inline auto pow2(auto val) { return CppAD::pow(val, 2); }

class FG_eval {
  const VectorXd& coeffs_;
public:

  // Fitted polynomial coefficients
  FG_eval(const VectorXd& coeffs)
  : coeffs_{coeffs} { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    auto& cost_func = fg[0];

    // setup reference state cost
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; ++t) {
      cost_func += 1000*pow2(vars[cte_start + t] - ref_cte);
      cost_func += 1000*pow2(vars[epsi_start + t] - ref_epsi);
      cost_func += pow2(vars[v_start + t] - ref_v);
    }

    // Minimize change-rate.
    for (size_t t = 0; t < N - 1; ++t) {
      cost_func += 10*pow2(vars[delta_start + t]);
      cost_func += 10*pow2(vars[a_start + t]);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; ++t) {
      cost_func += 100*pow2(vars[delta_start + t + 1] - vars[delta_start + t]);
      cost_func += 10*pow2(vars[a_start + t + 1] - vars[a_start + t]);
    }

    // Setup Constraints
    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; ++t) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta_t = vars[delta_start + t - 1];
      AD<double> a_t = vars[a_start + t - 1];
      AD<double> f_t = coeffs_[0]+
                       coeffs_[1]*x0+
                       coeffs_[2]*x0*x0+
                       coeffs_[3]*x0*x0*x0;
      AD<double> psides_t = CppAD::atan(coeffs_[1]+
                                        coeffs_[2]*x0*2+
                                        coeffs_[3]*x0*x0*3);

      // set the model constraints using update equations from global kinematic 
      // model that tells how state variables changes in time t+1 based on state
      // at time t;
      // x_t_1 = x_t + v_t * cos(psi_t) *dt
      // y_t_1 = y_t + v_t * sin(psi_t) * dt
      // psi_t_1 = psi_t + v_t / Lf * delta_t * dt
      // cte_t_1 = f(x_t) - y_t + v_t * sin(epsi_t) * dt 
      // epsi_t_1 = psi_t - psides_t + v_t / Lf * delta_t * dt
      const auto psi_change = v0 * delta_t / Lf * dt;
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + psi_change);
      fg[1 + v_start + t] = v1 - (v0 + a_t * dt);
      fg[1 + cte_start + t] = cte1 - ((f_t - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides_t) + psi_change);
    }
  }
};


vector<double> MPC::Solve(const VectorXd& state, const VectorXd& coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // here I access the state of the model. The state includes:
  // - x and y position of the car: x,y
  // - orientation of the car: psi
  // - velocity of the car: v
  // - cross track error of the car: cte
  // - car orientation error: epsi
  const auto& x = state[0];
  const auto& y = state[1];
  const auto& psi = state[2];
  const auto& v = state[3];
  const auto& cte = state[4];
  const auto& epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // set actuators limits
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] =  0.436332*Lf;
  }
  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i) {
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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result{};
  // first add actuators values: steering and throttle
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  // then add the MPC predicted trajectory
  for(size_t t = 0; t <  N-1; ++t) {
    result.push_back(solution.x[x_start + t+1]);
    result.push_back(solution.x[y_start + t+1]);
  }
  return result;
}

} // model_predictive_control_project
