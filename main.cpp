// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Eigen/Dense"

using namespace Eigen;

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;


Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());

  //TODO: Implement the Global Kinematic Model, to return
  // the next state from inputs

  // NOTE: state is [x, y, psi, v]
  // NOTE: actuators is [delta, a]

  //Add your code below
  double x_t = state(0);
  double y_t = state(1);
  double psi_t = state(2);
  double v_t = state(3);
  double delta_t = actuators(0);
  double a_t = actuators(1);

  double x = x_t + v_t * cos(psi_t) * dt;
  double y = y_t + v_t * sin(psi_t) * dt;
  double psi = psi_t + v_t * delta_t * dt / Lf;
  double v = v_t + a_t * dt;
  next_state << x, y, psi, v;

  return next_state;
}

void validate_globalKinematic() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}

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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

void validate_polyfit() {
  Eigen::VectorXd xvals(6);
  Eigen::VectorXd yvals(6);
  // x waypoint coordinates
  xvals << 9.261977, -2.06803, -19.6663, -36.868, -51.6263, -66.3482;
  // y waypoint coordinates
  yvals << 5.17, -2.25, -15.306, -29.46, -42.85, -57.6116;

  // TODO: use `polyfit` to fit a third order polynomial to the (x, y)
  // coordinates.
  // Hint: call Eigen::VectorXd polyfit() and pass xvals, yvals, and the
  // polynomial degree/order
  // YOUR CODE HERE
  VectorXd fit = polyfit(xvals, yvals, 3);

  for (double x = 0; x <= 20; x += 1.0) {
    // TODO: use `polyeval` to evaluate the x values.
    std::cout << polyeval(fit, x) << std::endl;
  }
}

int main() {
  // validate_globalKinematic();
  validate_polyfit();
}