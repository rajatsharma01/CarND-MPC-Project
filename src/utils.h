#ifndef _UTILS_H
#define _UTILS_H

#include <math.h>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Uncomment following to enable debug prints
//#define __DEBUG__

using CppAD::AD;

// For converting back and forth between radians and degrees.
static inline constexpr double pi() { return M_PI; }
static inline double deg2rad(double x) { return x * pi() / 180; }
static inline double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Evaluate a polynomial with CppAD
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x);

// Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

// Calculate the derivative of polynomial given its coefficients
Eigen::VectorXd polyderive(Eigen::VectorXd coeffs);

#endif // _UTILS_H
