/*
 * ex2_test_ipopt.cc
 *
 *  Created on: Jul 16, 2018
 *      Author: jelavice
 */


#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/test_vars_constr_cost2.h>

using namespace ifopt;

int main()
{
  // 1. define the problem
  Problem nlp;
  nlp.AddVariableSet  (std::make_shared<Ex2Variables>());
  nlp.AddConstraintSet(std::make_shared<Ex2Constraint>());
  nlp.AddCostSet      (std::make_shared<Ex2Cost>());
  nlp.PrintCurrent();

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "ma57");
  ipopt.SetOption("jacobian_approximation", "exact");
  //ipopt.SetOption("use_jacobian_approximation", true);

  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << std::endl;

  // 4. test if solution correct
  double eps = 1e-5; //double precision
  assert(1.0-eps < x(0) && x(0) < 1.0+eps);
  assert(0.0-eps < x(1) && x(1) < 0.0+eps);
}




