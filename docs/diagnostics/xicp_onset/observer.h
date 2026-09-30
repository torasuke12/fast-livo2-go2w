#pragma once
#include <Eigen/Core>
void observeLio(double time, int iteration, bool final, bool enabled,
 const Eigen::MatrixXd &H, const Eigen::VectorXd &W, const Eigen::VectorXd &residual,
 const Eigen::VectorXd &prior_delta, const Eigen::VectorXd &solution,
 const Eigen::MatrixXd &P, const Eigen::MatrixXd &committed_P, const Eigen::MatrixXd &gain,
 const Eigen::MatrixXd &C, const Eigen::MatrixXd &D);
