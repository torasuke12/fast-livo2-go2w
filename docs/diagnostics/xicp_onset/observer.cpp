#include "observer.h"
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cstdlib>

void observeLio(double time, int iteration, bool final, bool enabled,
 const Eigen::MatrixXd &H, const Eigen::VectorXd &W, const Eigen::VectorXd &residual,
 const Eigen::VectorXd &prior_delta, const Eigen::VectorXd &solution,
 const Eigen::MatrixXd &P, const Eigen::MatrixXd &committed_P, const Eigen::MatrixXd &gain,
 const Eigen::MatrixXd &C, const Eigen::MatrixXd &D)
{
  const char *path=std::getenv("LIO_OBSERVER_FILE");
  if (!path) return;
  try {
    static std::ofstream out(path);
    static bool header=false;
    if (!out) return;
    const Eigen::MatrixXd A=H.transpose()*W.asDiagonal()*H;
    const Eigen::MatrixXd I=Eigen::MatrixXd::Identity(P.rows(),P.cols());
    Eigen::MatrixXd info=P.ldlt().solve(I);
    info.topLeftCorner(6,6)+=A;
    const auto solver=info.ldlt();
    Eigen::MatrixXd rhs=Eigen::MatrixXd::Zero(P.rows(),H.rows());
    rhs.topRows(6)=H.transpose()*W.asDiagonal();
    const Eigen::MatrixXd K0=solver.solve(rhs);
    const Eigen::MatrixXd &K=enabled ? gain : K0;
    const Eigen::VectorXd innovation=residual-H*prior_delta.head(6);
    const Eigen::VectorXd b=H.transpose()*W.asDiagonal()*innovation;
    Eigen::VectorXd b_full=Eigen::VectorXd::Zero(P.rows()); b_full.head(6)=b;
    const double wrss=innovation.dot(W.asDiagonal()*innovation);
    const double nis=wrss-b_full.dot(solver.solve(b_full));
    Eigen::MatrixXd KH=K*H;
    const Eigen::MatrixXd noise=K*W.cwiseInverse().cwiseSqrt().asDiagonal();
    Eigen::VectorXd noise_diag=noise.array().square().rowwise().sum();
    Eigen::VectorXd delta_gain=(K-K0).rowwise().norm();
    Eigen::VectorXd k_innov=K*innovation, k0_innov=K0*innovation;
    Eigen::MatrixXd C_pad=Eigen::MatrixXd::Zero(6,P.rows());
    Eigen::VectorXd target=Eigen::VectorXd::Zero(6);
    if(enabled && C.rows()) { C_pad.topRows(C.rows())=C; target.head(C.rows())=D*innovation; }
    auto fields=[&](const char *name,const Eigen::MatrixXd &v,bool titles) {
      for(int i=0;i<v.rows();++i) for(int j=0;j<v.cols();++j) {
        out<<','; if(titles) out<<name<<'_'<<i<<'_'<<j; else out<<v(i,j);
      }
    };
    auto matrices=[&](bool titles) {
      fields("HWH",A,titles); fields("Piter",P,titles); fields("Pcommitted",committed_P,titles);
      fields("KH",KH,titles); fields("gain_noise_diag",noise_diag,titles); fields("gain_delta_norm",delta_gain,titles);
      fields("Kinnov",k_innov,titles); fields("K0innov",k0_innov,titles);
      fields("solution",solution,titles); fields("prior_delta",prior_delta,titles);
      fields("C",C_pad,titles); fields("target",target,titles);
    };
    if(!header) {
      out<<"time,iteration,final,enabled,matches,residual_rms,residual_max,innovation_rms,wrss,nis,constraints";
      matrices(true); out<<'\n'; out<<std::setprecision(12); header=true;
    }
    out<<time<<','<<iteration<<','<<final<<','<<enabled<<','<<H.rows()<<','<<residual.norm()/std::sqrt(H.rows())
       <<','<<residual.cwiseAbs().maxCoeff()<<','<<innovation.norm()/std::sqrt(H.rows())<<','<<wrss<<','<<nis<<','<<(enabled ? C.rows() : 0);
    matrices(false); out<<'\n'; if(final) out.flush();
  } catch(const std::exception &e) { std::cerr<<"LIO observer error: "<<e.what()<<std::endl; }
}
