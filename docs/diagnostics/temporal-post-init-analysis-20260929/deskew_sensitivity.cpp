// Diagnostic only: link the captured estimator's temporal library. No mapper
// state or configuration is modified. The reference plane stays fixed so this
// isolates the source return's velocity/bias dependence from data association.
#include "temporal_lio.h"
#include <iomanip>
#include <iostream>

using namespace temporal_lio;
int main()
{
  Config config;
  State end;
  std::vector<ImuSample> imu;
  for(int k=0;k<=20;++k) imu.push_back({k*.005,V3::Zero(),V3(0,0,9.81)});
  const std::vector<Return> raw{{V3(3,1,-1),0,0}};
  const auto residual=[&](const State& state) {
    const V3 world=state.rotation*deskew(raw,imu,.1,state,config).front()+state.position;
    return world.z()+1.0;
  };
  const char* names[]={"velocity_z","gyro_bias_x","gyro_bias_y","accel_bias_z"};
  const int coordinates[]={8,9,10,14};
  std::cout<<std::setprecision(12)<<"{\n  \"scan_end_minus_point_time_sec\": 0.1,\n  \"components\": [\n";
  for(int i=0;i<4;++i) {
    V15 dx=V15::Zero();dx[coordinates[i]]=1e-5;
    const double derivative=(residual(end.plus(dx))-residual(end.plus(-dx)))/(2e-5);
    std::cout<<"    {\"state\": \""<<names[i]<<"\", \"finite_difference_raw_residual\": "<<derivative
      <<", \"current_frozen_point_lidar_jacobian\": 0}"<<(i==3 ? "\n" : ",\n");
  }
  std::cout<<"  ]\n}\n";
}
