"""Auditable, fail-closed instrumentation of isolated source snapshots."""
from pathlib import Path


def replace(path, old, new, count=1):
    text = path.read_text()
    if text.count(old) != count:
        raise ValueError(f'{path}: expected {count} occurrences of {old[:80]!r}, got {text.count(old)}')
    path.write_text(text.replace(old, new))


def instrument(src: Path, baseline: bool, reader: str):
    # Reader is frozen alongside both snapshots. Remove only the current-only export block.
    if baseline:
        start = reader.index('        if(mapper.temporal_window_) {')
        end = reader.index('        const double stamp=', start)
        reader = reader[:start] + reader[end:]
    (src / 'src/offline.cpp').write_text(reader)
    cm = src / 'CMakeLists.txt'
    if baseline:
        text = cm.read_text().replace('-march=native', '-mtune=native')
        text = text.replace('find_package(rclcpp REQUIRED)', 'find_package(rclcpp REQUIRED)\nfind_package(rosbag2_cpp REQUIRED)')
        text += '\nadd_executable(fastlivo2_offline src/offline.cpp)\nament_target_dependencies(fastlivo2_offline ${FAST_LIVO2_DEPS} rosbag2_cpp)\ntarget_link_libraries(fastlivo2_offline fast_livo2_mapper)\n'
        cm.write_text(text)
        # Do not let hard-coded omp_set_num_threads defeat the frozen one-thread policy.
        for path in (src / 'src').glob('*.cpp'):
            path.write_text(path.read_text().replace('omp_set_num_threads(MP_PROC_NUM)', 'omp_set_num_threads(1)'))
        replace(src / 'src/LIVMapper.cpp',
                'if (img_time_correct - last_timestamp_img < 0.02)',
                'if (img_time_correct - last_timestamp_img + 1e-4 < 0.1)')
        # Compute the candidate visual solution on the live object, restore ALL state
        # before downstream map processing. No candidate posterior escapes the boundary.
        replace(src / 'src/vio.cpp', '  computeJacobianAndUpdateEKF(img);', '''  const StatesGroup experiment_before = *state;
  computeJacobianAndUpdateEKF(img);
  if (std::getenv("EXPERIMENT_VISUAL_OFF")) {
    *state = experiment_before;
    G.setZero();
    Rcw = Rci * state->rot_end.transpose();
    Pcw = -Rcw * state->pos_end + Pci;
    updateFrameState(*state);
  }''')
    replace(src / 'include/LIVMapper.h', 'public:', 'public:\n  int experiment_matches=-1;\n  double experiment_rms=0;', count=1)
    if not baseline:
        replace(src / 'src/LIVMapper_temporal_visual.cpp', '      _state.inv_expo_time=exposure;_state.cov(6,6)=exposure_variance;', '      if(temporal_visual_factor_enabled_) { _state.inv_expo_time=exposure;_state.cov(6,6)=exposure_variance; }')
        replace(src / 'src/LIVMapper_temporal.cpp', '  applyTemporalState(report);', '  experiment_matches=report.matches; experiment_rms=report.lidar_rms;\n  applyTemporalState(report);')
        replace(src / 'include/temporal_lio.h', '  const Config& config() const { return config_; }', '''  const Config& config() const { return config_; }
  void experimentGeometry(std::ostream& out, size_t event) const {
    if(nodes_.empty() || !config_.structural_landmarks) return;
    const auto& anchor=nodes_.at(index(plane_anchor_id_));
    for(const auto& s:nodes_) for(const auto& m:s.landmark_matches) {
      const auto& plane=landmarks_.at(m.landmark);
      const V3 p=s.state.rotation*s.points[m.source]+s.state.position;
      const double residual=plane.normal.dot(p-anchor.state.position)-plane.distance;
      out<<event<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<plane.normal.x()<<','<<plane.normal.y()<<','<<plane.normal.z()<<','<<residual<<','<<m.weight<<'\\n';
    }
  }''')
    if baseline:
        replace(src / 'include/voxel_map.h', '  float dis_to_plane_;', '  float dis_to_plane_;\n  double experiment_weight=0;')
        replace(src / 'src/voxel_map.cpp', '      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;', '      ptpl_list_[i].experiment_weight=R_inv(i);\n      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;')
    path = src / 'src/offline.cpp'
    replace(path, '#include <optional>', '#include <optional>\n#include "experiment_capture.h"')
    replace(path, '    auto drain=[&]() {', '''    ExperimentCapture capture(output);
    auto drain=[&]() {''')
    replace(path, '        mapper.handleFirstFrame();mapper.processImu();mapper.stateEstimationAndMapping();', '''        mapper.handleFirstFrame();mapper.processImu();
        const auto before = mapper._state;
        const auto update_start = std::chrono::steady_clock::now();
        capture.heartbeat("update_begin");
        mapper.experiment_matches=-1;\n        mapper.stateEstimationAndMapping();
        capture.event(mapper, before, std::chrono::duration<double>(std::chrono::steady_clock::now()-update_start).count());
        capture.heartbeat("idle");''')
    replace(path, '    const auto wall_start=std::chrono::steady_clock::now();', '''    auto next_input=std::chrono::steady_clock::now();
    std::optional<int64_t> previous_record_time;''')
    replace(path, '''        const auto target=wall_start+std::chrono::duration<double>(
            (record->time_stamp-*first_record_time)*1e-9/rate);''', '''        if(previous_record_time && record->time_stamp < *previous_record_time)
          throw std::runtime_error("Nonmonotonic bag record timestamp");
        if(previous_record_time) next_input += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>((record->time_stamp-*previous_record_time)*1e-9/rate));
        const auto target=next_input;
        capture.heartbeat("pacing");''')
    replace(path, '      drain();', '''      capture.heartbeat("dispatch");
      drain();
      previous_record_time=record->time_stamp;
      next_input=std::max(next_input,std::chrono::steady_clock::now());''')
    replace(path, '    if(mapper.img_en) filter.topics.push_back(mapper.img_topic);', '    filter.topics.push_back(mapper.img_topic);')
    replace(path, '    bool duration_reached=false;', '''    bool duration_reached=false;
    double last_scene=-1e20;
    std::filesystem::create_directories(output/"images");
    std::ofstream images(output/"images.csv"), inputs(output/"inputs.csv");
    images<<"bag_ns,stamp,accepted,path\\n"<<std::setprecision(17);
    inputs<<"bag_ns,topic\\n";
''')
    replace(path, '      if(!first_record_time) first_record_time=record->time_stamp;', '''      if(!first_record_time) first_record_time=record->time_stamp;
      inputs<<record->time_stamp<<','<<record->topic_name<<'\\n';''')
    replace(path, '      else if(record->topic_name==mapper.img_topic) {mapper.img_cbk(decode<sensor_msgs::msg::Image>(*record));++image_count;}', '''      else if(record->topic_name==mapper.img_topic) {
        const auto image=decode<sensor_msgs::msg::Image>(*record);
        const double stamp=fast_livo2::stampToSec(image->header.stamp);
        const double previous=mapper.last_timestamp_img;
        mapper.img_cbk(image); ++image_count;
        std::string filename;
        if(stamp-last_scene>=0.5-1e-4) {
          filename="images/"+std::to_string(image_count)+".png";
          if(!cv::imwrite((output/filename).string(),mapper.getImageFromMsg(image)))
            throw std::runtime_error("Image export failed");
          last_scene=stamp;
        }
        images<<record->time_stamp<<','<<stamp<<','<<(mapper.last_timestamp_img!=previous)<<','<<filename<<std::endl;
      }''')
    # Resolve all declared and automatically declared parameters into JSON.
    replace(path, '    rosbag2_cpp::Reader reader;', '''    { std::ofstream effective(output/"effective_parameters.txt");
      for(const auto& name: node->list_parameters({}, 100).names)
        effective << name << "=" << node->get_parameter(name).value_to_string() << "\\n";
    }
    rosbag2_cpp::Reader reader;''')
    header=HEADER
    if baseline:
        header=header.replace('EXPERIMENT_GEOMETRY', '''if(!visual) for(const auto& p:pairs) geometry << sequence << ',' << p.point_w_.x() << ',' << p.point_w_.y() << ',' << p.point_w_.z() << ',' << p.normal_.x() << ',' << p.normal_.y() << ',' << p.normal_.z() << ',' << p.dis_to_plane_ << ',' << p.experiment_weight << '\\n';''')
    else:
        header=header.replace('EXPERIMENT_GEOMETRY', 'if(!visual && m.temporal_window_) m.temporal_window_->experimentGeometry(geometry,sequence);')
    (src / 'include/experiment_capture.h').write_text(header)


HEADER = r'''#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
class ExperimentCapture {
  std::filesystem::path root;
  std::ofstream events, covariances, geometry, states, visual_points;
  size_t sequence=0;
public:
  explicit ExperimentCapture(const std::filesystem::path& p):root(p),events(p/"events.csv"),covariances(p/"covariance.csv"),geometry(p/"lidar_correspondences.csv"),states(p/"states.csv"),visual_points(p/"visual_points.csv") {
    visual_points<<"event,u,v,patch_squared_error,reference_x,reference_y,reference_z\n"<<std::setprecision(17);
    states << "event,phase,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,inv_exposure\n" << std::setprecision(17);
    events << "id,time,kind,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,matches,rms,seconds,visual_candidates,visual_disabled\n" << std::setprecision(17);
    covariances << "event,phase,row,col,value\n" << std::setprecision(17);
    geometry << "event,x,y,z,nx,ny,nz,residual,weight\n" << std::setprecision(17);
  }
  void heartbeat(const char* stage) {
    std::ofstream f(root/"heartbeat.tmp"); f << stage << std::endl; f.close();
    std::filesystem::rename(root/"heartbeat.tmp",root/"heartbeat");
  }
  template<class Mapper, class State> void event(Mapper& m,const State& before,double seconds) {
    const auto& s=m._state;
    if(!s.pos_end.allFinite() || !s.rot_end.allFinite() || !s.vel_end.allFinite() || !s.bias_g.allFinite() || !s.bias_a.allFinite() || !s.cov.allFinite()) throw std::runtime_error("nonfinite state");
    for(int phase=0;phase<2;phase++) {
      const auto& v=phase?s:before; const Eigen::Quaterniond q(v.rot_end);
      states<<sequence<<','<<(phase?"after":"before");
      for(int i=0;i<3;i++) states<<','<<v.pos_end[i];
      states<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
      for(auto x:{v.vel_end,v.bias_g,v.bias_a}) for(int i=0;i<3;i++) states<<','<<x[i];
      states<<','<<v.inv_expo_time<<'\n';
    }
    states.flush();
    const bool visual=m.LidarMeasures.lio_vio_flg==VIO;
    const auto& pairs=m.voxelmap_manager->ptpl_list_;
    int matches=m.lidar_map_inited ? int(pairs.size()) : -1;
    if(m.experiment_matches>=0) matches=m.experiment_matches;
    double sum=0; for(const auto& p:pairs) sum+=double(p.dis_to_plane_)*p.dis_to_plane_;
    events << sequence << ',' << m.LidarMeasures.last_lio_update_time-m._first_lidar_time << ',' << (visual?"visual":"lidar");
    for(auto v:{s.pos_end,s.vel_end,s.bias_g,s.bias_a}) for(int i=0;i<3;i++) events << ',' << v[i];
    events << ',' << (visual?-1:matches) << ',' << (m.experiment_matches>=0?m.experiment_rms:(pairs.empty()?0:std::sqrt(sum/pairs.size()))) << ',' << seconds << ',' << (visual?m.vio_manager->total_points:-1) << ',' << (std::getenv("EXPERIMENT_VISUAL_OFF")?1:0) << std::endl;
    if(visual && m.vio_manager->visual_submap) {
      const auto& v=*m.vio_manager; const auto& map=*v.visual_submap;
      for(size_t i=0;i<map.voxel_points.size() && i<map.errors.size();i++) {
        const auto* p=map.voxel_points[i]; if(!p) continue;
        const V3D pc=v.Rci*s.rot_end.transpose()*(p->pos_-s.pos_end)+v.Pci;
        if(pc.z()<=0) continue;
        const V2D uv=v.cam->world2cam(pc);
        visual_points<<sequence<<','<<uv.x()<<','<<uv.y()<<','<<map.errors[i]<<','<<p->pos_.x()<<','<<p->pos_.y()<<','<<p->pos_.z()<<'\n';
      }
      visual_points.flush();
    }
    for(int i=0;i<DIM_STATE;i++) for(int j=0;j<DIM_STATE;j++) {
      covariances << sequence << ",before," << i << ',' << j << ',' << before.cov(i,j) << '\n';
      covariances << sequence << ",after," << i << ',' << j << ',' << s.cov(i,j) << '\n';
    }
    covariances.flush();
    EXPERIMENT_GEOMETRY
    geometry.flush(); ++sequence;
  }
};
'''
