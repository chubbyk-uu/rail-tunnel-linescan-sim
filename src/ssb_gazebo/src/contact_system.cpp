// Contact-driven front wheels; scan servo reads the two spring-loaded measuring-wheel encoders only.
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <vector>
#include "ssb_core/pipeline.hpp"
#include "ssb_core/optix_renderer.hpp"
#include "control_math.hpp"
#include "assembly_check.hpp"
#include "world_check.hpp"

namespace ssb_gazebo {
class ContactSystem final : public gz::sim::System, public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
 public:
  ~ContactSystem() override {
    if(pipeline_) {try {if(!completion_.valid()){pipeline_->Finish();completion_=std::async(std::launch::async,[this]{return pipeline_->Wait();});} completion_.get();}
      catch(const std::exception& e){std::cerr<<"[contact] capture failed: "<<e.what()<<std::endl;}}
  }
  void Configure(const gz::sim::Entity& entity,const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,gz::sim::EventManager&) override {
    try {
      const char* cp=std::getenv("SSB_CONFIG"),*sp=std::getenv("SSB_SESSION");
      if(!cp||!sp)throw std::runtime_error("SSB_CONFIG and SSB_SESSION required");
      config_path_=cp;session_=sp;c_=ssb::Config::Load(cp);dynamics_only_=std::getenv("SSB_DYNAMICS_ONLY")!=nullptr;
      if (const char* topic = std::getenv("SSB_MISSION_STATUS_TOPIC"))
        status_pub_ = transport_.Advertise<gz::msgs::StringMsg>(topic);
      gz::sim::Model model(entity);base_=gz::sim::Link(model.LinkByName(ecm,"base"));base_.EnableVelocityChecks(ecm,true);
      CheckAssembly(model,ecm,c_);
      car_=model;
      for(const char* n:{"odometer_suspension","wheel_joint_1_suspension","wheel_joint_2_suspension","wheel_joint_3_suspension"}){
        gz::sim::Joint j(model.JointByName(ecm,n));if(!j.Valid(ecm))continue;j.EnablePositionCheck(ecm,true);springs_.push_back(j);}
      auto joint=[&](std::string name){gz::sim::Joint j(model.JointByName(ecm,name));if(!j.Valid(ecm))throw std::runtime_error("missing joint "+name);j.EnablePositionCheck(ecm,true);j.EnableVelocityCheck(ecm,true);return j;};
      for(int i=0;i<2;++i){enc_[i]=joint(sdf->Get<std::string>(i?"right_encoder":"left_encoder"));drive_[i]=joint(sdf->Get<std::string>(i?"right_drive":"left_drive"));diameter_[i]=sdf->Get<double>(i?"right_diameter":"left_diameter");}
      for(int i=0;i<2;++i){slide_[i]=joint(i?"measure_right_slide":"measure_left_slide");rear_[i]=joint(i?"wheel_joint_2":"odometer");}
      drive_diameter_=sdf->Get<double>("drive_diameter");settle_=sdf->Get<double>("settle_s");
      if(!c_.contact_enabled || std::abs(diameter_[0]-c_.odo_left_calibrated)>1e-12 || std::abs(diameter_[1]-c_.odo_right_calibrated)>1e-12)throw std::runtime_error("contact world/config mismatch");
      scan_=joint("scan");scan_.ResetPosition(ecm,{c_.start_theta_rad});
      counts_per_rad_=c_.odo_ppr*c_.odo_edges_per_cycle*c_.odo_gear_ratio/(2*M_PI);
      target_theta_=c_.start_theta_rad;
      log_root_=session_+"_dynamics";
      if(std::filesystem::exists(log_root_))throw std::runtime_error("diagnostics already exist");
      std::filesystem::create_directories(log_root_+"/metadata");std::filesystem::create_directories(log_root_+"/evaluation");
      obs_.open(log_root_+"/metadata/encoders.csv");truth_.open(log_root_+"/evaluation/contact.csv");
      obs_<<std::setprecision(17)<<"t,count_left,count_right,s_hat,theta_target,scan,scan_rate,torque_left,torque_right\n";
      truth_<<std::setprecision(17)<<"t,x,y,z,roll,pitch,yaw,vx,vy,vz,left_angle,right_angle,left_rate,right_rate,scan,scan_rate,"
              "measure_slide_left,measure_slide_right"
            <<(springs_.empty()?"":",suspension_rear_left,suspension_front_left,suspension_rear_right,suspension_front_right")<<"\n";
      std::cout<<"[contact] front drive, sprung measuring-wheel encoders; no world joint; diagnostics "<<log_root_<<std::endl;
    }catch(const std::exception& e){std::cerr<<"[contact] "<<e.what()<<std::endl;std::_Exit(2);}
  }
  void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
    if(info.paused)return;
    const double dt=std::chrono::duration<double>(info.dt).count(),t=std::chrono::duration<double>(info.simTime).count()-settle_;
    if(std::abs(dt-c_.sample_period_s)>1e-10){std::cerr<<"contact timestep mismatch";std::_Exit(2);}
    if(!world_checked_){
      // Models after the vehicle in the SDF do not exist yet during Configure.
      try{
        CheckTrackAndWheels(car_,ecm,c_.source_text);
        // Physical-world manifest beside the world: rails, wheel radii and springs as generated.
        const char* world=std::getenv("SSB_WORLD");
        if(!world)throw std::runtime_error("SSB_WORLD required for the physical-world check");
        world_path_=world;
        const auto manifest_path=world_path_.parent_path()/"physical_manifest.json";
        if(!std::filesystem::exists(manifest_path))
          throw std::runtime_error("physical-world manifest missing beside "+world_path_.string()+"; regenerate the world");
        std::ifstream in(manifest_path);nlohmann::json manifest;in>>manifest;
        images_=CheckPhysicalManifest(car_,ecm,c_.source_text,manifest);
        SnapshotPhysical(world_path_,config_path_,images_,log_root_+"/evaluation/physical");
        // Encoder distance is relative: a world spawned elsewhere would scan the wrong place.
        if(std::abs(gz::sim::worldPose(base_.Entity(),ecm).Pos().X()-c_.start_x_m)>1e-3)
          throw std::runtime_error("vehicle start position differs between world and capture configuration");
      }
      catch(const std::exception& e){std::cerr<<"[contact] "<<e.what()<<std::endl;std::_Exit(2);}
      world_checked_=true;
    }
    double factor=t>=0&&t<=c_.profile.back()[0]?ssb::EvaluateProfile(c_.profile,t).factor:0;
    double speed=c_.advance_per_rev_m*c_.NominalOmega()/(2*M_PI)*factor;
    for(int i=0;i<2;++i){auto v=drive_[i].Velocity(ecm);if(!v||v->empty())continue;
      const double error=2*speed/drive_diameter_-v->front();
      torque_[i]=std::clamp(12*error,-8.,8.);drive_[i].SetForce(ecm,{torque_[i]});}
    if(t<0){scan_.SetVelocity(ecm,{0});return;}
    if(!started_){
      for(int i=0;i<2;++i){auto p=enc_[i].Position(ecm);if(!p||p->empty())return;zero_[i]=static_cast<long long>(std::floor(p->front()*counts_per_rad_));}
      started_=true;
      if(!dynamics_only_) {try {StartPipeline();SnapshotPhysical(world_path_,config_path_,images_,session_+"/evaluation/physical");}
        catch(const std::exception& e){std::cerr<<"[contact] "<<e.what()<<std::endl;std::_Exit(2);}}
    }
    double s=0;
    for(int i=0;i<2;++i){auto p=enc_[i].Position(ecm);if(!p||p->empty())return;
      count_[i]=static_cast<long long>(std::floor(p->front()*counts_per_rad_))-zero_[i];s+=.5*count_[i]/counts_per_rad_*diameter_[i]/2;}
    s_hat_=s;
    auto p=scan_.Position(ecm);if(p&&!p->empty()){
      const double command=servo_.Update(s_hat_,p->front(),dt,c_.start_theta_rad,c_.advance_per_rev_m);
      target_theta_=servo_.Target();
      scan_.SetVelocity(ecm,{command});
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
    PublishStatus(info, ecm);
    if(info.paused||finished_)return;
    const double t=std::chrono::duration<double>(info.simTime).count()-settle_;
    auto pose=gz::sim::worldPose(base_.Entity(),ecm);auto linear=base_.WorldLinearVelocity(ecm);
    auto a=enc_[0].Position(ecm),b=enc_[1].Position(ecm),av=enc_[0].Velocity(ecm),bv=enc_[1].Velocity(ecm),th=scan_.Position(ecm),w=scan_.Velocity(ecm);
    if(!linear||!a||!b||!av||!bv||!th||!w||a->empty()||b->empty()||th->empty())return;
    auto rpy=pose.Rot().Euler();
    truth_<<t<<','<<pose.Pos().X()<<','<<pose.Pos().Y()<<','<<pose.Pos().Z()<<','<<rpy.X()<<','<<rpy.Y()<<','<<rpy.Z()<<','<<linear->X()<<','<<linear->Y()<<','<<linear->Z()<<','<<a->front()<<','<<b->front()<<','<<av->front()<<','<<bv->front()<<','<<th->front()<<','<<w->front();
    for(const auto& j:slide_){auto q=j.Position(ecm);truth_<<','<<(q&&!q->empty()?q->front():std::nan(""));}
    for(const auto& j:springs_){auto q=j.Position(ecm);truth_<<','<<(q&&!q->empty()?q->front():std::nan(""));}
    truth_<<'\n';
    if(started_){
      obs_<<t<<','<<count_[0]<<','<<count_[1]<<','<<s_hat_<<','<<target_theta_<<','<<th->front()<<','<<w->front()<<','<<torque_[0]<<','<<torque_[1]<<'\n';
      if(pipeline_) {
        ssb::PoseSample sample{t,pose.Pos().X(),linear->X(),th->front(),w->front(),a->front(),av->front()};
        sample.body_valid=1;sample.y=pose.Pos().Y();sample.z=pose.Pos().Z();
        sample.roll=rpy.X();sample.pitch=rpy.Y();sample.yaw=rpy.Z();sample.vy=linear->Y();sample.vz=linear->Z();
        if(auto angular=base_.WorldAngularVelocity(ecm)){
          const auto rates=EulerRates(sample.pitch,sample.yaw,{angular->X(),angular->Y(),angular->Z()});
          sample.roll_rate=rates[0];sample.pitch_rate=rates[1];sample.yaw_rate=rates[2];
        }
        sample.right_wheel=b->front();sample.right_wheel_omega=bv->front();
        pipeline_->Push(sample);
      }
    }
    if(t>=c_.profile.back()[0]){
      finished_=true;obs_.close();truth_.close();
      nlohmann::json summary={{"complete",true},{"end_s",t},{"s_hat",s_hat_},{"end_x",pose.Pos().X()},{"start_x",c_.start_x_m},
        {"end_y",pose.Pos().Y()},{"end_z",pose.Pos().Z()},{"encoder_counts",{count_[0],count_[1]}},{"layout","front-drive/measuring-wheel-encoders"},
        {"config",ssb::FileIdentity(config_path_)},{"mode","rigid friction contact"}};
      std::ofstream(log_root_+"/summary.json")<<summary.dump(2)<<'\n';
      if(pipeline_){pipeline_->Finish();completion_=std::async(std::launch::async,[this]{return pipeline_->Wait();});}
      std::cout<<"[contact] motion complete "<<summary.dump()<<std::endl;
    }
  }
 private:
  void PublishStatus(const gz::sim::UpdateInfo& info,
                     const gz::sim::EntityComponentManager& ecm) {
    if (!status_pub_) return;
    const auto now = std::chrono::steady_clock::now();
    // 30 Hz, the RViz frame rate: vehicle motion in RViz must not step visibly.
    if (now - last_status_ < std::chrono::milliseconds(33)) return;
    last_status_ = now;
    const auto pose = gz::sim::worldPose(base_.Entity(), ecm);
    const auto& p = pose.Pos(); const auto& q = pose.Rot();
    auto position = [&](const gz::sim::Joint& joint) {
      const auto value = joint.Position(ecm);
      return value && !value->empty() ? value->front() : 0.;
    };
    auto rate = scan_.Velocity(ecm);
    auto velocity = base_.WorldLinearVelocity(ecm);
    nlohmann::json status = {
      {"session", session_},
      {"sim_time", std::chrono::duration<double>(info.simTime).count()},
      {"paused", info.paused}, {"started", started_}, {"motion_complete", finished_},
      {"s_hat", s_hat_}, {"scan", position(scan_)},
      {"scan_rate", rate && !rate->empty() ? rate->front() : 0.},
      {"speed", velocity ? velocity->X() : 0.},
      {"base_pose", {p.X(), p.Y(), p.Z(), q.X(), q.Y(), q.Z(), q.W()}},
      {"wheel_angles", {position(rear_[0]), position(drive_[0]),
                         position(rear_[1]), position(drive_[1])}},
      {"measure_angles", {position(enc_[0]), position(enc_[1])}},
      {"measure_slides", {position(slide_[0]), position(slide_[1])}},
      {"suspension", [&]{nlohmann::json v=nlohmann::json::array();for(const auto& j:springs_)v.push_back(position(j));return v;}()},
      {"capture", pipeline_ ? pipeline_->Progress() : nlohmann::json::object()}};
    gz::msgs::StringMsg message; message.set_data(status.dump());
    status_pub_.Publish(message);
  }
  void StartPipeline(){
    auto renderer=std::make_unique<ssb::OptixRenderer>(c_,ssb::DefaultPtxPath(),c_.batch_rows);
    ssb::PipelineOptions options{session_,{"gz-contact",config_path_},"gazebo_contact"};options.inputs["config"]=ssb::FileIdentity(config_path_);
    if(const char* p=std::getenv("SSB_WORLD"))options.inputs["world"]=ssb::FileIdentity(p);
    options.planned_end_s=c_.profile.back()[0];pipeline_=std::make_unique<ssb::Pipeline>(c_,std::move(renderer),options);
  }
  ssb::Config c_;gz::sim::Joint enc_[2],drive_[2],slide_[2],rear_[2],scan_;gz::sim::Link base_;
  std::vector<gz::sim::Joint> springs_;
  gz::sim::Model car_;bool world_checked_=false;std::filesystem::path world_path_;std::vector<std::string> images_;  // rear-left, front-left, rear-right, front-right
  gz::transport::Node transport_;
  gz::transport::Node::Publisher status_pub_;
  std::chrono::steady_clock::time_point last_status_{};
  std::string config_path_,session_,log_root_;std::ofstream obs_,truth_;
  std::unique_ptr<ssb::Pipeline> pipeline_;std::future<nlohmann::json> completion_;
  ScanServo servo_;
  double diameter_[2]{},drive_diameter_=0,settle_=2,counts_per_rad_=0,torque_[2]{},s_hat_=0,target_theta_=0;
  long long count_[2]{},zero_[2]{};bool started_=false,finished_=false,dynamics_only_=false;
};
}
GZ_ADD_PLUGIN(ssb_gazebo::ContactSystem,gz::sim::System,ssb_gazebo::ContactSystem::ISystemConfigure,
 ssb_gazebo::ContactSystem::ISystemPreUpdate,ssb_gazebo::ContactSystem::ISystemPostUpdate)
