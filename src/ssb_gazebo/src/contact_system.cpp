// Contact-driven front wheels; scan servo reads rear encoder counts only.
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include "ssb_core/pipeline.hpp"
#include "ssb_core/optix_renderer.hpp"

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
      gz::sim::Model model(entity);base_=gz::sim::Link(model.LinkByName(ecm,"base"));base_.EnableVelocityChecks(ecm,true);
      auto joint=[&](std::string name){gz::sim::Joint j(model.JointByName(ecm,name));if(!j.Valid(ecm))throw std::runtime_error("missing joint "+name);j.EnablePositionCheck(ecm,true);j.EnableVelocityCheck(ecm,true);return j;};
      for(int i=0;i<2;++i){enc_[i]=joint(sdf->Get<std::string>(i?"right_encoder":"left_encoder"));drive_[i]=joint(sdf->Get<std::string>(i?"right_drive":"left_drive"));diameter_[i]=sdf->Get<double>(i?"right_diameter":"left_diameter");}
      drive_diameter_=sdf->Get<double>("drive_diameter");settle_=sdf->Get<double>("settle_s");
      scan_=joint("scan");scan_.ResetPosition(ecm,{c_.start_theta_rad});
      counts_per_rad_=c_.odo_ppr*c_.odo_edges_per_cycle*c_.odo_gear_ratio/(2*M_PI);
      target_theta_=c_.start_theta_rad;
      log_root_=session_+"_dynamics";
      if(std::filesystem::exists(log_root_))throw std::runtime_error("diagnostics already exist");
      std::filesystem::create_directories(log_root_+"/metadata");std::filesystem::create_directories(log_root_+"/evaluation");
      obs_.open(log_root_+"/metadata/encoders.csv");truth_.open(log_root_+"/evaluation/contact.csv");
      obs_<<std::setprecision(17)<<"t,count_left,count_right,s_hat,theta_target,scan,scan_rate,torque_left,torque_right\n";
      truth_<<std::setprecision(17)<<"t,x,y,z,roll,pitch,yaw,vx,vy,vz,left_angle,right_angle,left_rate,right_rate,scan,scan_rate\n";
      std::cout<<"[contact] front drive, rear dual encoders; no world joint; diagnostics "<<log_root_<<std::endl;
    }catch(const std::exception& e){std::cerr<<"[contact] "<<e.what()<<std::endl;std::_Exit(2);}
  }
  void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
    if(info.paused)return;
    const double dt=std::chrono::duration<double>(info.dt).count(),t=std::chrono::duration<double>(info.simTime).count()-settle_;
    if(std::abs(dt-c_.sample_period_s)>1e-10){std::cerr<<"contact timestep mismatch";std::_Exit(2);}
    double factor=t>=0&&t<=c_.profile.back()[0]?ssb::EvaluateProfile(c_.profile,t).factor:0;
    double speed=c_.advance_per_rev_m*c_.NominalOmega()/(2*M_PI)*factor;
    for(int i=0;i<2;++i){auto v=drive_[i].Velocity(ecm);if(!v||v->empty())continue;
      const double error=2*speed/drive_diameter_-v->front();
      torque_[i]=std::clamp(12*error,-8.,8.);drive_[i].SetForce(ecm,{torque_[i]});}
    if(t<0){scan_.SetVelocity(ecm,{0});return;}
    if(!started_){
      for(int i=0;i<2;++i){auto p=enc_[i].Position(ecm);if(!p||p->empty())return;zero_[i]=std::llround(p->front()*counts_per_rad_);}
      started_=true;
      if(!dynamics_only_)throw std::runtime_error("contact imaging requires the full-pose integration step; use physics-only validation meanwhile");
    }
    double s=0;
    for(int i=0;i<2;++i){auto p=enc_[i].Position(ecm);if(!p||p->empty())return;
      count_[i]=std::llround(p->front()*counts_per_rad_)-zero_[i];s+=.5*count_[i]/counts_per_rad_*diameter_[i]/2;}
    filtered_speed_+=(s-s_hat_-filtered_speed_*dt)/(0.025+dt);s_hat_=s;
    target_theta_=c_.start_theta_rad+2*M_PI*s_hat_/c_.advance_per_rev_m;
    auto p=scan_.Position(ecm);if(p&&!p->empty()){
      double rate=2*M_PI*filtered_speed_/c_.advance_per_rev_m+12*(target_theta_-p->front());
      scan_.SetVelocity(ecm,{std::clamp(rate,0.,3.3)});
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo& info,const gz::sim::EntityComponentManager& ecm) override {
    if(info.paused||finished_)return;
    const double t=std::chrono::duration<double>(info.simTime).count()-settle_;
    auto pose=gz::sim::worldPose(base_.Entity(),ecm);auto linear=base_.WorldLinearVelocity(ecm);
    auto a=enc_[0].Position(ecm),b=enc_[1].Position(ecm),av=enc_[0].Velocity(ecm),bv=enc_[1].Velocity(ecm),th=scan_.Position(ecm),w=scan_.Velocity(ecm);
    if(!linear||!a||!b||!av||!bv||!th||!w||a->empty()||b->empty()||th->empty())return;
    auto rpy=pose.Rot().Euler();
    truth_<<t<<','<<pose.Pos().X()<<','<<pose.Pos().Y()<<','<<pose.Pos().Z()<<','<<rpy.X()<<','<<rpy.Y()<<','<<rpy.Z()<<','<<linear->X()<<','<<linear->Y()<<','<<linear->Z()<<','<<a->front()<<','<<b->front()<<','<<av->front()<<','<<bv->front()<<','<<th->front()<<','<<w->front()<<'\n';
    if(started_){
      obs_<<t<<','<<count_[0]<<','<<count_[1]<<','<<s_hat_<<','<<target_theta_<<','<<th->front()<<','<<w->front()<<','<<torque_[0]<<','<<torque_[1]<<'\n';
      if(pipeline_) {
        ssb::PoseSample sample{t,pose.Pos().X(),linear->X(),th->front(),w->front(),a->front(),av->front()};
        pipeline_->Push(sample);
      }
    }
    if(t>=c_.profile.back()[0]){
      finished_=true;obs_.close();truth_.close();
      nlohmann::json summary={{"complete",true},{"end_s",t},{"s_hat",s_hat_},{"end_x",pose.Pos().X()},{"start_x",c_.start_x_m},
        {"end_y",pose.Pos().Y()},{"end_z",pose.Pos().Z()},{"encoder_counts",{count_[0],count_[1]}},{"layout","front-drive/rear-encoders"},
        {"config",ssb::FileIdentity(config_path_)},{"mode","rigid friction contact"}};
      std::ofstream(log_root_+"/summary.json")<<summary.dump(2)<<'\n';
      if(pipeline_){pipeline_->Finish();completion_=std::async(std::launch::async,[this]{return pipeline_->Wait();});}
      std::cout<<"[contact] motion complete "<<summary.dump()<<std::endl;
    }
  }
 private:
  void StartPipeline(){
    auto renderer=std::make_unique<ssb::OptixRenderer>(c_,ssb::DefaultPtxPath(),c_.batch_rows);
    ssb::PipelineOptions options{session_,{"gz-contact",config_path_},"gazebo_contact"};options.inputs["config"]=ssb::FileIdentity(config_path_);
    if(const char* p=std::getenv("SSB_WORLD"))options.inputs["world"]=ssb::FileIdentity(p);
    options.planned_end_s=c_.profile.back()[0];pipeline_=std::make_unique<ssb::Pipeline>(c_,std::move(renderer),options);
  }
  ssb::Config c_;gz::sim::Joint enc_[2],drive_[2],scan_;gz::sim::Link base_;
  std::string config_path_,session_,log_root_;std::ofstream obs_,truth_;
  std::unique_ptr<ssb::Pipeline> pipeline_;std::future<nlohmann::json> completion_;
  double diameter_[2]{},drive_diameter_=0,settle_=2,counts_per_rad_=0,torque_[2]{},s_hat_=0,filtered_speed_=0,target_theta_=0;
  long long count_[2]{},zero_[2]{};bool started_=false,finished_=false,dynamics_only_=false;
};
}
GZ_ADD_PLUGIN(ssb_gazebo::ContactSystem,gz::sim::System,ssb_gazebo::ContactSystem::ISystemConfigure,
 ssb_gazebo::ContactSystem::ISystemPreUpdate,ssb_gazebo::ContactSystem::ISystemPostUpdate)
