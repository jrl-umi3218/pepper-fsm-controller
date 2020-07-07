#include "NavigateToHuman.h"
#include "../PepperFSMController.h"
#include <mc_tasks/MetaTaskLoader.h>

// Short names for types
using Color = mc_rtc::gui::Color;

void NavigateToHuman::configure(const mc_rtc::Configuration & config)
{
  // Read in entire state config
  config_.load(config);
}

void NavigateToHuman::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Only run this state if human model exists
  if(!ctl_.robots().hasRobot("human")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman | state requires human model in solver");
  }

  // Get desired mobile base target defined wrt human torso frame
  if(!config_.has("target_X_humanTorso")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | target_X_humanTorso config entry missing"); 
  }
  target_X_humanTorso = config_("target_X_humanTorso");
  sva::PTransformd humanPosW = ctl_.robots().robot("human").posW();
  // Set target to be on the groud for convenience
  target_X_humanTorso = target_X_humanTorso * sva::PTransformd(Eigen::Vector3d(0.0, 0.0, -humanPosW.translation()[2]));

  // Add IBVS task to solver to controll camera orientation
  if(!config_.has("ibvsTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | ibvsTask config entry missing"); 
  }
  // Unselect neck joints for the posture task
  ctl_.getPostureTask("pepper")->reset();
  ctl_.getPostureTask("pepper")->selectUnactiveJoints(ctl_.solver(), {"HeadYaw", "HeadPitch"});
  // Load IBVS task from config
  ibvsTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::GazeTask>(ctl_.solver(), config_("ibvsTask"));
  ctl_.solver().addTask(ibvsTask_);

  // Add PBVS task to solver to controll mobile base
  if(!config_.has("mobileBasePBVSTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | mobileBasePBVSTask config entry missing");
  }
  // Load PBVS task from config
  mobileBasePBVSTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::PositionBasedVisServoTask>(ctl_.solver(), config_("mobileBasePBVSTask"));
  if(!config_("mobileBasePBVSTask").has("completion")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | completion config entry missing for mobileBasePBVSTask");
  }
  pbvsTaskCriteria_.configure(*mobileBasePBVSTask_, ctl_.solver().dt(), config_("mobileBasePBVSTask")("completion"));
  ctl_.solver().addTask(mobileBasePBVSTask_);
  // Remove default free floating base position task from solver in this state
  ctl_.solver().removeTask(ctl.mobileBaseTask());

  // Add visual element to the scene
  ctl_.gui()->addElement({"NavigateToHuman", "Frames"},
      mc_rtc::gui::Transform("humanTorso", [this]() { return humanTorso_X_camera * camera_X_world; }),
      mc_rtc::gui::Point3D("humanHead", mc_rtc::gui::PointConfig({0., 1., 0.}, 0.03),
                            [this]() { return (humanHead_X_camera * camera_X_world).translation(); }),
      mc_rtc::gui::Transform("mBaseTarget", [this]() { return target_X_camera * camera_X_world; }),
      mc_rtc::gui::Transform("mBase", [this]() { return mobileBase_X_camera * camera_X_world; }),
      mc_rtc::gui::Transform("camera", [this]() { return camera_X_world; })
  );

  // Add PBVS task error components plot
  ctl_.gui()->addPlot("Navigation error",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return t_; }),
    mc_rtc::gui::plot::Y("X error", [this]() { return mobileBasePBVSTask_->eval()[3]; }, Color::Red),
    mc_rtc::gui::plot::Y("Y error ", [this]() { return mobileBasePBVSTask_->eval()[4]; }, Color::Green),
    mc_rtc::gui::plot::Y("W_z error", [this]() { return mobileBasePBVSTask_->eval()[2]; }, Color::Blue)
  );

  mc_rtc::log::success("NavigateToHuman state start done");
}

bool NavigateToHuman::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // State termination criteria
  if(pbvsTaskCriteria_.completed(*mobileBasePBVSTask_) && !firstStateRun_){
    output("OK");
    return true;
  }

  // Updtae time for plot
  t_ += ctl_.solver().dt();

  // Update humanTorso and humanHead to camera transformations
  camera_X_world = ctl_.robot().bodyPosW(ctl.camOpticalFrame());
  humanTorso_X_world = ctl_.robots().robot("human").bodyPosW("base_link");
  humanHead_X_world = ctl_.robots().robot("human").bodyPosW("HeadLink");
  humanTorso_X_camera = humanTorso_X_world * camera_X_world.inv();
  humanHead_X_camera = humanHead_X_world * camera_X_world.inv();

  // Update mobile base target to camera transformation
  target_X_camera = target_X_humanTorso * humanTorso_X_camera;

  // Update mobileBase to camera transformation
  mobileBase_X_camera = ctl_.robot().X_b1_b2(ctl.camOpticalFrame(), "base_link");

  // Update PBVS task error
  mobileBasePBVSTask_->error(mobileBase_X_camera * target_X_camera.inv());

  // Keep visual marker in the center of the onboard camera image
  ibvsTask_->error(humanHead_X_camera.translation());

  // Prevent terminating the state before task eval is updated after first task error update
  if(firstStateRun_){
    firstStateRun_ = false;
  }

  return false;
}

void NavigateToHuman::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Remove added gui elements
  ctl_.gui()->removeCategory({"NavigateToHuman", "Frames"});
  ctl_.gui()->removePlot("Navigation error");

  // Add default high weight mobile base position task back to solver
  ctl.mobileBaseTask()->reset();
  ctl_.solver().addTask(ctl.mobileBaseTask());

  // Remove PBVS task
  ctl_.solver().removeTask(mobileBasePBVSTask_);

  // Remove IBVS task
  ctl_.solver().removeTask(ibvsTask_);

  // Set neck joints in the posture task back to active
  ctl_.getPostureTask("pepper")->resetJointsSelector(ctl_.solver());

  mc_rtc::log::info("NavigateToHuman teardown done");
}

EXPORT_SINGLE_STATE("NavigateToHuman", NavigateToHuman)
