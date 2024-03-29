#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_pepper/tasks/CoMRelativeBodyTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include "api.h"

struct PepperFSMController_DLLAPI PepperFSMController : public mc_control::fsm::Controller
{
  PepperFSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::map<std::string, std::vector<double>> uprightStanding()
  {
    return uprightStanding_;
  }

  std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask()
  {
    return mobileBaseTask_;
  }

  std::shared_ptr<mc_pepper::CoMRelativeBodyTask> comTask()
  {
    return comTask_;
  }

  bool pepperHasSpeakers()
  {
    return speakerDeviceName_ != "";
  }
  bool pepperHasTablet()
  {
    return tabletDeviceName_ != "";
  }
  bool pepperHasBumpers()
  {
    return bumperSensorNames_.size() != 0;
  }

  std::string speakerDeviceName()
  {
    return speakerDeviceName_;
  }
  std::string tabletDeviceName()
  {
    return tabletDeviceName_;
  }
  std::vector<std::string> bumperSensorNames()
  {
    return bumperSensorNames_;
  }

private:
  // Controller configuration
  mc_rtc::Configuration config_;

  // Time for plot
  double t_ = 0.0;

  // Default Pepper straight posture
  std::map<std::string, std::vector<double>> uprightStanding_;

  // MobileBase position task
  std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask_;

  // Relative CoM task
  std::shared_ptr<mc_pepper::CoMRelativeBodyTask> comTask_;

  // Robot device names
  std::string speakerDeviceName_ = "";
  std::string tabletDeviceName_ = "";
  std::vector<std::string> bumperSensorNames_ = {};
};
