#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_control/mc_controller.h>

#include "constraints/BoundedAccelerationConstr.h"
#include "tasks/CoMRelativeBodyTask.h"
#include "api.h"

struct PepperFSMController_DLLAPI PepperFSMController : public mc_control::fsm::Controller
{
    PepperFSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    std::map<std::string, std::vector<double>> uprightStanding() { return uprightStanding_; }

    std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask() {return mobileBaseTask_; }

    bool pepperHasSpeakers() {return speakerDeviceName_ != ""; }
    bool pepperHasTablet() {return tabletDeviceName_ != ""; }
    bool pepperHasBumpers() {return bumperSensorNames_.size() != 0; }

    std::string speakerDeviceName() { return speakerDeviceName_; }
    std::string tabletDeviceName() { return tabletDeviceName_; }
    std::vector<std::string> bumperSensorNames() { return bumperSensorNames_; }

private:
    // Controller configuration
    mc_rtc::Configuration config_;

    // Time for plot
    double t_ = 0.0;

    // Default Pepper straight posture
    std::map<std::string, std::vector<double>> uprightStanding_;

    // MobileBase position task
    std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask_;

    // Mobile base acceleration constraints
    std::shared_ptr<BoundedAccelerationConstr> baseAccCstr_;
    double maxBaseTransAcc_, maxBaseRotAcc_;

    // Relative CoM task
    bool useCoMTask_;
    std::shared_ptr<CoMRelativeBodyTask> comTask_;
    double comTaskWeight_, comTaskStiffness_;

    // Robot device names
    std::string speakerDeviceName_ = "";
    std::string tabletDeviceName_ = "";
    std::vector<std::string> bumperSensorNames_ = {};
};
