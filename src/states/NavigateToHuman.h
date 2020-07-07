#pragma once

#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/GazeTask.h>

struct NavigateToHuman : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    // Full state configuration
    mc_rtc::Configuration config_;

    // Time for plot
    double t_ = 0.0;

    // Indicate if state run function is called for the first time
    bool firstStateRun_ = true;

    // IBVS task for camera orientation control
    std::shared_ptr<mc_tasks::GazeTask> ibvsTask_;

    // PBVS task for mobile base navigation
    std::shared_ptr<mc_tasks::PositionBasedVisServoTask> mobileBasePBVSTask_;
    mc_control::CompletionCriteria pbvsTaskCriteria_;

    // Desired mobilebase taget position w.r.t to the human torso frame
    sva::PTransformd target_X_humanTorso = sva::PTransformd::Identity();
    // Target experessed in the camera frame as target_X_humanTorso * humanTorso_X_camera
    sva::PTransformd target_X_camera = sva::PTransformd::Identity();

    // Transformation between humanTorso and world/camera
    sva::PTransformd humanTorso_X_world = sva::PTransformd::Identity();
    sva::PTransformd humanTorso_X_camera = sva::PTransformd::Identity();

    // Transformation between humanHead and world/camera
    sva::PTransformd humanHead_X_world = sva::PTransformd::Identity();
    sva::PTransformd humanHead_X_camera = sva::PTransformd::Identity();

    // Mobile base position w.r.t camera as computed from kinematic chain
    sva::PTransformd mobileBase_X_camera = sva::PTransformd::Identity();

    // Camera transformation wrt world frame
    sva::PTransformd camera_X_world = sva::PTransformd::Identity();
};
