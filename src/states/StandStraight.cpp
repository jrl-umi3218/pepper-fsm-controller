#include "StandStraight.h"

#include "../PepperFSMController.h"

void StandStraight::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void StandStraight::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  if(config_.has("postureTaskCompletion")){
    config_("postureTaskCompletion", postureTaskCompletion_);
  }else{
    mc_rtc::log::error_and_throw<std::runtime_error>("StandStraight | postureTaskCompletion config entry missing");
  }

  ctl_.getPostureTask("pepper")->target(ctl.uprightStanding());
}

bool StandStraight::run(mc_control::fsm::Controller & ctl_)
{
  if(ctl_.getPostureTask("pepper")->eval().norm() < postureTaskCompletion_){
    output("OK");
    return true;
  }
  return false;
}

void StandStraight::teardown(mc_control::fsm::Controller &)
{
}

EXPORT_SINGLE_STATE("StandStraight", StandStraight)
