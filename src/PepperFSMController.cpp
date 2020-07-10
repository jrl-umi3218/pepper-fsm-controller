#include "PepperFSMController.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_pepper/devices/VisualDisplay.h>
#include <mc_pepper/devices/TouchSensor.h>
#include <mc_pepper/devices/Speaker.h>
#include <mc_rtc/gui/plot.h>

// Short names for types
using Color = mc_rtc::gui::Color;
using Style = mc_rtc::gui::plot::Style;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;

PepperFSMController::PepperFSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // Load default Pepper straight posture
  if(!config.has("uprightStanding")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PepperFSMController | uprightStanding config entry missing");
  }
  config("uprightStanding", uprightStanding_);

  // Camera optical frame name
  if(!config.has("camOpticalFrame")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PepperFSMController | camOpticalFrame config entry missing");
  }
  config("camOpticalFrame", camOpticalFrame_);

  // Load entire controller configuration file
  config_.load(config);
  mc_rtc::log::success("PepperFSMController init done");
}

bool PepperFSMController::run()
{
  // Update time value for plot
  t_ += solver().dt();
  return mc_control::fsm::Controller::run();
}

void PepperFSMController::reset(const mc_control::ControllerResetData & reset_data)
{
  // Actuate mobile base
  auto inf = std::numeric_limits<double>::infinity();
  robot().tl()[0] = {0, 0, -inf, -inf, -inf, 0};
  robot().tu()[0] = {0, 0, inf, inf, inf, 0};
  // Update dynamics constraints
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(),
                                                     robot().robotIndex(),
                                                     solver().dt(), {0.1, 0.01, 0.5});
  // Must be added to the solver before controller reset
  solver().addConstraintSet(dynamicsConstraint);

  // Add human dynamics constraints
  if(robots().hasRobot("human")){
    humanDynamicsConstraint_ = mc_solver::DynamicsConstraint(robots(),
                                                       robots().robot("human").robotIndex(),
                                                       solver().dt(), {0.1, 0.01, 0.5});
    solver().addConstraintSet(humanDynamicsConstraint_);

    // Human model start posture
    if(config_("human").has("posture")){
      if(config_("human")("posture").has("target")){
        std::map<std::string, std::vector<double>> humanPostureTarget = config_("human")("posture")("target");
        // Set human mbc equal to the posture target
        for(auto const & t : humanPostureTarget){
          robots().robot("human").mbc().q[robots().robot("human").jointIndexByName(t.first)] = t.second;
        }
      }
    }
    // Adjust chair position relative to human model
    if(robots().hasRobot("chair")){
      robots().robot("chair").posW(robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.0, -0.65)));
    }
  }

  // Takes care of updating constraints and reseting posture tasks
  mc_control::fsm::Controller::reset(reset_data);

  // Mobile base position task
  if(!config_.has("mobileBaseTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PepperFSMController | mobileBaseTask config entry missing");
  }
  mobileBaseTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), config_("mobileBaseTask"));
  solver().addTask(mobileBaseTask_);

  // Load relative CoM task
  if(config_.has("comTask")){
    comTask_ = mc_tasks::MetaTaskLoader::load<mc_pepper::CoMRelativeBodyTask>(solver(), config_("comTask"));
    solver().addTask(comTask_);

    // CoM projection plot
    if(config_("comPlot", false)){
      gui()->addXYPlot(
          "Center of Mass projection",
          mc_rtc::gui::plot::XY("CoM model", [this]() { return robot().com()[0] - robot().posW().translation()[0]; },
                                             [this]() { return robot().com()[1] - robot().posW().translation()[1]; }, Color(1, 0.5, 0), Style::Point),
          mc_rtc::gui::plot::XY("CoM real", [this]() { return realRobot().com()[0] - realRobot().posW().translation()[0]; },
                                            [this]() { return realRobot().com()[1] - realRobot().posW().translation()[1]; }, Color::Red, Style::Point),
          mc_rtc::gui::plot::XY("CoM target", [this]() { return comTask_->target()[0]; }, [this]() { return comTask_->target()[1]; }, Color::Green, Style::Point),
          mc_rtc::gui::plot::Polygon("Support", []() { return PolygonDescription({{0.09, -0.155}, {-0.17, 0}, {0.09, 0.155}}, Color::Blue); }));
    }
  }

  // Check robot devices
  if(config_.has("speakerDeviceName")){
    std::string deviceName = config_("speakerDeviceName");
    if(robot().hasDevice<mc_pepper::Speaker>(deviceName)){
      speakerDeviceName_ = deviceName;
    }else{
      mc_rtc::log::warning("PepperFSMController | robot has no device named {}", deviceName);
    }
  }
  if(config_.has("tabletDeviceName")){
    std::string deviceName = config_("tabletDeviceName");
    if(robot().hasDevice<mc_pepper::VisualDisplay>(deviceName)){
      tabletDeviceName_ = deviceName;
    }else{
      mc_rtc::log::warning("PepperFSMController | robot has no device named {}", deviceName);
    }
  }
  if(config_.has("bumperSensorNames")){
    std::vector<std::string> deviceNames = config_("bumperSensorNames");
    if(deviceNames.size() == 3){
      for(unsigned int i = 0; i < deviceNames.size(); i++){
        if(robot().hasDevice<mc_pepper::TouchSensor>(deviceNames[i])){
          bumperSensorNames_.push_back(deviceNames[i]);
        }else{
          mc_rtc::log::warning("PepperFSMController | robot has no device named {}", deviceNames[i]);
        }
      }
    }
  }

  // Set up log
  auto getBaseTau = [this]() -> const std::vector<double> &{
    static std::vector<double> tauOut(robot().mbc().jointTorque[0].size());
    solver().fillTorque(dynamicsConstraint);
    for(size_t i = 0; i < tauOut.size(); ++i){
      tauOut[i] = robot().mbc().jointTorque[0][i];
    }
    return tauOut;
  };
  auto getBaseAlphaD = [this]() -> const std::vector<double> &{
    static std::vector<double> alphaDOut(robot().mbc().alphaD[0].size());
    for(size_t i = 0; i < alphaDOut.size(); ++i){
      alphaDOut[i] = robot().mbc().alphaD[0][i];
    }
    return alphaDOut;
  };
  auto getBaseAlpha = [this]() -> const std::vector<double> &{
    static std::vector<double> alphaOut(robot().mbc().alpha[0].size());
    for(size_t i = 0; i < alphaOut.size(); ++i){
      alphaOut[i] = robot().mbc().alpha[0][i];
    }
    return alphaOut;
  };
  auto getBaseQ = [this]() -> const std::vector<double> &{
    static std::vector<double> qOut(robot().mbc().q[0].size());
    for(size_t i = 0; i < qOut.size(); ++i){
      qOut[i] = robot().mbc().q[0][i];
    }
    return qOut;
  };

  logger().addLogEntry(
      "base_tau", [getBaseTau]() -> const std::vector<double> & { return getBaseTau(); });
  logger().addLogEntry(
      "base_alphaD", [getBaseAlphaD]() -> const std::vector<double> & { return getBaseAlphaD(); });
  logger().addLogEntry(
      "base_alpha", [getBaseAlpha]() -> const std::vector<double> & { return getBaseAlpha(); });
  logger().addLogEntry(
      "base_q", [getBaseQ]() -> const std::vector<double> & { return getBaseQ(); });

  // Human model log
  auto getHumanTau = [this]() -> const std::vector<double> &
  {
    static std::vector<double> tauOut(robots().robot("human").refJointOrder().size());
    solver().fillTorque(humanDynamicsConstraint_);
    auto & mbc = robots().robot("human").mbc();
    const auto & rjo = robots().robot("human").refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & tauj : mbc.jointTorque[robots().robot("human").jointIndexByName(jn)]){
        tauOut[i] = tauj;
      }
      i++;
    }
    return tauOut;
  };

  logger().addLogEntry(
    "Human_Torques", [getHumanTau]() -> const std::vector<double> & { return getHumanTau(); });

}
