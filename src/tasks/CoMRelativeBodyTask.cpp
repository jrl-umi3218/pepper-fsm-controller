#include "CoMRelativeBodyTask.h"
#include <mc_rtc/gui/Point3D.h>

namespace details
{

CoMRelativeBodyTask::CoMRelativeBodyTask(const mc_rbdyn::Robots & robots,
                                         unsigned int robotIndex,
                                         const std::string & body,
                                         const Eigen::Vector3d & target)
: robots_(robots), robotIndex_(robotIndex), comJac_(robots.robot(robotIndex).mb()), bodyJac_(robots.robot(robotIndex).mb(), body), target_(target)
{
  const auto & robot = robots_.robot(robotIndex_);
  bIndex_ = robot.bodyIndexByName(body);
  eval_.setZero(3);
  speed_.setZero(3);
  normalAcc_.setZero(3);
  jacMat_.setZero(3, robot.mb().nrDof());
}

int CoMRelativeBodyTask::dim()
{
  return 3;
}

void CoMRelativeBodyTask::update(const std::vector<rbd::MultiBody> &,
                                 const std::vector<rbd::MultiBodyConfig> &,
                                 const tasks::qp::SolverData &)
{
  const auto & robot_ = robots_.robot(robotIndex_);
  eval_ = -target_ + robot_.com() - robot_.mbc().bodyPosW[bIndex_].translation();
  speed_ = robot_.mbc().bodyVelW[bIndex_].linear() - robot_.comVelocity();
  normalAcc_ = bodyJac_.normalAcceleration(robot_.mb(), robot_.mbc()).linear()
               - comJac_.normalAcceleration(robot_.mb(), robot_.mbc());

  const auto & bJac = bodyJac_.jacobian(robot_.mb(), robot_.mbc()).block(3, 0, 3, bodyJac_.dof());
  bodyJac_.fullJacobian(robot_.mb(), bJac, jacMat_);
  jacMat_-= comJac_.jacobian(robot_.mb(), robot_.mbc());
}

const Eigen::MatrixXd & CoMRelativeBodyTask::jac()
{
  return jacMat_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::eval()
{
  return eval_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::speed()
{
  return speed_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::normalAcc()
{
  return normalAcc_;
}

void CoMRelativeBodyTask::target(const Eigen::Vector3d & target)
{
  target_ = target;
}

const Eigen::Vector3d & CoMRelativeBodyTask::target() const
{
  return target_;
}

const mc_rbdyn::Robot & CoMRelativeBodyTask::robot() const
{
  return robots_.robot(robotIndex_);
}

unsigned int CoMRelativeBodyTask::bIndex() const
{
  return bIndex_;
}

}

CoMRelativeBodyTask::CoMRelativeBodyTask(const std::string & body, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: mc_tasks::TrajectoryTaskGeneric<details::CoMRelativeBodyTask>(robots, robotIndex, stiffness, weight)
{
  const auto & robot = robots.robot(robotIndex);
  Eigen::Vector3d init = robot.bodyPosW(body).translation() - robot.com();
  finalize(robots, robotIndex, body, init);
  type_ = "com_relative_body";
  name_ = "com_relative_" + body;
}

void CoMRelativeBodyTask::target(const Eigen::Vector3d & pos)
{
  errorT->target(pos);
}

const Eigen::Vector3d & CoMRelativeBodyTask::target() const
{
  return errorT->target();
}

void CoMRelativeBodyTask::reset()
{
  const auto & robot = errorT->robot();
  target(robot.mbc().bodyPosW[errorT->bIndex()].translation() + robot.com());
}

void CoMRelativeBodyTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", [this]() -> const Eigen::VectorXd & { return errorT->eval(); });
  logger.addLogEntry(name_ + "_body_pos", [this]() -> const Eigen::Vector3d & { return errorT->robot().mbc().bodyPosW[errorT->bIndex()].translation(); });
  logger.addLogEntry(name_ + "_com", [this]() { return errorT->robot().com(); });
  logger.addLogEntry(name_ + "_comTarget", [this]() { return errorT->target(); });
}

void CoMRelativeBodyTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_eval");
  logger.removeLogEntry(name_ + "_body_pos");
  logger.removeLogEntry(name_ + "_com");
  logger.removeLogEntry(name_ + "_comTarget");
}

void CoMRelativeBodyTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<details::CoMRelativeBodyTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayInput("relPos", [this]() { return errorT->target(); },
                                      [this](const Eigen::VectorXd & pos) { errorT->target(pos); }),
                                   mc_rtc::gui::Point3D("com", [this]() { return errorT->robot().com(); }),
                                   mc_rtc::gui::Point3D("target", mc_rtc::gui::PointConfig({0., 1., 0.}, 0.03),
                                      [this]() { return Eigen::Vector3d(errorT->target() + errorT->robot().mbc().bodyPosW[errorT->bIndex()].translation() ); }));
}
