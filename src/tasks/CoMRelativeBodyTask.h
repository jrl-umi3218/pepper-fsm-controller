#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace details
{

/** Define the task that can be understood by Tasks
 *
 * We need to implement the HighLevelTask interface:
 * - int dim() : the task dimension
 * - void update(const std::vector<rbd::MultiBody> & mbs,
 *               const std::vector<rbd::MultiBodyConfig> & mbcs,
 *               const SolverData & data) : update all the task quantities
 * - const Eigen::MatrixXd & jac() : returns the task jacobian
 * - const Eigen::VectorXd & eval() : returns the task evaluation
 * - const Eigen::VectorXd & speed() : returns the task speed
 * - const Eigen::VectorXd & normalAcc() : returns the task normal acceleration
 *
 * Here eval is: \f$p_{Body} -p_{CoM} - p^*\f$
 * Speed is: \f$v_{Body} - v_{CoM}\f$
 * Normal acc. is: \f$normalAcc_{Body} - normalAcc_{CoM}\f$
 * Jacobian is: \f$J_{Body} - J_{CoM}\f$
 *
 * Note: due to how the task is built we actually return -eval here
 */
struct CoMRelativeBodyTask : public tasks::qp::HighLevelTask
{
  CoMRelativeBodyTask(const mc_rbdyn::Robots & robots,
                      unsigned int robotIndex,
                      const std::string & body,
                      const Eigen::Vector3d & target);

  virtual ~CoMRelativeBodyTask() {}

  int dim() override;

  void update(const std::vector<rbd::MultiBody> & mbs,
              const std::vector<rbd::MultiBodyConfig> & mbcs,
              const tasks::qp::SolverData & data) override;

  const Eigen::MatrixXd & jac() override;
  const Eigen::VectorXd & eval() override;
  const Eigen::VectorXd & speed() override;
  const Eigen::VectorXd & normalAcc() override;

  /** Set the target relative position */
  void target(const Eigen::Vector3d & pos);

  /** Get the target relative position */
  const Eigen::Vector3d & target() const;

  const mc_rbdyn::Robot & robot() const;

  unsigned int bIndex() const;
private:
  const mc_rbdyn::Robots & robots_;
  unsigned int robotIndex_;
  rbd::CoMJacobian comJac_;
  rbd::Jacobian bodyJac_;
  Eigen::Vector3d target_;
  unsigned int bIndex_;

  Eigen::VectorXd eval_;
  Eigen::VectorXd speed_;
  Eigen::VectorXd normalAcc_;
  Eigen::MatrixXd jacMat_;
};

}

/** Now that we have a tasks::qp::HighLevelTask we can wrap it in mc_rtc's
 * generic TrajectoryTask wrapper */
struct CoMRelativeBodyTask : public mc_tasks::TrajectoryTaskGeneric<details::CoMRelativeBodyTask>
{
  // Shortcut name for the base class
  using Base = mc_tasks::TrajectoryTaskGeneric<details::CoMRelativeBodyTask>;

  CoMRelativeBodyTask(const std::string & body, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight);

  virtual ~CoMRelativeBodyTask() {}

  void target(const Eigen::Vector3d & pos);

  const Eigen::Vector3d & target() const;

  void reset() override;

  void addToLogger(mc_rtc::Logger & logger) override;

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;
};
