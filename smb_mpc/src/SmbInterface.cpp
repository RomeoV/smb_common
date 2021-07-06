#include "smb_mpc/SmbInterface.h"

#include "smb_mpc/SmbConstraints.h"
#include "smb_mpc/SmbCostWrapper.h"
#include "smb_mpc/SmbCost.h"
#include "smb_mpc/SmbOperatingPoint.h"
#include "smb_mpc/SmbSystemDynamics.h"

namespace smb_path_following {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbInterface::SmbInterface() {
  const std::string packagePath = ros::package::getPath("smb_mpc");
  const std::string taskFilePath = packagePath + "/config";
  const std::string taskFile = taskFilePath + "/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;

  // Model settings
  modelSettings_.loadSettings(taskFile, true);

  // SLQ-MPC settings
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile);

  // Dynamics
  auto dynamics = std::make_unique<SmbSystemDynamics>();
  std::cerr << "The system model name is " << modelSettings_.systemName_ << std::endl;
  dynamics->initialize(modelSettings_.systemName_, "tmp/ocs2", modelSettings_.recompileLibraries_);
  dynamicsPtr_ = std::move(dynamics);

  // Rollout
  const auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  ddpSmbRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings));

  // Cost function: joint space tracking cost
  ocs2::matrix_t QPosition(3, 3);
  ocs2::matrix_t QOrientation(3, 3);
  ocs2::matrix_t R(SmbDefinitions::INPUT_DIM, SmbDefinitions::INPUT_DIM);

  ocs2::loadData::loadEigenMatrix(taskFile, "QPosition", QPosition);
  ocs2::loadData::loadEigenMatrix(taskFile, "QOrientation", QOrientation);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);

  std::cerr << "QPosition:\n" << QPosition << std::endl;
  std::cerr << "QOrientation:\n" << QOrientation << std::endl;
  std::cerr << "R: \n" << R << std::endl;

  auto baseTrackingCost = std::make_unique<SmbCost>(QPosition, QOrientation, R);
  baseTrackingCost->initialize(7, 2, 7, "base_tracking_cost", "tmp/ocs2", modelSettings_.recompileLibraries_);
  costPtr_.reset(new SmbCostWrapper(std::move(baseTrackingCost)));

  // Constraints
  if (modelSettings_.activateObstacleAvoidance_) {
    ObstaclesParameters obstaclesParam;
    smbSynchronizedModulePtr_.reset(new SmbSynchronizedModule(std::move(obstaclesParam)));
    constraintPtr_.reset(new SmbConstraints(*smbSynchronizedModulePtr_));
  } else {
    constraintPtr_.reset(new ocs2::ConstraintBase);
  }

  // Initializer
  operatingPointPtr_.reset(new SmbOperatingPoint);

  // MPC
  mpcPtr_.reset(new ocs2::MPC_DDP(ddpSmbRolloutPtr_.get(), dynamicsPtr_.get(), constraintPtr_.get(),
                                  costPtr_.get(), operatingPointPtr_.get(), ddpSettings_, mpcSettings_));

  // set SynchronizedModules
  if (modelSettings_.activateObstacleAvoidance_) {
    mpcPtr_->getSolverPtr()->setSynchronizedModules({smbSynchronizedModulePtr_});
  }
}

} // namespace smb_path_following
