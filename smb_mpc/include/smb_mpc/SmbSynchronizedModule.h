/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "smb_mpc/ObstaclesParameters.h"
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <ros/ros.h>

namespace smb_path_following {

class SmbSynchronizedModule final : public ocs2::SolverSynchronizedModule {
public:
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using matrix_t = ocs2::matrix_t;

    SmbSynchronizedModule(ObstaclesParameters obstaclesParameters) {
      sub_ = nh_.subscribe("mapGuy/obstacles", 1000, &SmbSynchronizedModule::chatterCallback, this);
    }

  ~SmbSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override {

    // new approach
    // read from publisher: mapGuy/obstacles
    this->obstaclesParameters_ = obstaclesParameters2_;

    // delete this
    // const std::string packagePath = ros::package::getPath("smb_mpc");
    // const std::string taskFilePath = packagePath + "/config";
    // const std::string obstacleFile = taskFilePath + "/obstacles.info";
    // ObstaclesParameters obstaclesParam;
    // obstaclesParam.loadSettings(obstacleFile, "obstacles_parameters");
    // this->obstaclesParameters_ = obstaclesParam;
  }

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override {}

  const ObstaclesParameters& getObstaclesParameters() const { return obstaclesParameters_; }

protected:
    void chatterCallback(const geometry_msgs::PoseArray& msg)
    {
      this->obstaclesParameters2_ = ObstaclesParameters();
      this->obstaclesParameters2_.numberOfObstacles_ = msg.poses.size();
      this->obstaclesParameters2_.vectorOfObstacles_.resize(msg.poses.size()*4);

      for (size_t i = 0; i < msg.poses.size(); i+=4) {
        auto pose = msg.poses[i];
        this->obstaclesParameters2_.vectorOfObstacles_[i+0] = 0.3;  // radius
        this->obstaclesParameters2_.vectorOfObstacles_[i+1] = 1.;  //  height
        this->obstaclesParameters2_.vectorOfObstacles_[i+2] = pose.position.x;
        this->obstaclesParameters2_.vectorOfObstacles_[i+3] = pose.position.y;
      }
    }


 private:
  ObstaclesParameters obstaclesParameters_;
  ObstaclesParameters obstaclesParameters2_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

}  // namespace smb_path_following
