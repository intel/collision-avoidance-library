/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#pragma once

#include <common/common.hh>
#include <common/DepthCamera.hh>
#include <memory>
#include <vector>

class DepthImagePolarHistDetector : public Detector<DepthCamera>
{
  public:
    DepthImagePolarHistDetector(std::shared_ptr<DepthCamera> depth_camera,
            double angle_step, double threshold = 5.0, double density = 0.1);
    const std::vector<Obstacle> &detect() override;

  private:
    double step;
    double threshold;
    double density;
    std::vector<Obstacle> obstacles;
};
