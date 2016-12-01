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

/**
 * @file
 * @brief Simple detector for depth inputs
 */

#include <memory>
#include <vector>

#include "common/common.hh"
#include "common/DepthCamera.hh"

/**
 * @brief Simple detector based on depth data
 */
class DepthImageSimpleDetector : public Detector<DepthCamera, bool>
{
  public:
    DepthImageSimpleDetector(std::shared_ptr<DepthCamera> depth_camera);
    const std::vector<bool> &detect() override;

  private:
    std::vector<bool> detection = {false};
};

