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

#include <vector>

struct Set {
    Set *parent;
    int rank = 0;

    Set();
    Set *repr();
    static void join(Set *a, Set *b);
};

class PPTree
{
  public:
    PPTree(int num_nodes);
    ~PPTree();

    void ds_union(int node_x_id, int node_y_id);
    int ds_find(int node_id);

  private:
    Set *find(int node_id);
    std::vector<Set> sets;
};

