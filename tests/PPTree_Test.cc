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

#include <utils/PPTree.hh>
#include "PPTree_Test.hh"
#include "Tests.hh"
#include <iostream>
#include <vector>

PPTree_Test::PPTree_Test()
{
}

PPTree_Test::~PPTree_Test()
{
}

bool PPTree_Test::run() {
    int num_nodes = 10;

    PPTree *tree = new PPTree(num_nodes);

    std::vector<int> expected_result;
    std::vector<int> result;

    if (!tree) {
        return false;
    }

    /* Test tree union. */
    tree->ds_union(1, 2);
    tree->ds_union(5, 8);
    expected_result = {0, 1, 1, 3, 4, 5, 6, 7, 5, 9};

    for (int i = 0; i < num_nodes; i++) {
        std::cout << tree->ds_find(i) << " ";
        result.push_back(tree->ds_find(i));
    }
    std::cout << std::endl;

    if(expected_result != result) {
        return false;
    }

    tree->ds_union(1, 3);
    expected_result = {0, 1, 1, 1, 4, 5, 6, 7, 5, 9};
    result.clear();

    for (int i = 0; i < num_nodes; i++) {
        std::cout << tree->ds_find(i) << " ";
        result.push_back(tree->ds_find(i));
    }
    std::cout << std::endl;

    if(expected_result != result) {
        return false;
    }

    tree->ds_union(5, 1);
    expected_result = {0, 5, 5, 5, 4, 5, 6, 7, 5, 9};
    result.clear();

    for (int i = 0; i < num_nodes; i++) {
        std::cout << tree->ds_find(i) << " ";
        result.push_back(tree->ds_find(i));
    }
    std::cout << std::endl;

    if(expected_result != result) {
        return false;
    }

    delete tree;

    return true;
}

