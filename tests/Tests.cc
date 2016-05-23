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

#include <memory>
#include <iostream>

#include "Tests.hh"
#include "PPTree_Test.hh"

int main(int argc, char **argv) {

    // Instantiate tests
    std::unique_ptr<Test> pptree_test(new PPTree_Test());

    // Run tests
    if(!pptree_test->run()) {
        std::cout << "Err: PPTree_Test failed" << std::endl;
    }
}
