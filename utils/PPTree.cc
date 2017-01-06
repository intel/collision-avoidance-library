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

#include <cstdlib>
#include "PPTree.hh"

Set::Set()
    : parent(this)
{
}

Set *Set::repr()
{
    if (!this->parent || this->parent == this)
        return this->parent = this;

    return this->parent = this->parent->repr();
}

PPTree::PPTree(int num_nodes)
    : sets(num_nodes)
{
}

int PPTree::ds_find(int node_id)
{
    return this->find(node_id) - this->sets.data();
}

Set *PPTree::find(int node_id)
{
    return this->sets[node_id].repr();
}


void PPTree::ds_union(int a, int b)
{
    Set::join(&this->sets[a], &this->sets[b]);
}

PPTree::~PPTree()
{
}

void Set::join(Set *a, Set *b)
{
    a = a->repr();
    b = b->repr();

    if (a == b) {
        /* Already united. */
        return;
    }

    /* Unite the roots of the nodes. */
    if (a->rank < b->rank) {
        a->parent = b;
        return;
    }

    b->parent = a;
    if (a->rank == b->rank)
        a->rank += 1;
}
