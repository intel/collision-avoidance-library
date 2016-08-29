#!/usr/bin/env python
# Copyright (c) 2016 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import sys
import re

def dist(x, y, z):
    return math.sqrt(x * x + y * y + z * z)

if __name__ == '__main__':
    takeoff_distance = float(sys.argv[1])
    pos_re = re.compile("position\ \{\ *x\:(.+)y\:(.+)z\:(.+)\}\ orientation.*")

    for line in sys.stdin:
        m = pos_re.match(line)
        if not m:
            continue
        x, y, z = m.groups()
        if dist(float(x), float(y), float(z)) >= takeoff_distance:
            exit(0)

    exit(1)
