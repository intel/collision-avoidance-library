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

import os
import signal
import sys


if __name__ == '__main__':
    timeout = int(sys.argv[1])

    def handler(signum, frame):
        sys.exit(0)

    signal.signal(signal.SIGALRM, handler)
    signal.signal(signal.SIGINT, handler)
    signal.alarm(timeout)

    data = ""
    while "contact" not in str(data): # 'data' has type 'bytes' on python3
        # using os.read istead of readiline because of the bug:
        # https://bugs.python.org/issue9504
        data = os.read(sys.stdin.fileno(), 4096)

    sys.exit(1)
