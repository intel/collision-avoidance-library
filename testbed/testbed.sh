#!/usr/bin/env bash
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

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
SIM_ARGS="-d DI_OBSTACLE -a QC_SHIFT_AVOIDANCE -s ST_GAZEBO_REALSENSE"

silentkill () {
    if [ ! -z $2 ]; then
        kill $2 $1 > /dev/null 2>&1 || true
    else
        kill -KILL $1 > /dev/null 2>&1 && wait $1 2> /dev/null || true
    fi
}

sleep_until_takeoff () {
    gz topic -u -e /gazebo/default/gzsitl_quadcopter_rs/vehicle_pose \
        | python detect_takeoff.py $1 $2 # Detect takeoff on distance $1
}

listen_collision () {
    SLEEPTIME=$1
    TOPIC=/gazebo/default/gzsitl_quadcopter_rs/sensors/contact/contacts
    gz topic -u -e $TOPIC | python detect_collision.py $SLEEPTIME
}

testcase () {
    WORLD=$1
    SLEEPTIME=$2

    WORLD=$WORLD LOGDIR="${SCRIPT_DIR}/output/${1}" ./coav-sim.sh $SIM_ARGS > /dev/null 2>&1 &
    COAVSIM_ID=$!
    sleep 30

    # Detect takeoff on a distance from origin >= 0.5 meters
    # Fail test if no takeoff is detect after 5 seconds
    sleep_until_takeoff 0.5 10
    if [ $? -ne 0 ]; then
        echo "[${WORLD}] FAIL! Takeoff not detected in time"
        cleanup
        return
    fi

    # Maximum acceptable time for this particular mission
    listen_collision $SLEEPTIME \
        && echo "[${WORLD}] OK!" || echo "[${WORLD}] FAIL! Collision detected"

    cleanup
}

runtests () {
    mkdir -p "${SCRIPT_DIR}/output"

    testcase simple.sdf 30
    testcase simple_obstacle.sdf 40
}

replay () {
    gazebo -p "${SCRIPT_DIR}/output/${1}/state.log"
}

cleanup () {
    silentkill -TERM $COAVSIM_ID # Terminate coav-sim.sh
    wait $COAVSIM_ID # Just because we know TERM will exit coav-sim.sh
}

cleanup_and_exit () {
    cleanup
    exit 0
}

trap cleanup_and_exit SIGINT SIGTERM

if [ -z "$1" ]; then
    # TODO: add help text
    runtests # default subcommand
else
    $@ # replay scene-name
fi
