#!/usr/bin/env bash
# Copyright (c) 2017 Intel Corporation
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

# Set autopilot
# Supported Autopilots: AP_PX4 and AP_APM
AUTOPILOT=${AUTOPILOT:-"AP_PX4"}

# ArduCopter Variables
APM_CMD="${APM_DIR:+"${APM_DIR}/"}arducopter"
APM_TCP_PORT_1=5760
APM_TCP_PORT_2=5762

# PX4 Variables
PX4_DIR=${PX4_DIR:-"~/px4/Firmware"}
PX4_CMD="make posix_sitl_default jmavsim"
PX4_UDP_PORT_1=14550
PX4_UDP_PORT_2=14540

# Simulation Parameters
WORLD=${WORLD:-"simple_obstacle.sdf"}

# Collision Avoidance and Gazebo variables
GZSITL_UDP_PORT=15556
COAV_GCS_UDP_PORT=15557

run_autopilot () {
    # Run SITL Simulator
    if [ "$AUTOPILOT" = "AP_PX4" ]; then
        cd $PX4_DIR
        $PX4_CMD &
        SITLID=$!
        cd - > /dev/null

        # Wait for sitl
        sleep 15
    elif [ "$AUTOPILOT" = "AP_APM" ]; then
        cd $SCRIPT_DIR # sitl must run in the same dir of "eeprom.bin"
        $APM_CMD --model x &
        SITLID=$!
        cd - > /dev/null

        # Wait for sitl
        sleep 5
    fi
}

run_gazebo () {
    # Gazebo engine without GUI.
    # The log can be played through `gazebo -p logfile`
    SDFFILE="${SCRIPT_DIR}/worlds/${WORLD}"
    gzserver --verbose $SDFFILE &
    GZID=$!

    # Wait until gazebo is up and running
    sleep 8

    SOCAT_ARG_2="udp:localhost:$GZSITL_UDP_PORT"
    if [ "$AUTOPILOT" = "AP_PX4" ]; then
        SOCAT_ARG_1="udp-listen:$PX4_UDP_PORT_1"
    elif [ "$AUTOPILOT" = "AP_APM" ]; then
        SOCAT_ARG_1="tcp:localhost:$APM_TCP_PORT_1"
    fi

    # Bidirectional bridge between autopilot and gazebo-sitl
    socat $SOCAT_ARG_1 $SOCAT_ARG_2 &
    GZSITL_SOCATID=$!

    # Wait for gzsitl-sitl connection to be stabilished
    sleep 2
}

run_coav_control() {
    # Run the collision avoidance
    ../build/tools/coav-control/coav-control "$@" &
    COAVID=$!

    # Wait until is up and running
    sleep 4

    SOCAT_ARG_2="udp:localhost:$COAV_GCS_UDP_PORT"
    if [ "$AUTOPILOT" = "AP_PX4" ]; then
        SOCAT_ARG_1="udp-listen:$PX4_UDP_PORT_2"
    elif [ "$AUTOPILOT" = "AP_APM" ]; then
        SOCAT_ARG_1="tcp:localhost:$APM_TCP_PORT_2"
    fi

    # Bidirectional bridge between the autopilot and the coav_gcs
    socat $SOCAT_ARG_1 $SOCAT_ARG_2 &
    COAV_SOCATID=$!
}


simulate () {
    run_autopilot
    run_gazebo
    run_coav_control "$@"

    # wait forever
    cat
}

silentkill () {
    if [ ! -z $2 ]; then
        kill $2 $1 > /dev/null 2>&1 || true
    else
        kill -KILL $1 > /dev/null 2>&1 && wait $1 2> /dev/null || true
    fi
}

cleanup_and_exit () {
    silentkill $GZSITL_SOCATID # Kill gzsitl socat
    silentkill $COAV_SOCATID # Kill coav_gcs socat
    silentkill $SITLID # Kill sitl
    silentkill $GZID -INT && sleep 3 # Wait gzserver to save the log
    silentkill $GZID  # Kill gzserver
    silentkill $COAVID # Kill coav-control sitl
    exit 0
}

test_dep () {
    command -v $1 > /dev/null 2>&1 || { echo >&2 "Error: command '$1' not found"; exit 1; }
}

check_deps () {
    test_dep gzserver
    test_dep socat
    test_dep $APM_CMD
}

test_dir () {
    [ -d $1 ] > /dev/null 2>&1 || { echo >&2 "Error: directory '$1' not found"; exit 1; }
}

check_dirs () {
    test_dir $SCRIPT_DIR
    if [ "$AUTOPILOT" = "AP_PX4" ]; then
        test_dir $PX4_DIR
    fi
}

trap cleanup_and_exit SIGINT SIGTERM

check_deps
check_dirs

simulate "$@"
