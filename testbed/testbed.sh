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

# Supported Autopilots
AP_PX4=0
AP_APM=1

# ArduCopter Variables
APM_CMD="arducopter-quad"
APM_TCP_PORT_1=5760
APM_TCP_PORT_2=5762

# PX4 Variables
PX4_DIR=${PX4_DIR:-"~/px4/Firmware"}
PX4_CMD="make posix_sitl_default jmavsim"
PX4_UDP_PORT_1=14550
PX4_UDP_PORT_2=14540

# Collision Avoidance and Gazebo variables
GZSITL_UDP_PORT=15556
COAV_CONTROL_UDP_PORT=15557

# Set autopilot
AUTOPILOT=${TESTBED_AUTOPILOT:-AP_PX4}

# Check if a supported autopilot has been selected
if (("$AUTOPILOT" != "$AP_PX4" && "$AUTOPILOT" != "$AP_APM")); then
    echo >&2 "Error: Autopilot '$AUTOPILOT' is not supported"
    exit 1
fi

silentkill () {
    if [ ! -z $2 ]; then
        kill $2 $1 > /dev/null 2>&1 || true
    else
        kill -KILL $1 > /dev/null 2>&1 && wait $1 2> /dev/null || true
    fi
}

test_dep () {
    command -v $1 > /dev/null 2>&1 || { echo >&2 "Error: command '$1' not found"; exit 1; }
}

check_deps () {
    test_dep gz
    test_dep socat
    test_dep $APM_CMD
}

test_path () {
    [ -d $1 ] > /dev/null 2>&1 || { echo >&2 "Error: directory '$1' not found"; exit 1; }
}

check_paths () {
    test_path $SCRIPT_DIR
    if (("$AUTOPILOT" == "$AP_PX4")); then
        test_path $PX4_DIR
    fi
}

sleep_until_takeoff () {
    gz topic -u -e /gazebo/default/gzsitl_quadcopter_rs/vehicle_pose \
        | python detect_takeoff.py $1 # Detect takeoff on distance $1
}

listen_collision () {
    SLEEPTIME=$1
    TOPIC=/gazebo/default/gzsitl_quadcopter_rs/sensors/contact/contacts
    gz topic -u -e $TOPIC | python detect_collision.py $SLEEPTIME
}

testcase () {
    WORLD=$1
    SLEEPTIME=$2

    # Create logdir if it does not exist
    LOGDIR="${SCRIPT_DIR}/output/${1}"
    mkdir -p $LOGDIR

    # Run SITL Simulator
    if (("$AUTOPILOT" == "$AP_PX4")); then
        cd $PX4_DIR
        $PX4_CMD > "${LOGDIR}/sitl.log" \
            2> "${LOGDIR}/sitlerr.log" &
        SITLID=$!
        cd - > /dev/null
    elif (("$AUTOPILOT" == "$AP_APM")); then
        cd $SCRIPT_DIR # sitl must run in the same dir of "eeprom.bin"
        $APM_CMD --model x \
            > "${LOGDIR}/sitl.log" \
            2> "${LOGDIR}/sitlerr.log" &
        SITLID=$!
        cd - > /dev/null
    fi

    # Wait until gazebo is up and running
    sleep 8

    # Gazebo engine without GUI.
    # The log can be played through `gazebo -p logfile`
    SDFFILE="${SCRIPT_DIR}/worlds/${WORLD}"
    gzserver --verbose -r --record_path $LOGDIR $SDFFILE \
        > "${LOGDIR}/gzserver.log" \
        2> "${LOGDIR}/gzservererr.log" &
    GZID=$!

    # Wait until gazebo is up and running
    sleep 8

    SOCAT_ARG_2="udp:localhost:$GZSITL_UDP_PORT"
    if (("$AUTOPILOT" == "$AP_PX4")); then
        SOCAT_ARG_1="udp-listen:$PX4_UDP_PORT_1"
    elif (("$AUTOPILOT" == "$AP_APM")); then
        SOCAT_ARG_1="tcp:localhost:$APM_TCP_PORT_1"
    fi

    # Bidirectional bridge between autopilot and gazebo-sitl
    socat $SOCAT_ARG_1 $SOCAT_ARG_2 \
        > "${LOGDIR}/gzsitl_socat.log" \
        2> "${LOGDIR}/gzsitl_socaterr.log" &
    GZSITL_SOCATID=$!

    # Wait for gzsitl-sitl connection to be stabilished
    sleep 2

    # Run the collision avoidance app
    ../build/tools/coav-control/coav-control -s ST_GAZEBO_REALSENSE -a QC_SHIFT_AVOIDANCE -d DI_POLAR_HIST \
        > "${LOGDIR}/coav-control.log" \
        2> "${LOGDIR}/coav-controlerr.log" &
    COAVCONTROLID=$!

    # Wait until the app is up and running
    sleep 4

    SOCAT_ARG_2="udp:localhost:$COAV_CONTROL_UDP_PORT"
    if (("$AUTOPILOT" == "$AP_PX4")); then
        SOCAT_ARG_1="udp-listen:$PX4_UDP_PORT_2"
    elif (("$AUTOPILOT" == "$AP_APM")); then
        SOCAT_ARG_1="tcp:localhost:$APM_TCP_PORT_2"
    fi

    # Bidirectional bridge between the autopilot and the coav-control
    socat $SOCAT_ARG_1 $SOCAT_ARG_2 \
        > "${LOGDIR}/coav_socat.log" \
        2> "${LOGDIR}/coav_socaterr.log" &
    COAV_SOCATID=$!

    # Wait for coav-control-SITL connection to be established
    sleep 2

    sleep_until_takeoff 0.5 # Detect takeoff on a distance from origin >= 0.5 meters

    # Maximum acceptable time for this particular mission
    listen_collision $SLEEPTIME \
        && echo "[${WORLD}] OK!" || echo "[${WORLD}] FAIL!"

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
    silentkill $GZSITL_SOCATID # Kill gzsitl socat
    silentkill $COAV_SOCATID # Kill coav-control socat
    silentkill $SITLID # Kill sitl
    silentkill $GZID -INT && sleep 3 # Wait gzserver to save the log
    silentkill $GZID  # Kill gzserver
    silentkill $COAVCONTROLID # Kill coav-control sitl
}

cleanup_and_exit () {
    cleanup
    exit 0
}

trap cleanup_and_exit SIGINT SIGTERM

check_deps
check_paths

if [ -z "$1" ]; then
    # TODO: add help text
    runtests # default subcommand
else
    $@ # replay scene-name
fi
