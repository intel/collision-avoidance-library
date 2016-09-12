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

# The SITL must be in the path.
# It is possible to install it with:
# $ ./modules/waf/waf-light install
SITL="arducopter-quad"

silentkill () {
	SIGNAL=-SIGKILL
	if [ ! -z $2 ]; then
		SIGNAL=$2
	fi
	kill $SIGNAL $1 > /dev/null 2> /dev/null
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
	cd $SCRIPT_DIR # sitl must run in the same dir of "eeprom.bin"
	$SITL --model x \
		> "${LOGDIR}/sitl.log" \
		2> "${LOGDIR}/sitlerr.log" &
	SITLID=$!
	cd - > /dev/null

	# Gazebo engine without GUI.
	# The log can be played through `gazebo -p logfile`
	SDFFILE="${SCRIPT_DIR}/worlds/${WORLD}"
	gzserver --verbose -r --record_path $LOGDIR $SDFFILE \
		> "${LOGDIR}/gzserver.log" \
		2> "${LOGDIR}/gzservererr.log" &
	GZID=$!

	# Wait for gazebo being up and running
	sleep 8

	# Bidirectional bridge between sitl (tcp) and gzsitl (udp)
	socat udp:localhost:14556 tcp:localhost:5760 \
		> "${LOGDIR}/gzsitl_socat.log" \
		2> "${LOGDIR}/gzsitl_socaterr.log" &
	GZSITL_SOCATID=$!

	# Wait for gzsitl-SITL connection to be stabilished
	sleep 2

	# Run the collision avoidance gcs
	../build/samples/coav_gcs --depth-camera GAZEBO --vehicle GAZEBO \
		> "${LOGDIR}/coav_gcs.log" \
		2> "${LOGDIR}/coav_gcserr.log" &
	COAVGCSID=$!

	# Wait for gcs being up and running
	sleep 4

	# Bidirectional bridge between sitl (tcp) and coav_gcs (udp)
	socat udp:localhost:14557 tcp:localhost:5762 \
		> "${LOGDIR}/coav_socat.log" \
		2> "${LOGDIR}/coav_socaterr.log" &
	COAV_SOCATID=$!

	# Wait for coav_gcs-SITL connection to be stabilished
	sleep 2

	sleep_until_takeoff 0.5 # Detect takeoff on a distance from origin >= 0.5 meters

	# Maximum acceptable time for this particular mission
	listen_collision $SLEEPTIME \
		&& echo "[${WORLD}] OK!" || echo "[${WORLD}] FAIL!"

	silentkill $GZID -SIGINT && sleep 3 # Wait gzserver to save the log
	silentkill $GZID  # Kill gzserver
	silentkill $GZSITL_SOCATID # Kill gzsitl socat
	silentkill $COAV_SOCATID # Kill coav_gcs socat
	silentkill $SITLID # Kill arducopter sitl
	silentkill $COAVGCSID # Kill coav_gcs sitl
}

runtests () {
	mkdir -p "${SCRIPT_DIR}/output"

	testcase simple.sdf 30
	testcase simple_obstacle.sdf 40
}

replay () {
	gazebo -p "${SCRIPT_DIR}/output/${1}/state.log"
}

if [ -z "$1" ]; then
	# TODO: add help text
	runtests # default subcommand
else
	$@ # replay scene-name
fi
