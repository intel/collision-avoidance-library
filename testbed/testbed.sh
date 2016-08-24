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
	kill -SIGKILL $1 > /dev/null 2> /dev/null || true
}

testcase () {
	# TODO: run avoidance strategy
	# TODO: check collision

	# Create logdir if it does not exist
	LOGDIR="${SCRIPT_DIR}/output/${1}"
	mkdir -p $LOGDIR

	# Run SITL Simulator
	cd $SCRIPT_DIR # sitl must run in the same dir of "eeprom.bin"
	$SITL --model x > "${LOGDIR}/sitl.log" 2> "${LOGDIR}/sitlerr.log" &
	SITLID=$!
	cd - > /dev/null

	# Gazebo engine without GUI.
	# The log can be played through `gazebo -p logfile`
	SDFFILE="${SCRIPT_DIR}/worlds/${1}"
	gzserver -r --record_path $LOGDIR $SDFFILE \
		> "${LOGDIR}/gzserver.log" 2> "${LOGDIR}/gzservererr.log" &
	GZID=$!

	# socat creates a bidirectional conversion between SITL and gzsitl plugin
	socat udp:localhost:14556 tcp:localhost:5760 \
		> "${LOGDIR}/socat.log" 2> "${LOGDIR}/socaterr.log" &
	SOCATID=$!

	sleep $2 # Maximum acceptable time for this particular mission

	kill -SIGINT $GZID > /dev/null || true # Wait gzserver to save the log.
	sleep 3 # Ensure that there is enough time to gzserver close the log fd.
	silentkill $GZID  # Kill gazebo
	silentkill $SOCATID # Kill socat
	silentkill $SITLID # Kill SITL

	echo "[${1}] "
}

runtests () {
	mkdir -p "${SCRIPT_DIR}/output"

	testcase simple.sdf 55
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
