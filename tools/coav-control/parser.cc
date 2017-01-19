/*
// Copyright (c) 2017 Intel Corporation
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

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "coav-control.hh"

using namespace std;

struct {
    bool realsense;
    bool gazebo;
} control_features {
#ifdef HAVE_REALSENSE
    .realsense = true,
#else
    .realsense = false,
#endif

#ifdef HAVE_GAZEBO
    .gazebo = true,
#else
    .gazebo = false,
#endif
};

struct v_pair {
    string option;
    string val;
};

void print_help(void)
{
    cout << "Usage: coav-control [options]\n\n"
        "Options:\n"
        "  -d, --detect\n"
        "       Detection Algorithms. Can be one of the following:\n"
        "           DI_OBSTACLE\n"
        "           DI_STRAIGHT_LINE\n"
        "           DI_POLAR_HIST\n"
        "           DI_SIMPLE\n"
        "  -a, --avoidance\n"
        "       Avoidance Algorithm. Can be one of the following:\n"
        "           QC_SHIFT_AVOIDANCE\n"
        "           QC_STOP\n"
        "           QC_VFF\n"
        "  -s, --sensor\n"
        "       Vehicle Sensor. Can be one of the following:\n"
        "           ST_REALSENSE\n"
        "           ST_GAZEBO_REALSENSE\n"
        "  -q, --quiet\n"
        "       Supress info messages \n"
        "  -h, --help\n"
        "       Display this help and exit\n\n"
        "Example:\n"
        "  $ coav-control -d DI_POLAR_HIST -a QC_SHIFT_AVOIDANCE -s ST_REALSENSE"
        << endl;
}

string detect_to_name(detect_algorithm d)
{
    switch (d) {
        case DA_UNDEFINED:
            return string("DA_UNDEFINED");
        case DI_OBSTACLE:
            return string("DI_OBSTACLE");
        case DI_STRAIGHT_LINE:
            return string("DI_STRAIGHT_LINE");
        case DI_POLAR_HIST:
            return string("DI_POLAR_HIST");
        case DI_SIMPLE:
            return string("DI_SIMPLE");
    }

    return string("UNKOWN_VALUE");
}

detect_algorithm name_to_detect(string name)
{
    transform(name.begin(), name.end(), name.begin(), ::toupper);

    if (name == "DI_OBSTACLE") {
        return DI_OBSTACLE;
    } else if (name == "DI_STRAIGHT_LINE") {
        return DI_STRAIGHT_LINE;
    } else if (name == "DI_POLAR_HIST") {
        return DI_POLAR_HIST;
    } else if (name == "DI_SIMPLE") {
        return DI_SIMPLE;
    }

    return DA_UNDEFINED;
}

string avoidance_to_name(avoidance_algorithm a)
{
    switch (a) {
        case AA_UNDEFINED:
            return string("AA_UNDEFINED");
        case QC_SHIFT_AVOIDANCE:
            return string("QC_SHIFT_AVOIDANCE");
        case QC_STOP:
            return string("QC_STOP");
        case QC_VFF:
            return string("QC_VFF");
    }

    return string("UNKOWN_VALUE");
}

avoidance_algorithm name_to_avoidance(string name)
{
    transform(name.begin(), name.end(), name.begin(), ::toupper);

    if (name == "QC_SHIFT_AVOIDANCE") {
        return QC_SHIFT_AVOIDANCE;
    } else if (name == "QC_STOP") {
        return QC_STOP;
    } else if (name == "QC_VFF") {
        return QC_VFF;
    }

    return AA_UNDEFINED;
}

string sensor_to_name(sensor_type s)
{
    switch (s) {
        case ST_UNDEFINED:
            return string("ST_UNDEFINED");
        case ST_REALSENSE:
            return string("ST_REALSENSE");
        case ST_GAZEBO_REALSENSE:
            return string("ST_GAZEBO_REALSENSE");
    }

    return string("UNKOWN_VALUE");
}

sensor_type name_to_sensor(string name)
{
    transform(name.begin(), name.end(), name.begin(), ::toupper);

    if (name == "ST_REALSENSE") {
        return ST_REALSENSE;
    } else if (name == "ST_GAZEBO_REALSENSE") {
        return ST_GAZEBO_REALSENSE;
    }

    return ST_UNDEFINED;
}


vector<v_pair> get_arg_list(int argc, char* argv[])
{
    vector<v_pair> list;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] != '\0') {
            list.push_back((v_pair) {argv[i], ""});
        } else if (argv[i][0] != '-' && list.size() != 0) {
            v_pair& cur = list.back();
            string val = argv[i];
            cur.val += cur.val.size() != 0 ? " " + val : val;
        } else {
            cerr << "Invalid argument : " << argv[i] << endl;
            exit(-EINVAL);
        }
    }

    return list;
}

control_options parse_cmdline(int argc, char *argv[])
{
    vector<v_pair> list = get_arg_list(argc, argv);

    control_options opts = {
        .detect = DA_UNDEFINED,
        .avoidance = AA_UNDEFINED,
        .sensor = ST_UNDEFINED,
        .quiet = false,
    };

    for (v_pair p : list) {
        // Detect Algorithms
        if (p.option == "-d" || p.option == "--detect") {
            if (opts.detect != DA_UNDEFINED) {
                cerr << "ERROR: Multiple detect algorithms provided" << endl;
                exit(-EINVAL);
            }

            opts.detect = name_to_detect(p.val);

            if (opts.detect == DA_UNDEFINED) {
                cerr << "ERROR: Unkown detect algorithm value '" << p.val << "'" << endl;
                exit(-EINVAL);
            }

        // Avoid Algorithms
        } else if (p.option == "-a" || p.option == "--avoidance") {
            if (opts.avoidance != AA_UNDEFINED) {
                cerr << "ERROR: Multiple avoidance algorithms provided" << endl;
                exit(-EINVAL);
            }

            opts.avoidance = name_to_avoidance(p.val);

            if (opts.avoidance == AA_UNDEFINED) {
                cerr << "ERROR: Unkown avoidance algorithm value '" << p.val << "'" << endl;
                exit(-EINVAL);
            }

        // Sensor Type
        } else if (p.option == "-s" || p.option == "--sensor") {
            if (opts.sensor != ST_UNDEFINED) {
                cerr << "ERROR: Multiple sensors provided" << endl;
                exit(-EINVAL);
            }

            opts.sensor = name_to_sensor(p.val);

            if (opts.sensor == ST_UNDEFINED) {
                cerr << "ERROR: Unkown sensor value '" << p.val << "'" << endl;
                exit(-EINVAL);
            } else if ((opts.sensor == ST_REALSENSE && !control_features.realsense) ||
                    (opts.sensor == ST_GAZEBO_REALSENSE && !control_features.gazebo)) {
                cerr << "ERROR: Feature not  available '" << opts.sensor << endl;
                exit(-EINVAL);
            }

        // Quiet
        } else if (p.option == "-q" || p.option == "--quiet") {
            opts.quiet = true;

        //Help
        } else if (p.option == "-h" || p.option == "--help") {
            print_help();
            exit(0);
        } else {
            cerr << "ERROR: Invalid Option '" << p.option << "'" << endl;
            print_help();
            exit(-EINVAL);
        }
    }

    return opts;
}

