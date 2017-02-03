# Collision Avoidance Library #

A framework for testing and benchmarking collision avoidance strategies.

## Requirements ##
    * Gazebo 7.0+ (for virtual camera/vehicle support)
    * socat 1.7+ (for testbed support)
    * GZSitl (for virtual vehicle support)
    * Autopilot, either:
      * Ardupilot (https://github.com/ArduPilot/ardupilot) or
      * PX4 (https://github.com/PX4/Firmware)

## Build and Install ##

1. Make sure you have initialized and updated all the required submodules at
least once with:

    ```
    git submodule update --init --recursive
    ```

2. Create a "build" folder and build the library using CMAKE as follows:

    ```
    mkdir build
    cd build
    cmake ..
    sudo make install
    cd -
    ```

    These instructions will build the targets and install gazebo plugins and
    models into the default gazebo installation folder. To select a custom
    installation path for gazebo plugins and models, call cmake with the
    following line instead:

    ```
    cmake -D COAVLIB_MODEL_INSTALL_PATH=<model_install_path> -D COAVLIB_PLUGIN_INSTALL_PATH=<plugin_install_path> ..
    ```

    Make sure you have set the proper gazebo environment variables for your
    plugins/models to be found in your custom path.

## Run Testbed ##

1. The testbed uses ArduPilot as autopilot by default. Set the following
environment variables prior to running testbed to use PX4 instead.

    ```
    export PX4_DIR="your/px4/directory/Firmware"
    export USE_PX4_AUTOPILOT=1
    ```

    Before running the testbed, make sure PX4 sitl and jmavsim have already
    been built by following the instructions on PX4 documentation.

2. The testbed.sh will automate the execution of a series of missions, given as
world files in `testbeds/worlds/<world_name>.sdf`. The script will start
gazebo, the autopilot (APM or PX4), and the collision avoidance gcs
automatically, outputting the result and the logs of each mission, as they're
executed.

    The missions are simple gazebo world files composed by at least a
    gzsitl_perm_target and a gzsitl_quadcopter_rs. Two test missions are
    provided as samples:

    ```
    testbed/worlds/simple.sdf
    testbed/worlds/simple_obstacle.sdf
    ```

    They are called by testbed.sh runtests() function in the following lines:

    ```
    runtests () {
        mkdir -p "${SCRIPT_DIR}/output"

        testcase simple.sdf 30
        testcase simple_obstacle.sdf 60
    }
    ```

    The first argument to testcase() is the name of the world file and the
    second argument is the maximum time a mission will run before moving to the
    next. You're invited to create your own missions and include them as you
    wish inside runtests() to have them executed.

    In orther to execute the testbed, make sure you have all the dependencies
    installed and run the following commands **from the testbed directory**.

    ```
    testbed.sh
    ```

3. To replay a previously executed mission in gazebo, make sure **your current
folder is testbed** and simply execute the following command:

    ```
    ./testbed.sh replay <world_file>.sdf
    ```

    For example:

    ```
    ./testbed.sh replay simple.sdf
    ```

    If the `<world_file>.sdf` mission has been already executed by testbed.sh,
    the logs necessary to replay the mission will be available to gazebo.

## Deploying on Intel Aero ##

Intel Aero firmware is based on Yocto, so the Yocto SDK for Intel Aero will be
used to properly compile Collision Avoidance Library for deploy on Intel Aero.

Instruction on how to build Intel Aero image and the associated SDK can be found
on Intel Aero [Wiki](https://github.com/intel-aero/meta-intel-aero/wiki).

Intel Aero SDK will be missing two of the Collision Avoidance Library
dependencies:
  * GLM
  * MavLink

Since both are "headers only" libraries, cmake just need to know where to find
the headers in order to successfully cross-compile the library. This will be
done with "-DCMAKE_PREFIX_PATH" parameter as described by the instructions
bellow.

Once Intel Aero SDK is successfully installed, the following instructions will
configure the environment and compile the library:

    source <SDK_PATH>/environment-setup-core2-64-poky-linux

    mkdir build
    cd build
    cmake .. -DCMAKE_PREFIX_PATH="<GLM_HEADERS_PATH>:<MAVLINK_HEADERS_PATH>"
    make

If MavLink and GLM can be found under the same path, one entry will be enough.

After a successful build, you can install Collision Avoidance Library in a
temporary path:

    make install DESTDIR=<TMP_PATH>

Pack everything:

    cd <TMP_PATH>
    tar cvf coav.tar *

Copy coav.tar to Intel Aero root dir and execute the following on Intel Aero:

    [intel-aero]$ cd /
    [intel-aero]$ tar xvf coav.tar

And Collision Avoidance Library should be successfully installed!
