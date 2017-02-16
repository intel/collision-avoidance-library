# Collision Avoidance Library #

A framework for testing and benchmarking collision avoidance strategies.

## Requirements ##

 * Gazebo 7.0+ (for virtual camera/vehicle support)
 * socat 1.7+ (for testbed support)
 * GZSitl (for virtual vehicle support)
 * Mavlink (https://github.com/mavlink/mavlink)
 * GLM (https://github.com/g-truc/glm.git)
 * Autopilot, either:
   * Ardupilot (https://github.com/ArduPilot/ardupilot) or
   * PX4 (https://github.com/PX4/Firmware)

Collision Avoidance Library (Coav) is developed having drones in mind, so when compiling
the library without additional options, features related to benchmark and simulation
will be OFF by default. This should be the preferred way when you want to ship
the library on your drone target/product.

### Library Features and Options ###

Collision Avoidance Library has support to following features that can be defined
on compile time:

Feature/Option          | Compile Options | Default Value
----------------------- | --------------- | -------------
Intel RealSense support | WITH_REALSENSE  | ON
Gazebo support          | WITH_GAZEBO     | OFF
Visual Debugger support | WITH_VDEBUG     | OFF (depends on Gazebo)
Coav Tools              | WITH_TOOLS      | OFF
Compile code samples    | WITH_SAMPLES    | OFF

## Build and Install ##

1. Make sure you have initialized and updated all the required submodules at
least once with:

    ```
    git submodule update --init --recursive
    ```

2. Create a "build" folder and build the library using CMake as follows:

    ```
    mkdir build
    cd build
    cmake ..
    sudo make install
    cd -
    ```

    These instructions will build and install the targets on cmake's
    default install path (usually '/usr/local'). To modify the library options,
    the following syntax is used when issuing `cmake`:

    ```
    cmake .. -D<COMPILE_OPTION_1>=<OPTION_1_VALUE> -D<COMPILE_OPTION_2>=<OPTION_2_VALUE>
    ```

    Also, the following CMake options may be of value:

    Option | Description
    --- | ---
    CMAKE_INSTALL_PREFIX | Set a custom installation path. This path is also used for dependance search.
    CMAKE_PREFIX_PATH | Add paths to be searched when looking for dependances

    A more complete explanation of those options can be found on CMake's Documentation.

    Example:

      * Search GLM and Mavlink on <custom_deps_path>
      * Change the install path to <custom_install_path>
      * Compile the library additional tools (coav-sim)

    ```
    cmake .. -DCMAKE_INSTALL_PREFIX=<custom_install_path> -DCMAKE_PREFIX_PATH=<custom_deps_path> -DWITH_TOOLS=ON
    ```

## Testing Collision Avoidance Library with *coav-control* ##

Make sure that the library was compiled with 'Coav Tools' turned on. This will
build a target `coav-control` that can be found in 'tools/coav-control/' inside
the build folder.

`coav-control` can be used execute a simple collision avoidance system for a
Mavlink controlled Quadcopter that is composed by: a sensor, a detection algorithm
and a detection strategy.

The following will list the possible options for each component:

```
./coav-control --help
```

Example:

Run a collision avoidance system composed by:
 * Intel Realsense
 * Obstacle detector based on 'Blob extraction'
 * 'Stop' avoidance strategy

```
./coav-control -d DI_OBSTACLE -a QC_STOP -s ST_REALSENSE
```

## Build and deploy on Intel Aero ##

This section explains how to compile the collision avoidance library locally in
your x86-64 system for deployment on Intel Aero.

1. Intel Aero currently supports the collision avoidance binaries only if they
are compiled with gcc-5. Older versions have not been tested and gcc-6 is
currently not supported. Before deploying the collision avoidance binaries on
Intel Aero, please make sure you're using gcc-5.

    If your gcc version is older than gcc-5, please update it before compiling
    for Intel Aero. If your default compiler is gcc-6, install gcc-5 in your
    system and set the following environmnet variables before building:


    ```
    export CXX=/bin/g++-5
    export CC=/bin/gcc-5
    ```

    To build, use cmake as explained in [Build and Install](#build-and-install).

2. After compiling, you may deploy your generated binaries to Intel Aero by
copying your files to the board using scp, as follows:

    Make sure Intel Aero is turned on and connected to your machine through
    usb. Wait for a new virtual ethernet connection to show up and when that
    happens, manually set the ip of the connection to 192.168.7.1

    With the connection set up, you can easily copy the binaries of your
    collision avoidance gcs to Intel Aero using scp. To copy the simple_gcs
    sample to the Intel Aero home directory, for example, use the following
    command line:

    ```
    scp -p build/samples/simple_gcs root@192.168.7.1:~
    ```

    To run your collision avoidance gcs, remotely log into Intel Aero with ssh
    as follows:

    ```
    ssh root@192.168.7.2
    ```

    Then run your collision avoidance gcs. Per our example:

    ```
    ~\simple_gcs
    ```

    As soon as you start running your app, it will try to connect to the
    vehicle and to the sensors you have configured. If it succeeds, your
    collision avoidance strategy and the vehicle's autopilot will start running
    in parallel.

## Run Testbed ##

Make sure that Gazebo Support was ON during compile and that you have set the
proper gazebo environment variables for your plugins/models to be found in your
custom path (via GAZEBO_PLUGIN_PATH environment variable).

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
gazebo, the autopilot (APM or PX4), and the coav-control automatically,
outputting the result and the logs of each mission, as they're
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
