# Collision Avoidance Library #

A framework for testing and benchmarking collision avoidance strategies.

## Requirements ##
    * Gazebo 7.0+ (for virtual camera/vehicle support)
    * socat 1.7+ (for testbed support)
    * GZSitl (for virtual vehicle support)
    * Ardupilot (https://github.com/ArduPilot/ardupilot)

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

1. The testbed.sh will automate the execution of a series of missions, given as
world files in `testbeds/worlds/<world_name>.sdf`. The script will start gazebo,
ardupilot and the collision avoidance gcs automatically, outputting the result
and the logs of each mission, as they're executed.

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
        testcase simple_obstacle.sdf 40
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

2. To replay a previously executed mission in gazebo, make sure **your current
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

