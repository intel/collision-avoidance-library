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

