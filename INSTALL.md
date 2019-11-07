# Installing ROSRV

Here are instructions for installing and building ROSRV by checking out its
source code on GitHub.

## Prerequisites

### Core Prerequisites

1. [ROS Kinetic](http://wiki.ros.org/kinetic)
2. [PyTest](https://docs.pytest.org/)
3. [Maven](https://maven.apache.org)
4. [Catkin](http://wiki.ros.org/catkin)

### RV-Monitor

1. [RV-Monitor](https://github.com/runtimeverification/rv-monitor)
    To use RV-Monitor supported logics RV-Monitor install RV-Monitor
    using the instructions at the tool's page (use `mvn install`
    to install snapshot to local maven repository).
    Set environment variable `RVMONITOR` to point to `rv-monitor's`
    root directory.

## Install and Build

1. After Installing Prerequisites, run `./Test` to
   Build and Run Tests.

2. The default behavior of the testing setup is to run
   all tests. Use `PYTEST_FLAGS="--skip-dl-tests"` to
   skip Differential Dynamic Logic based tests.

### Running
ROSRV can simply be treated as a catkin project, and
dropped into an existing catkin workspace.

The `bin/rosrv` provides a wrapper to the
C++ code generator. For example `bin/rosrv <SPEC_FILE> -o [MONITOR_NAME]`
will generate a monitor `MONITOR_NAME` which can be run
via catkin as `catkin run rvmonitor <MONITOR_NAME>`.

See [docs/Usage.md](docs/Usage.md) for further information on how to run ROSRV.
Get help or report problems on
[ROSRV's issues page](https://github.com/FormalSystemsLaboratory/ROSRV/issues).
