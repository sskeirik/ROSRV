STRUCTURE
=========

In order to build the simulation, the directory is
structured as a typical catkin project.

Usage
=====

ROS tools (like catkin, rosrun, roslaunch) recommend sourcing
shell scripts to setup the workspace. The provided makefile
is intended to build and run ROS nodes without the cluttering the user's
shell environment.

Targets in the makefile are prefixed as follows -

  * CREATE - Targets to create temporary workspaces where files are copied into

  * COPY - Targets to move necessary files into aforementioned workspaces

  * GEN - Generate monitor code from spec files

  * BUILD - Targets to detect changes and build monitoring infrastructure, generated
    monitors and simulations.

  * LAUNCH - Targets to launch ros nodes for simulation and mointoring


Thus, `make LAUNCH-isolated-monitor-node` will do the following -

  1. Create a temporary catkin workspace under `.build`

  2. Check for changes to RV-Monitor and ROSRV. For ROSRV,
     copy relevant files into the temporary workspace, via
     `COPY` prefixed rules.

  3. Check for changes to specs, and use `GEN` prefixed rules
     to generate monitoring code, and copy the generated files
     into the temporary workspace.

  4. Build the temporary workspace and external tools, if needed

  5. Set the enviroment (in a subshell) and launch the monitoring node


Makefile Structure
------------------

In order to make it easier to run and maintain, the makefile
is structured as -

```Makefile

LAUNCH-(functionality) : $(ST)/LAUNCH-(functionality).step

$(ST)/LAUNCH-(functionality).step : $(ST)/(prerequisites).step
    RECIPE
```

The `LAUNCH-(...)` is an `alias` target for usability as it avoids having to type
out entire paths. It has "one" dependency: a `.step` file
which serves as a timestamp.


`ST` is a makefile variable that holds the current `state` of the
build process via the timestamps. The name of the timestamp is derived from
the `alias` target.

Here's an example rule -

```Makefile

LAUNCH-test-simulation : $(ST)/LAUNCH-test-simulation.step

$(ST)/LAUNCH-test-simulation.step : $(ST)/CREATE-catkin-ws.step $(ST)/BUILD-test-simulation-sources.step
$(ST)/LAUNCH-test-simulation.step : $(ST)/LAUNCH-roscore.step
	(cd $(BUILD_DIR) \
	   && source devel/setup.bash \
	   && (ROS_MASTER_URI="http://localhost:12345/" rosrun water_tank_sim level_sensor &))

```

In the example `LAUNCH-test-simulation` is the alias with the single
`$(ST)/LAUNCH-test-simulation.step` as the only prerequisite.

The target `$(ST)/LAUNCH-test-simulation.step` depends on
3 other steps: a CREATE step, a BUILD step and another LAUNCH step,
and refers to them via the full path and timestamp.

In the makefile, the `alias` rule is placed directly above
the `timestamp` rule, with one newline separating them. Two newlines
are placed between every `alias-timestamp` rule pair. A timestamp
rule with a long prerequisites list is split (as is standard with makefiles) with no newlines between
split segments. The example above has such a rule.

