# The ROS2 Turtle Driver

The simple ROS2 node which control a turtle moving to the particular point.

The code is distributed under the GNU General Public License (version 3).

## Requirements

* Python 3.x
* ROS 2
* hsm-interfaces package - https://github.com/kruzhok-team/hsm_robot_ros_interfaces
* python3-transforms3d

## Running

```
ros2 launch turtle_driver start.launch.py                  # with a window
ros2 launch turtle_driver start.launch.py headless:=true   # with no display
```

## The Tests

The `test` directory holds the L2 tier of the framework: the driver node alone, driven through
the ROS2 objects it declares, with the fixtures of the `hsm_test_utils` package.

```
colcon test --packages-select turtle_driver --pytest-args -m node
```

The driver is also the robot of the system tier, where the diagrams are driven against the real
turtlesim simulator. The testing architecture, and how to add a test to any tier, are described
in the testing project - https://github.com/kruzhok-team/hsm_robot_ros_tests,
`docs/hsm_robot_testing.md` and `docs/howto_write_a_test.md`.
