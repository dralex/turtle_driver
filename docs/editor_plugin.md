# The HSM Turtle Plugin of CyberiadaPro

The `hsm-turtle` plugin makes a diagram the behaviour of the turtle of turtlesim: it checks the
diagram for the turtle, generates and builds the `hsm_controller` ROS 2 package and drives the
turtle with it, bringing back the path the turtle went. It is written against the editor's plugin
interface, level 1 (`docs/PLUGIN-API.md` of CyberiadaHSM-Editor), next to the `hsm-ros2` plugin
(`hsm_robot_ros_generator/docs/editor_plugin.md`) whose adapter it shares.

| | |
|---|---|
| id | `hsm-turtle` |
| name | `HSM Turtle`, in Russian `Черепаха ПРИМС` |
| folder | `editor-plugin/` of this repository |
| platforms | Linux with ROS 2 Jazzy |
| needs | `turtlesim`, `turtle_driver`, `hsm_generator`, `hsm_tests`, `hsm_robot`, `hsmd` |

## Architecture

```
 CyberiadaPro               editor-plugin/          hsm_generator share     hsm_tests
+-------------------+ argv +-------------+ exec   +-------------+  exec  +------------------+
| Plugins >         |----->| turtle_env  |------->| ros2_env    |------->| hsm_turtle_run   |
|  HSM Turtle       |      | source setup|        |  --ros-exec |        |  --check         |
|  check  (diag.)   |      | find        |        |  hsm_genera-|------->| run: generate,   |
|  generate (files) |      | ros2_env in |        |  tor        |   |    |  build, drive,   |
|  run    (files)   |      | AMENT_PREFIX|        +-------------+   |    |  trace           |
+-------------------+      +-------------+                          |    +------------------+
   ^ diagnostics.json, WS/src/hsm_controller, TRACE/<name>.csv|.svg  |             | a random domain
   +----------------------------------------------------------------+             v
                         6 API nodes <-> turtle_driver <-> turtlesim_node (window)
                         WS/install/hsm_controller  ---- the diagram
```

The plugin folder holds no copy of `ros2_env`: `turtle_env` sources the `setup` parameter and
execs the adapter `share/hsm_generator/editor-plugin/ros2_env` of the first prefix of
`AMENT_PREFIX_PATH` which has one.

## Actions

| id | result | command after `turtle_env --setup ${param:setup}` | exit_ok | timeout |
|---|---|---|---|---|
| `check` | `diagnostics` | `--ros-exec hsm_tests hsm_turtle_run ${input} --check --json ${diagnostics}` | 0, 1 | 60 s |
| `generate` | `files` (`outdir: ws`) | `hsm_generator ${input} -o ${outdir}/src/hsm_controller -f -q` | 0 | 60 s |
| `run` | `files` (`outdir: trace`) | `--ros-exec hsm_tests hsm_turtle_run ${input} --workspace ${param:ws} --trace ${outdir} --name ${document_name} --duration ... --pose ... [--headless]` | 0, 1 | none |

* `check` reports the generator's finding first; a diagram the generator accepts gets the
  turtle warnings below. Warnings do not stop a run.
* `generate` is the one of `hsm-ros2`: both plugins share the workspace by default.
* `run` generates and builds the package into the workspace, starts the API nodes, turtlesim
  and the driver, teleports the turtle to the start pose, and starts the built controller. The
  build and the event timeline stream into the output tab. The run ends with the duration, when
  the controller ends, or on Cancel; the duration 0 runs until Cancel. Every node is stopped.
* `run` writes the path of the turtle into the trace folder as `<document name>.csv`
  (`t,x,y,theta`) and `.svg` (the world at the turtlesim scale and colours, the path, the start
  and the turtle). The editor lists them when the run ends by itself; a cancelled run writes
  them too, but the editor drops a cancelled run, so they are not listed.
* All three actions are on the plugin toolbar and read `${input}`, unsaved edits included.

## Parameters

| id | type | scope | default | used by |
|---|---|---|---|---|
| `setup` | `file` | `persistent` | `/opt/ros/jazzy/setup.bash` | all |
| `ws` | `dir`, `create`, `required` | `document` | `${document_dir}/ros2_ws` | generate, run |
| `trace` | `dir`, `create`, `required` | `document` | `${document_dir}/turtle-trace` | run |
| `duration` | `int`, min 0 | `document` | 60 | run: the limit, s; 0 - until Cancel |
| `x`, `y` | `double`, 0..11.088889 | `document` | 5.544445 | run: the start position, turtlesim's spawn |
| `theta` | `double` | `document` | 0 | run: the start heading, rad |
| `headless` | `bool` | `document` | false | run: no simulator window |

## The turtle checks

| rule | severity | element | finds |
|---|---|---|---|
| `turtle-scan` | warning | the transition or state | a trigger on `COLLISION_WARNING` or `RIGHT_OPEN_SPACE`: the navigation node raises them from `/scan`, and the driver publishes no scan |
| `turtle-pump` | warning | the state or transition | a `Pump` call: the turtle has no pump |
| `turtle-world` | warning | the state or transition | a literal point of `move_to_point` or `move_along_traj` outside the world 0..11.09 |
| `turtle-test` | info | the comment | a `ros2 hsm test` rule expecting a scan event |

`COLLISION_DETECTED` is not reported: the driver raises it when the turtle stops progressing to
its goal, e.g. at a wall.

## Install

| How | Where the editor finds it |
|---|---|
| `colcon build` of this package | `install/turtle_driver/share/turtle_driver/editor-plugin` - add it with `--plugin-path` or `CYBERIADA_PRO_PLUGIN_PATH` |
| `ros2 run hsm_tests hsm_turtle_run --install-editor-plugin [DIR]` | `DIR/hsm-turtle`, by default `~/.local/share/Cyberiada/CyberiadaPro/plugins/hsm-turtle` |

The installer (`hsmplugin.install` of the generator) pins `HSM_GENERATOR` and the `setup`
default to the environment it runs from. The plugin needs the `hsm_tests` runner at run time,
but this package does not depend on it: `hsm_tests` depends on `turtle_driver`.

## Test

```
colcon test --packages-select turtle_driver --pytest-args -m unit
colcon test --packages-select hsm_tests --pytest-args -m "unit or system"
CyberiadaPro --batch examples/right_hand_maze_solver.graphml --plugin-path turtle_driver \
    --run-plugin hsm-turtle:check --dump-plugin
```

`test/test_editor_plugin.py` covers the descriptor, the translation and `turtle_env`; the runner
is tested in `hsm_robot_ros_tests` (`docs/hsm_robot_testing.md` §11).
