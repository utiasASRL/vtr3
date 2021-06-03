### Overview

The safety monitor is a single piece of code that sequentially checks a number of inputs, called safety_monitor_inputs.
Each safety monitor input provides a status for the path tracker such as:
1. Halt: the path tracker should stop and further discard the desired path as it is no longer necessary or passable (not currently used)
2. Pause:  the path tracker should stop but may resume tracking the current path in the near future
3. Slow: the path tracker should slow down to the provided speed
4. Continue:  All is well and the path tracker should continue like there's no tomorrow

The safety monitor may also check the liveness of the input, so if the status of the input is not updated as specified by
the creator of the input, the safety monitor could detect this and stop the rover.

Each safety monitor input can check multiple signal monitors.
Typically each signal monitor corresponds to one ROS2 msg callback.
Currently we support the following monitors:
- Deadman monitor: Requires a button on the handheld controller to be pressed for the vehicle to move in autonomous mode. 
  This adds an extra layer of safety when working near the robot.
- Localization monitor: Checks two signals - estimated uncertainty on our localization and number of keyframes since we successfully localized (dead-reckoning condition).
  It is expected the latter will be the main thing to slow and safely stop the robot when it cannot localize due to factors such as large appearance change.

### Contributing

To create a new input:

1. Create new monitor input files based on existing:
  - include/vtr_safety_monitor/inputs/my_new_monitor.hpp
  - src/inputs/my_new_monitor.cpp

2. Set the monitor class name to MyNewMonitorInput.

3. Create an entry for your new monitor in the monitor factory
  - src/safety_monitor_node.cpp -> SafetyMonitorFactory(...)
  - Add the following entry:

```c++
} else if (std::strcmp(string, "my_new_monitor") == 0){
   return new MyNewMonitorInput(static_cast<std::shared_ptr<Node>>(this));
```

4. Modify your safety monitor as necessary

5. Adjust the parameter files in the config directory to reflect the new safety monitor:

  - Update the list of monitors:
    ```yaml
    list_of_monitors: ["localization_monitor", "deadman_monitor", "my_new_monitor"]
    ```

  - Add the necessary ROS parameters







