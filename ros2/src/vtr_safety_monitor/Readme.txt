Structure:
The safety monitor is a single piece of code that sequentially checks a number of inputs, called safety_monitor_inputs.
Each safety monitor input provides a status for the path tracker (and eventually Graph Nav) such as:
1) Halt: the path tracker should stop and further discard the desired path as it is no longer necessary or passable
2) Pause:  the path tracker should stop but may resume tracking the current path in the near future
3) Slow: the path tracker should slow down to the provided speed
4) Continue:  All is well and the path tracker should continue like there's no tomorrow

The safety monitor also checks the liveness of the input, so if the status of the input is not updated as specified by
the creator of the input, the safety monitor will detect this and stop the rover.


To create a new input:

1) Create new monitor input files based on existing "abcd" monitor
-- include/asrl/safety_monitor/abcd_monitor.hpp
-- inlcude/asrl/safety_monitor/implementation/abcd_monitor.cpp

2) Choose a monitor class name
-- e.g. vtr_monitor_input, deadman_monitor_input, etc

3) Create an entry for your new monitor in the monitor factory
-- src/safety_monitor_node.cpp -> SafetyMonitorFactory(...)
-- Add the following entry:

} else if (std::strcmp(string, "abcd_monitor") == 0){
   return new abcd_monitor_input(nh);

4) Include your files in the safety_monitor_node header file
-- include/asrl/safety_monitor/safety_monitor_node.hpp

5) Modify your safety monitor as necessary

6) Adjust the launch file to reflect the new safety monitor:

-- Update the list of monitors:
    <!-- List of Monitors to load -->
    <rosparam param="list_of_monitors">
       ["deadman_monitor","grizzly_diagnostics_monitor","path_tracker_monitor","vtr_monitor","abcd_monitor"]
    </rosparam>

-- Add the necessary rosparams
<!-- ABCD monitor params -->
    <remap from="/safety_monitor_node/in/blah" to="/beep"/>
    <param name="abcd_param" value="really good number"/>







