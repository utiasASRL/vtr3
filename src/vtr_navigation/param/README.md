# ROS params for the asrl__navigation binaries

## Organisation
ROS params are organised by a three level heirarchy.
1. Default params are stored in the *base* directory. These are the default params for every vtr2 module and are always loaded.
1. Robot specific params are stored in the *robot* directory. Any params in thse files override the *base* params, otherwise, they default to the *base* value.
1. Scenario specific params are stored in the *scenario* directory. Any params in thse files override the *robot* or *base* params.

## Making a new scenario
1. Copy *asrl__navigation/param/TESTED-EXAMPLE_default.yaml* to a new file for your scenario in the same directory.
1. Edit the params as appropriate, and add more (taking into account namespaces) for any non-default params needed for your scenario in this params file.
1. Copy *asrl__navigation/launch/TESTED-EXAMPLE_default.launch* to a new file for your scenario in the same directory.
1. Change the scenario_params argument in this file to point to your scenario params, then specify the feature type and robot you will be running the binary on (these params determine which other params get loaded on launch).
