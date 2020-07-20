# vtr_navigation

This is a top-level package that contains the VTR2 assemblies/modules framework, the Navigator executable and its support files including launch and param files. This is probably the biggest and most frequently used and modified package in VTR2. It has lots of stuff. Below is an overview of the architecture, running instructions and where to start when making code changes.

## Table of Contents

- [vtr_navigation](#vtr_navigation)
  - [Table of Contents](#table-of-contents)
  - [Architecture and Terminology](#architecture-and-terminology)
  - [Contributing](#contributing)
    - [Making a new module](#making-a-new-module)
    - [Adding a new cache variable](#adding-a-new-cache-variable)

## Architecture and Terminology

The VTR2 architecture is designed to be as modular as possible. We are running VTR2 on many different robots, and have different requirements, which means we need to use different Visual Odometry (VO) and localisation pipelines. We will explain some of the concepts and terms we use in vtr_navigation here. You will see class names and comments throughout the code that reference these terms:

- **cache**. A large class/struct containing pointers to data that will be modified by *modules* in an *assembly*. There are two caches: the **map** cache and the **query** cache. The **query** cache primarily contains data that is being input to the pipeline, like images, IMU or GPS updates, and the data processed from those pieces of data. The **map** cache contains data primarily pulled from the map, data that we are trying to match data in the **query** cache to. Both the map and query caches are filled out with some data on input to a pipeline and its child assemblies, passed to each module in turn to add or modify data, and then the key parts saved out once the assembly has finished.
- **module**: An independent piece of code with a `run()` function that performs some activity with data in the pair of caches, either adding or changing data in the cache. Usually only has one source file and an associated header, located in the `vtr_navigation/src/modules/` and `vtr_navigation/include/vtr/navigation/modules/`
- **assembly**: An ordered list of modules, executed in order when the assembly is run. There are 5 specific assemblies in VTR2:
  1. *converter*: Run on every new pair of caches pushed through the _pipeline_. Pre-processes every image by converting colour spaces and extracting features.
  2. *quick_vo*: Run on every new pair of caches, after the _converter_. Usually performs feature matching, RANSAC, estimates a pose update and runs a vertex creation test as the last module.
  3. *refined_vo*: If the vertex test module in *quick_vo* asserts that a new vertex should be created, this module is run. Performs a windowed bundle adjustment. It's slow, so not run after every _quick_vo_ assembly.
  4. *localisation*: Performs localisation against the map. The heart of VTR2's repeating ability. Generally only run after *refined_vo*. It's also quite slow, so not run after every *quick_vo* assembly.
- **pipeline**: Pre-processes data, runs different assemblies depending on the current state in the VTR2 state machine and post-processes data including saving it. There are different pipelines for different states:
  1. *idle*: Do nothing. Doesn't process any data and discards everything in the cache. Usually for the grizzly when stopped.
  2. *branch*: Runs only *quick_vo* and *refined_vo*. Used when adding a new branch to the graph in 'teach' mode.
  3. *localization_search*: Uses the *localisation* assembly to try and localise on the path. Doesn't run VO.
  4. *merge*: Usually run after *branch*, when trying to loop-close during teach to a path already in the graph. Runs all assemblies.
  5. *metric_localisation*: Runs while repeating a path. Runs VO and localisation.
- **tactic**: A method of parallelising the *converter* and *quick_vo* assmblies. `BasicTactic` runs the converter and quick_vo assemblies in sequence. `ParallelTactic` runs the converter and quick_vo assemblies in parallel. i.e. while quick_vo is processing new data and performing a pose update, the next image is already being preprocessed and and new features are extracted in parallel.
- **factory**: Factories generate modules, assemblies and tactics from ros param files on executable startup.

## Contributing

The most likely way you will contribute code to VTR2 is to support a new robot or sensor. This will mean creating new modules, configuring a new assembly setup by modifying launch files, and tuning new parameters through a param file.

### Making a new module

- First, take a look at the modules in `vtr_navigation/src/modules/` and see if there is a module that is most similar to the module you want to create. Copy the module `.cpp` and corresponding `.h` file in `vtr_navigation/include/vtr/navigation/modules/` and rename them appropriately. Here we will call it `my_new_module`.
- Give your module a unique name in the `.h` file by editing this line (and remembering it):

  ```c++
  static constexpr auto type_str_ = "my_new_module_name";
  ```

- You may also pass this as the default argument to the `BaseModule`'s constructor so that you know your module is called at runtime.
- When a module is first run as part of an assembly, the module's `run()` function is called, with the *query_cache*, *map_cache* and *graph* as arguments. You will need to fill out the `run()` function to:
  - Check that the pointers in the cache to the data you want are valid before accessing them (i.e. there is data in this cache attached to this pointer) by calling `cache.data.is_valid()`. Depending on the data input to the pipeline, not all caches will have all data. i.e. a cache may have IMU data or images, but usually not both. If the data you want isn't in this cache, the function will just have to `return`.
  - If you have data that needs a new cache variable, follow the instructions in [Adding a new cache variable](#adding-a-new-cache-variable) (TODO: We should change this part for easy add/remove cache data.)
  - Access and process the data in the caches and/or graph
  - Insert the new or modified data back into the appropriate cache.

- When a pipeline has decided that the data cache you just processed is a keyframe, it will call the module's `updateGraph()`. Use this function to insert the important data from the cache into the graph. Take a look at the other modules to get an idea of how and when to do this.
- If the module needs to be configured on startup, make a `Config` inside the class definition of the hpp file with the right variables that you will use in the cpp file
- In `vtr_navigation/include/vtr/navigation/factories/ros_module_factory.h`, add a new member function to the `RosModuleFactory` class appropriatey named to configure your new module like `configureMyNewModule()`.
- In `vtr_navigation/src/factories/ros_module_factory.cpp`, add another ```else if``` to `RosModuleFactory::configureModule()` like so:

  ```c++
  else if (isType<MyNewModule>(type_str))
      configureMyNewModule(new_module);
  ```

- In `vtr_navigation/src/factories/ros_module_factory.cpp`, add the new member function `configureMyNewModule()` definition to the `RosModuleFactory` and fill out the variables in your config using the ROS param functions.
  - Add the new module header include definition to `vtr_navigation/include/vtr/navigation/modules/modules.h`
  - Add the type to `vtr_navigation/src/factories/module_factory.cpp` as :

    ```c++
    type_switch.add<MyNewModule>();
    ```

- In the launch file for the binary you will be using (such as `vtr_navigation/launch/navigator.xml`) you will need to add the module to the assembly you want it to run in by adding it to the `<rosparam param="$assembly_name$/modules">` list, where `$assembly_name$` is one of either converter, quick_vo, refined_vo, localization etc. For example (for quick_vo):

  ```xml
  <rosparam param="quick_vo/modules">
    [
      "recall",
      "matcher",
      "ransac",
      "steam",
      "my_new_module",     <!-- ADD THIS LINE with your new module name -->
      "vertex_test"
    ]
  </rosparam>
  ```

- Add the paths to the default base and robot params files under the assembly configuration like so:

  ```xml
  <!-- frame-frame matcher module params -->
  <rosparam command="load" ns="quick_vo/my_new_module" file="$(find vtr_navigation)/param/base/$assembly_name$/my_new_module.yaml"/>
  <rosparam command="load" ns="$assembly_name$/my_new_module" file="$(find vtr_navigation)/param/$(arg robot_type)/quick_vo/my_new_module.yaml"/>
  ```

  where `$assembly_name$` is one of either converter, quick_vo, refined_vo, localization etc. as before.

- You now need to add param files at the two locations referenced above i.e.:

  ```bash
  vtr_navigation/param/base/$assembly_name$/my_new_module.yaml
  vtr_navigation/param/$robot_type$/$assembly_name$/my_new_module.yaml
  ```

  where `$assembly_name$` is as before, and `$robot_type$` is one of either grizzly, trex, lancaster, m600 etc. The *base* file must contain a default for each of the parameters in your config. The *robot_type* file can be blank (but MUST exist), and can overwrite the base values stored in default for a specific robot. You can further overwrite these params in your specific scenario file that should live in `vtr_navigation/param/scenarios/my_scenario_file.yaml`. This three-level heirarchy means your top level scenario params *overwrite* robot params that *overwrite* base params (if they are defined in the higher level file).

- Run `catkin build` in the vtr2 *src* directory with a `--force-cmake` command to make it find your new source and header files.

### Adding a new cache variable

If you have a new sensor or want to operate on a new type of data with a module, you will likely have to add a new variable to either the map cache or query cache. Follow these general instructions to do so:

- In `vtr_navigation/include/vtr/navigation/Caches.hpp`, determine whether your variable should live in the *map* cache or *query* cache. Generally, the query cache contains data directly gathered from a sensor and the products directly generated from it. e.g. images and features extracted from the image. The map cache generally contains data that is loaded from the graph e.g. 3D points associated with a vertex that you are trying to match your features (stored in a query cache) to.

- Add a special `cache_ptr` to your variable type to the appropriate cache in `Caches.hpp` and name it appropriately:

  ```c++
  common::cache_ptr<variable_type> my_new_cache_data;
  ```

- Add a constructor for your variable to the cache constructor in `Caches.hpp`:

  ```c++
  my_new_cache_data("my_new_cache_data",janitor_.get()),
  ```

- Add a class definition near the top of `Caches.hpp` if one doesn't already cover your new variable. We do this instead of including the appropriate header to speed up build times.
- In `vtr_navigation/src/Caches.cpp`, add a class template to match the new class definition (if necessary):

  ```c++
  template class cache_ptr<variable_type>;
  ```

- Run `catkin build` in the vtr2 _src_ directory to compile with your new cache variables
