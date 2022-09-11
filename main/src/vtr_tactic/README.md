# VT&amp;R3 Tactic (State Estimation)

## Add a new pipeline

A pipeline is used to estimate motion and build map using various sensor data. The tactic (state estimation block) in VTR is sensor, robot and algorithm agnostic. In othe words, any pipeline can be plugged into VTR for odometry, localization and mapping.

To create a new pipeline: (TODO: this section is obsolete)

- add new header and implementation files in [include](./include/vtr_tactic/pipelines) and [src](./src/pipelines).
- use the [TemplatePipeline](./include/vtr_tactic/pipelines/template_pipeline.hpp) as a starting point to create the class of your pipeline.
- customize your pipeline to do the work following the instructions in [TemplatePipeline](./include/vtr_tactic/pipelines/template_pipeline.hpp).
- add the header file to [pipelines.hpp](./include/vtr_tactic/pipelines/pipelines.hpp).
- append the pipeline class to the `type_switch_` container in [PipelineFactory](./include/vtr_tactic/pipelines/pipeline_factory.hpp).
- if your pipeline needs to cache any intermediate data, such as image features or landmarks, put them in [QueryCache](./include/vtr_tactic/cache.hpp). Take a look at the source code of this class and the stereo pipeline to see how it works.
- use modules to structure your pipeline and properly balance computation across data preprocessing, odometry and mapping, and localization. Note that data preprocessing. odometry and mapping, and localization are by default run in parallel. Feel free to use extra threads inside the pipeline to perform expensive operations such as keyframe optimization or data saving.
- create unit tests for your pipeline.

## Add a new module

A module is a processing unit that should be created and called from a pipeline.

Suppose that you want to create a new module for the lidar pipeline

- Use the [TemplateModule](./include/vtr_tactic/modules/template_module.hpp) as a starting point to create the class of your module and put it inside the corresponding namespace, i.e. lidar.
- Copy [TemplateModule](./include/vtr_tactic/modules/template_module.hpp) header into the desired directory and rename it: `vtr_lidar/include/modules/<module name>.hpp`. Also create its implementation file (`<module name>.cpp`) in the respective directory: `vtr_lidar/src/modules/<module name>.cpp`
- Change function templates in the copied TemplateModule.
- Add directory of the header file to `vtr_lidar/include/modules/modules.hpp` for convenience.
- Register the new module in lidar pipline `vtr_lidar/src/pipeline.cpp: LidarPipeline::addModules()`.
- If possible, create unit-tests for your module. Greatly appreciated.
