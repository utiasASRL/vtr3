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

To create a new module: (TODO: this section is obsolete)

- add new header and implementation files in [include](./include/vtr_tactic/modules) and [src](./src/modules).
- use the [TemplateModule](./include/vtr_tactic/modules/template_module.hpp) as a starting point to create the class of your module and put it inside the corresponding namespace, e.g. lidar, stereo.
- customize your module to do the work following the instructions in [TemplateModule](./include/vtr_tactic/modules/template_module.hpp).
- add the header file to [modules.hpp](./include/vtr_tactic/modules/modules.hpp).
- append the module class to the `type_switch_` container in `ModuleFactory` class in [module_factory.hpp](./include/vtr_tactic/modules/module_factory.hpp).
- create unit tests for your module.
