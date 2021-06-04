# VT&amp;R3 Tactic (State Estimation)

## Add a new module

A module is a processing unit that should be created and called from a pipeline.

To create a new module:

- add new header and implementation files in [include](./include/vtr_tactic/modules) and [src](./src/modules).
- use the [TemplateModule](./include/vtr_tactic/modules/template_module.hpp) as a starting point to create the class of your module and put it inside the corresponding namespace, e.g. lidar, stereo.
- customize your module to do the work following the instructions in `TemplateModule`.
- add the header file to [modules.hpp](./include/vtr_tactic/modules/modules.hpp).
- append the module class to the `type_switch_` container in `ModuleFactory` class in [module_factory.hpp](./include/vtr_tactic/modules/module_factory.hpp).

To create a new pipeline:
