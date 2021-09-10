# (TODO: Yuchen) Contributing Guide

- pull-request and review work-flow
  - never push directly to `main` branch
  - never merge branches that fail in Jenkins (build and unit-tests)
  - must get approved from at least 1 reviewer from within ASRL
  - breaking changes must be tested on a robot
  - prefer small PR focus on a particular feature/fix
- GitHub issue management
  - in general, will not be monitored frequently
  - mostly only deal with major bugs
- development best practices
  - use style defined in `.clang-format` for C++ and `.style.yapf` for Python
  - write unit-tests for changes
  - follow verbosity policy (TODO: debug, info, warning, error)
  - write at least one high level comment per method and class on the purpose unless the method name is entirely self explanatory.
  - avoid comments that don't contain additional information beyond the obvious.
