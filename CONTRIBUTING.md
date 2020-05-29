# Contributing guideline

Note: The old contributing guide is [here](https://github.com/utiasASRL/vtr2), but mostly on how to add new robots.

The following guideline applies when pulling changes from a feature branch back to `devel` branch. Do not submit changes to `master` branch directly.

## Before submitting a pull request

Make sure your code conforms to Google's [C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and [Python Style Guide](https://google.github.io/styleguide/pyguide.html).
- For C++, run [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) and [clang-format](https://clang.llvm.org/docs/ClangFormat.html) on your source code files. A [.clang-format](./.clang-format) file has been created in this repo, which basically asks clang to format your code according to Google style.
- For Python, run [pylint](https://www.pylint.org/) and [yapf](https://github.com/google/yapf) on your source code files. Use the following flags for yapf: `--style={based_on_style: google, indent_width: 2, column_limit: 80}`

The only exception is our C++ naming convention, which is slightly different from Google style. Use the following naming convention for C++:
- ClassName, functionName, variable_name, private_variable_name_, header.h, implementation.cpp, templates.inc

Some extra notices and tips:
- Remove trailing whitespaces.
- Indent using spaces instead of tabs.
- C++ code should target C++17, and Python code should target Python3.8. No backward compatibility needed.
- For C++:
  - Use `auto` whenever possible.
  - Prefer `using` over `typedef`, e.g. `using ptr = std::shared_ptr`
  - Prefer `class` over `typename` in templates, e.g. `template<class T>`
  - Use `#pragma once` or `guards` for header files.
  - Do not include unused headers. Also do not let reader follow a chain of `#include`s in order to find a declaration, i.e. include the header file for any type/variable being used, even if it is included in another header indirectly.
  - If some code is left there for later use, use `#if 0 ... #endif` instead of commenting it out.
  - Use `filesystem` and ROS's `ros::names` libraries to handle system directories.
  - It's OK to use `pragma` to suppress warnings, but a clear reason should be provided in comment.
  - Let your code describe what it does, rather than using comments. Avoid comments like `// Construct a pipeline`.

Make sure your code is clearly documented and captured by Doxygen. Take a look at the examples in [vtr_documentation](./src/vtr_documentation).

Create unit tests for any new features. Unit tests should follow [ROS Unit Testing](http://wiki.ros.org/Quality/Tutorials/UnitTesting), which uses [gtest](https://github.com/google/googletest) for C++ and [unittest](https://docs.python.org/3/library/unittest.html) for Python.

If necessary, create a flow/regression test in the [vtr_testing](./src/vtr_testing) ROS package.

Make sure that your changes pass all unit tests and regression tests. If there are any expected test failures, disable the tests and provide reason for that.

## Sumitting a pull request

Choose proper reviewers to review your change. Ideally each pull request should have at least one reviewer, even if you own all files that have been touched.

Mention the following in pull request:
- New features being added
- Unit tests and regression tests being added
- Any breaking changes