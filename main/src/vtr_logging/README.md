# vtr_logging

Logger for VTR3, which is simply a wrapper over [easylogging++ v9.97.0](https://github.com/amrayn/easyloggingpp) making it a ROS2 package with a convenient [configuration function](./src/configure.cpp).

## To Developers

- Include [logging_init.hpp](./include/vtr_logging/logging_init.hpp) in and only in where `int main(int, char**)` function is defined.
- Include [logging.hpp](./include/vtr_logging/logging.hpp) in other files using the logger.
- Take a look at the [easylogging++ documentation](https://github.com/amrayn/easyloggingpp) for its features and the [configuration function](./include/vtr_logging/configure.hpp) for how we configure it and options.
