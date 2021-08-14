/**
 * \file logging_init.hpp
 * \brief
 * \details note: include and only include this header in main.cpp.
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_logging/easylogging++.h"
// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

#include <vtr_logging/configure.hpp>