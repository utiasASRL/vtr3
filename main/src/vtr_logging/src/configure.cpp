#include "vtr_logging/configure.hpp"

namespace vtr {
namespace logging {

void configureLogging(const std::string& log_filename, const bool debug,
                      const std::vector<std::string>& enabled) {
  // Logging flags
  el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);
  el::Loggers::addFlag(el::LoggingFlag::LogDetailedCrashReason);
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  // Set default configuration
  el::Configurations config;
  config.setToDefault();
  // clang-format off
  config.setGlobally(el::ConfigurationType::Format, "%datetime{%H:%m:%s.%g} %thread %level [%fbase:%line] [%logger] %msg");
  config.set(el::Level::Debug, el::ConfigurationType::Enabled, debug ? "true" : "false");
  // clang-format on
  if (!log_filename.empty()) {
    config.setGlobally(el::ConfigurationType::Filename, log_filename);
    config.setGlobally(el::ConfigurationType::ToFile, "true");
  }

  el::Loggers::setDefaultConfigurations(config, true);
  LOG_IF(!log_filename.empty(), INFO) << "Logging to: " << log_filename;
  LOG_IF(log_filename.empty(), WARNING) << "NOT LOGGING TO A FILE.";
  LOG_IF(!enabled.empty(), INFO) << "Enabled loggers: " << enabled;

  if (!enabled.empty()) {
    // disable all loggers except WARNING and ERROR
    el::Configurations disable_config;
    // clang-format off
    disable_config.setGlobally(el::ConfigurationType::Enabled, "false");
    disable_config.set(el::Level::Warning, el::ConfigurationType::Enabled, "true");
    disable_config.set(el::Level::Error, el::ConfigurationType::Enabled, "true");
    // clang-format on
    el::Loggers::setDefaultConfigurations(disable_config, true);
    // enable specified loggers
    for (const auto& id : enabled) el::Loggers::reconfigureLogger(id, config);
  }
}

}  // namespace logging
}  // namespace vtr
