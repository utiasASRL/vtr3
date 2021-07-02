#pragma once

#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/** \brief Interface for a generic tactic state publisher */
class PublisherInterface {
 public:
  /** \brief Sets the path followed by the robot for UI update */
  virtual void publishPath(const LocalizationChain &chain) const = 0;
  /** \brief Clears the path followed by the robot for UI update */
  virtual void clearPath() const = 0;
  /** \brief Updates robot messages for UI */
  virtual void publishRobot(
      const Localization &persistentLoc, uint64_t pathSeq = 0,
      const Localization &targetLoc = Localization(),
      const std::shared_ptr<rclcpp::Time> stamp = nullptr) const = 0;
};

}  // namespace tactic
}  // namespace vtr
