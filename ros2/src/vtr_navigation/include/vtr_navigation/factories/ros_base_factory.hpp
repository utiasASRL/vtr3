#pragma once

#include "rclcpp/rclcpp.hpp"

#include <vtr_logging/logging.hpp>
#include <vtr_navigation/factories/base_factory.hpp>

namespace vtr {
namespace navigation {

/** \brief Base factory for creating module, assembly and tactic from ROS
 * parameters.
 */
template <class T>
class ROSBaseFactory : public BaseFactory<T> {
 public:
  using NodePtr = std::shared_ptr<rclcpp::Node>;
  using base_t = BaseFactory<T>;

  /** \brief constructor with ros info detailing the factory */
  ROSBaseFactory(const NodePtr node, const std::string& param_prefix)
      : node_(node),
        param_prefix_(param_prefix) /*,
        parent_prefix_(ros::names::parentNamespace(param_prefix)) */
  {}

  /** \brief Make a module/assembly/tactic */
  typename base_t::T_ptr make() const {
    std::string param_name{param_prefix_ + "." + type_field_};
    auto type_str = node_->declare_parameter<std::string>(param_name, "");
    if (type_str.empty()) {
      auto msg = "No field: '" + param_name + "'";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }
    return make_str(type_str);
  }

 protected:
  const NodePtr node_;
  const std::string param_prefix_;
  const std::string parent_prefix_;

 private:
  static constexpr auto type_field_ = "type";

  /** \brief Make a module/assembly from its name.
   * \todo I don't like the current code as it requires to create a
   * ROSXXXFactory for each of module, assembly and tactic. If factory is
   * basically a wrapper that extracts parameters for building each module,
   * then we should make it a wrapper and only have 1 class for that.
   */
  virtual typename base_t::T_ptr make_str(const std::string& name) const = 0;
};

}  // namespace navigation
}  // namespace vtr
