#pragma once

#include <ros/ros.h>

#include <vtr/navigation/factories/ros_base_factory.h>
#include <vtr/navigation/tactics/base_tactic.h>
#include <vtr/navigation/tactics/basic_tactic.h>  // \todo replaced by above
// #include <vtr/navigation/tactics/ParallelTactic.hpp>

namespace vtr {
namespace navigation {

/** /brief Make a module based on ros configuration
 *
 * /todo this class should use template BaseTactic rather than BasicTactic.
 * Check this after we figure out what basic and parallel tactics are doing.
 */
class ROSTacticFactory : public ROSBaseFactory<BasicTactic> {
 public:
  using base_t = ROSBaseFactory<BasicTactic>;
  using tac_ptr = base_t::T_ptr;

  /** \brief constructed with ros param info
   * \param[in] nh the ros nodehandle with the params
   * \param[in] param_prefix the prefix (namespace) in the param path
   */
  ROSTacticFactory(nh_t* nh, const std::string& param_prefix)
      : base_t(nh, param_prefix){};

 private:
  /** \brief constructs a module based on ros params
   * \param[in] type_str the type_str trait of the requested module
   */
  tac_ptr make_str(const std::string& type_str) const;
};

}  // namespace navigation
}  // namespace vtr
