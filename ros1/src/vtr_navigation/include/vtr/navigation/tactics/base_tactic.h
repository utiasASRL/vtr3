#pragma once

namespace vtr {
namespace navigation {

/** brief Supposed to be the base class of tactic. API for a tactic is not
 * clear.
 */
class BaseTactic {
 public:
  BaseTactic() {}
  virtual ~BaseTactic() {}
  bool verify() { return true; }
};

}  // namespace navigation
}  // namespace vtr
