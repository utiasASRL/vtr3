#pragma once

namespace vtr {
namespace tactic {
/** \brief Helper class that default-constructs objects by static_name trait */
template <class Base>
class FactoryTypeSwitch {
 public:
  using BasePtr = std::shared_ptr<Base>;
  using BaseCtorPtr = Base* (*)();
  using BaseCtorFunc = std::function<Base*()>;

  /**
   * \brief Registers a new derived class D of base B for default construction
   */
  template <class Derived>
  void add() {
    static_assert(std::is_base_of<Base, Derived>::value,
                  "D is not derived from B :(.");
    constructors_[Derived::static_name] =
        BaseCtorFunc([] { return dynamic_cast<Base*>(new Derived); });
  }

  /**
   * \brief search for the derived class with matching static_name trait
   * \param[in] static_name the static name of the derived class
   * \return base class pointer to the derived class( nullptr if not found)
   */
  BasePtr make(const std::string& static_name) const noexcept {
    auto type_ctor = constructors_.find(static_name);
    if (type_ctor == constructors_.end()) return nullptr;
    return BasePtr(type_ctor->second());
  }

 private:
  /** \brief a map from type_str trait to a constructor function */
  std::unordered_map<std::string, BaseCtorFunc> constructors_;
};
}  // namespace tactic
}  // namespace vtr