#pragma once

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace navigation {

/** \brief base factory that enforces verification of build products */
template <class T>
class BaseFactory {
 public:
  using T_ptr = std::shared_ptr<T>;

  /** \brief public access to make a verified product */
  T_ptr makeVerified() {
    auto new_T = make();
    if (new_T && !new_T->verify()) {
      auto msg = "Assembly builder failed verification :(.";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }
    return new_T;
  }

 protected:
  /** \brief compare type_str traits, helper for factories who want more control
   * \param[in] type_str the type_str trait that might match derived class D
   * \return true if the trait is a match
   */
  template <class D>
  static bool isType(const std::string& type_str) {
    return type_str.compare(D::type_str_) == 0;
  }

 private:
  /** \brief the requested module type_str trait, the use case of this is not
   * clear const
   */
  std::string type_str_;

  /** \brief build function to be implemented by subclasses */
  virtual T_ptr make() const = 0;
};

/** \brief Helper class that default-constructs objects by type_str trait */
template <class B>
class FactoryTypeSwitch {
 public:
  using B_ptr = std::shared_ptr<B>;
  using B_ctor_p = B* (*)();
  using B_constructor_func = std::function<B*()>;

  /** \brief register a new derived class D of base B for default construction
   */
  template <class D>
  void add() {
    static_assert(std::is_base_of<B, D>::value, "D is not derived from B :(.");
    constructors_[D::type_str_] =
        B_constructor_func([] { return dynamic_cast<B*>(new D); });
  }

  /** \brief search for the derived class with matching type_str trait
   * \param[in] type_str the string name of the derived class
   * \return base class pointer to the derived class( nullptr if not found)
   */
  B_ptr make(const std::string& type_str) const noexcept {
    auto type_ctor = constructors_.find(type_str);
    if (type_ctor == constructors_.end()) return nullptr;
    return B_ptr(type_ctor->second());
  }

 private:
  /** \brief a map from type_str trait to a constructor function */
  std::unordered_map<std::string, B_constructor_func> constructors_;
};

}  // namespace navigation
}  // namespace vtr
