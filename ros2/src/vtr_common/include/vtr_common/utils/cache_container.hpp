/// Helper classes for passing optional named arguments.
/// CacheContainer.impl includes the reference-required functions so that
/// the cache classes can be declared with incomplete types.
/// Include it if you (a) don't care about that functionality or
/// (b) are trying to explicitly instantiate the classes.
#pragma once

// #include <vtr_logging/logging.hpp>
#include <vtr_common/utils/type_traits.hpp>

// cache pointer
#include <future>
#include <memory>
#include <stdexcept>

// janitor
#include <algorithm>
#include <list>

// ostream overloads
#include <iostream>
#include <ostream>

namespace vtr {
namespace common {

class cache_base;
////////////////////////////////////////////////////////////////////////////////
/// Provides cache cleanup utilities, knows about all cache variables.
////////////////////////////////////////////////////////////////////////////////
class cache_janitor {
 public:
  typedef std::list<cache_base*> list_t;
  typedef list_t::iterator it_t;

  /// Run a function for each registered cache.
  /// @param[in] f the function to run. examples:
  ///     auto l_display = [](cache_base* p) { std::cout << *p << std::endl; };
  ///     auto l_cleanup = [](cache_base* p) { p->cleanup(); };
  template <class F>
  void for_each_cache(F&& f) {
    std::for_each(caches_.rbegin(), caches_.rend(), f);
  }

 private:
  /// @brief add a cache element to the janitor (on construction)
  it_t add_cache(cache_base* cache) {
    caches_.push_front(cache);
    return caches_.begin();
  }

  /// @brief remove a cache element to the janitor (on destruction)
  void remove_cache(const it_t& cache_it) { caches_.erase(cache_it); }

  /// @brief the list of cache elements on which to operate
  list_t caches_;

  // The cache base needs to add and remove cache references
  friend class cache_base;
};

////////////////////////////////////////////////////////////////////////////////
/// A cache container with a helpful janitor inside.
////////////////////////////////////////////////////////////////////////////////
struct CacheContainer {
  // typedefs
  typedef cache_janitor janitor_t;
  typedef std::unique_ptr<cache_janitor> janitor_ptr_t;

  /// @brief get the janitor to perform functions for each cache element
  cache_janitor& janitor() { return *janitor_; }

  // hide the janitor
 protected:
  CacheContainer() : janitor_(new janitor_t) {}
  CacheContainer(const CacheContainer& other) =
      delete;  // non construction-copyable
  CacheContainer& operator=(const CacheContainer&);  // non copyable

  const janitor_ptr_t janitor_;
};

////////////////////////////////////////////////////////////////////////////////
/// Base class for cache variables that can be cleaned up by a janitor.
////////////////////////////////////////////////////////////////////////////////
class cache_base {
 public:
  // noncopyable
  cache_base(const cache_base&) = delete;
  cache_base& operator=(const cache_base&) = delete;

  /// @brief Delete any stale data
  virtual void cleanup() = 0;

  /// @brief stream out data
  virtual std::ostream& operator<<(std::ostream& os) const = 0;

  /// @returns whether the cache is valid (initialized)
  virtual bool is_valid() const = 0;
  explicit operator bool() const { return is_valid(); }

  /// Clears and invalidates the cache (if possible).
  virtual cache_base& clear() = 0;

 protected:
  /// @brief register with the janitor, and set the cache name
  cache_base(cache_janitor* janitor, const std::string& name = std::string())
      : name_(name), janitor_(&*janitor) {
    if (!janitor) throw std::invalid_argument("janitor is empty.");
    janitor_it_ = janitor_->add_cache(this);
  }
  /// @brief remove registration with the janitor
  virtual ~cache_base() {
    if (!janitor_) throw std::runtime_error("the janitor disappeared!");
    janitor_->remove_cache(janitor_it_);
  }

  /// @brief the channel name
  std::string name_;

 private:
  /// @brief the visiting janitor (so we can remove ourselves from its care)
  cache_janitor* janitor_;
  /// @brief our iterator in the janitors list (for easy removal)
  cache_janitor::it_t janitor_it_;
  // friend ostream insertion
  friend std::ostream& operator<<(std::ostream&, const cache_base&);
};

/// @brief the ostream operator overload, calls the virtual member
std::ostream& operator<<(std::ostream& os, const cache_base& me);

////////////////////////////////////////////////////////////////////////////////
/// Templated parent class for accessing cache variables
////////////////////////////////////////////////////////////////////////////////
template <typename Type>
class cache_accessor : public cache_base {
 public:
  typedef cache_accessor<Type> my_t;

  /// Constructor for a cache accessor.
  /// @param[in] janitor required for cleanup and container member registration
  /// @param[in] name string name for displaying in log messages
  cache_accessor(cache_janitor* janitor,
                 const std::string& name = std::string())
      : cache_base(janitor, name) {}

  //////////////////////////////////////////////////////////////////////////////
  // Setters

  /// Create the provided default if the cache doesn't exist.
  /// @param[in] that the default cache value
  virtual my_t& fallback(const Type& datum) = 0;
  virtual my_t& fallback(Type&& datum) = 0;

  /// Assign to cache, which is created if it doesn't exist.
  virtual my_t& operator=(Type&& datum) = 0;
  virtual my_t& operator=(const Type& datum) = 0;

  /// Empty and invalidate the cache if possible.
  virtual my_t& clear() = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Getters

  /// Dereference container to underlying type.
  /// @throws runtime_error if the cache is not initialized
  virtual const Type& operator*() const = 0;
  Type& operator*() {
    return const_cast<Type&>(*static_cast<const my_t&>(*this));
  }
  const Type* operator->() const { return &(this->operator*()); }
  Type* operator->() {
    return const_cast<Type*>(static_cast<const my_t&>(*this).operator->());
  }

  //////////////////////////////////////////////////////////////////////////////
  // Virtual overrides for the janitor

  /// @brief cleanup (not implemented)
  void cleanup() {
    throw std::runtime_error("cache_accessor::cleanup not implemented");
  }

  /// Call the global stream operator for template specialization.
  std::ostream& operator<<(std::ostream& os) const;
};

////////////////////////////////////////////////////////////////////////////////
/// A cache shared future that can be cleaned up by a janitor.
////////////////////////////////////////////////////////////////////////////////
template <typename Type>
class cache_future : public cache_accessor<Type> {
 public:
  typedef cache_future<Type> my_t;
  typedef cache_accessor<Type> parent_t;
  typedef std::shared_future<Type> future_t;

  //////////////////////////////////////////////////////////////////////////////
  // Constructors

  /// @brief constructor for a cache pointer (empty unless Guaranteed)
  /// @param[in] janitor required for cleanup and container member registration
  template <typename... Args>
  cache_future(cache_janitor* janitor) : parent_t(janitor) {}

  /// Constructor for a cache pointer (empty unless Guaranteed).
  /// @param[in] janitor required for cleanup and container member registration
  /// @param[in] name string name for displaying in log messages
  /// @param[in] args optional constructor arguments for guaranteed caches
  template <typename... Args>
  cache_future(const std::string& name, cache_janitor* janitor)
      : parent_t(janitor, name) {}

  //////////////////////////////////////////////////////////////////////////////
  // Setters

  /// Assign a new future to the cache.
  my_t& operator=(const future_t& datum) {
    datum_ = datum;
    return *this;
  }
  my_t& operator=(future_t&& datum) {
    datum_ = std::move(datum);
    return *this;
  }

  /// Assign a value to the cache, disconnecting from any promise.
  my_t& operator=(Type&& datum) { return assign(std::move(datum)); }
  my_t& operator=(const Type& datum) { return assign(datum); }

  /// Create the provided default if the cache doesn't exist.
  /// @param[in] args the default cache value
  template <typename... Args>
  my_t& fallback(Args&&... args) {
    if (!is_valid()) assign(std::forward<Args>(args)...);
    return *this;
  }
  my_t& fallback(const Type& datum) { return fallback<const Type&>(datum); }
  my_t& fallback(Type&& datum) { return fallback<Type&&>(std::move(datum)); }

  /// Reset the future, invalidating the cache.
  my_t& clear() {
    datum_ = {};
    return *this;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Getters

  /// Dereference container to underlying type.
  /// @throws runtime_error if the cache is not initialized
  const Type& operator*() const {
    if (is_valid())
      return const_cast<future_t&>(datum_).get();
    else {
      /// LOG(ERROR) << el::base::debug::StackTrace();
      throw std::runtime_error("cache datum '" + name_ +
                               "' was invalid on reference request.");
    }
  }
  using parent_t::operator*;
  using parent_t::operator->;

  /// @returns whether the cache is valid.
  bool is_valid() const { return datum_.valid(); }

  /// Get a ref to the underlying future
  const future_t& future() const { return datum_; }
  future_t& future() { return datum_; }

 protected:
  /// Assign a value to the cache, disconnecting from any promise.
  template <typename... Args>
  my_t& assign(Args&&... args) {
    // fill a temporary promise to pass on to the future
    std::promise<Type> temp;
    temp.set_value(std::forward<Args>(args)...);
    datum_ = temp.get_future();
    return *this;
  }

  std::shared_future<Type> datum_;
  using cache_base::name_;
};

////////////////////////////////////////////////////////////////////////////////
/// A cache shared pointer that can be cleaned up by a janitor.
/// @note Guaranteed -> will always be allocated and never throw
////////////////////////////////////////////////////////////////////////////////
template <typename Type, bool Guaranteed = false>
class cache_ptr : public cache_accessor<Type> {
 public:
  // Typedefs
  typedef cache_ptr<Type, Guaranteed> my_t;
  typedef cache_accessor<Type> parent_t;
  typedef std::shared_ptr<Type> shared_ptr_t;

  //////////////////////////////////////////////////////////////////////////////
  // Constructors

  /// @brief constructor for a cache pointer (empty unless Guaranteed)
  /// @param[in] janitor required for cleanup and container member registration
  template <typename... Args>
  cache_ptr(cache_janitor* janitor, Args&&... args) : parent_t(janitor) {
    make_shared_if_guaranteed<Guaranteed>(std::forward<Args>(args)...);
  }

  /// Constructor for a cache pointer (empty unless Guaranteed).
  /// @param[in] janitor required for cleanup and container member registration
  /// @param[in] name string name for displaying in log messages
  /// @param[in] args optional constructor arguments for guaranteed caches
  template <typename... Args>
  cache_ptr(const std::string& name, cache_janitor* janitor, Args&&... args)
      : parent_t(janitor, name) {
    make_shared_if_guaranteed<Guaranteed>(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Setters

  /// Assign a new shared pointer to the cache.
  my_t& operator=(const shared_ptr_t& datum) {
    datum_ = datum;
    return *this;
  }
  my_t& operator=(shared_ptr_t&& datum) {
    datum_ = std::move(datum);
    return *this;
  }

  /// Assign to cache, which is created if it doesn't exist.
  my_t& operator=(Type&& datum);
  my_t& operator=(const Type& datum);

  /// Create the provided default if the cache doesn't exist.
  /// @param[in] args the default cache value
  template <typename... Args>
  my_t& fallback(Args&&... args) {
    if (!is_valid())
      datum_ = std::make_shared<Type>(std::forward<Args>(args)...);
    return *this;
  }
  my_t& fallback(const Type& datum);
  my_t& fallback(Type&& datum);

  /// Release the pointer, emptying the cache (unless guaranteed).
  /// @note Didn't want to mess with allocators, and can't guarantee a default
  ///       constructor, so guaranteed data currently does not get reset.
  my_t& clear() {
    if (!Guaranteed) datum_.reset();
    return *this;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Getters

  /// Dereference container to underlying type.
  /// @throws runtime_error if the cache is not initialized
  const Type& operator*() const {
    if (is_valid())
      return *datum_;
    else {
      /// LOG(ERROR) << el::base::debug::StackTrace();
      throw std::runtime_error("cache datum '" + name_ +
                               "' was unset on reference request.");
    }
  }
  using parent_t::operator*;
  using parent_t::operator->;

  /// @returns whether the cache is valid.
  bool is_valid() const { return datum_ != nullptr; }

  /// Get a ref to the underlying shared pointer.
  const shared_ptr_t& ptr() const { return datum_; }
  shared_ptr_t& ptr() { return datum_; }

 protected:
  /// Assign to cache, which is created if it doesn't exist.
  template <typename T>
  my_t& assign(T&& datum) {
    if (is_valid())
      *datum_ = datum;
    else
      datum_ = std::make_shared<Type>(std::forward<T>(datum));
    return *this;
  }

  // Ensures the constructor only calls make_new if the data is guaranteed
  // Some non-guaranteed types might not have default constructors,
  // so SFINAE is required so those cases are not compiled.
  template <bool G2, typename... Args>
  typename std::enable_if<G2, void>::type make_shared_if_guaranteed(
      Args&&... args) {
    datum_ = std::make_shared<Type>(std::forward<Args>(args)...);
  }
  template <bool G2, typename... Args>
  typename std::enable_if<!G2, void>::type make_shared_if_guaranteed(
      Args&&...) {}

  /// @brief cache data storage
  shared_ptr_t datum_;
  using cache_base::name_;
};

}  // namespace common
}  // namespace vtr
