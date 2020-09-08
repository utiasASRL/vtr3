#pragma once

#include <array>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#define PTR_NAMED_TYPEDEFS(ClassName)                             \
  typedef std::shared_ptr<ClassName> ClassName##Ptr;              \
  typedef std::shared_ptr<const ClassName> ClassName##ConstPtr;   \
  typedef std::weak_ptr<ClassName> ClassName##WeakPtr;            \
  typedef std::weak_ptr<const ClassName> ClassName##WeakConstPtr; \
  typedef std::unique_ptr<ClassName> ClassName##UniquePtr;

#define PTR_TYPEDEFS(ClassName)                        \
  typedef std::shared_ptr<ClassName> Ptr;              \
  typedef std::shared_ptr<const ClassName> ConstPtr;   \
  typedef std::weak_ptr<ClassName> WeakPtr;            \
  typedef std::weak_ptr<const ClassName> WeakConstPtr; \
  typedef std::unique_ptr<ClassName> UniquePtr;

#define CONTAINER_NAMED_TYPEDEFS(TypeName)                     \
  typedef std::vector<TypeName> TypeName##Vector;              \
  typedef std::set<TypeName> TypeName##Set;                    \
  typedef std::list<TypeName> TypeName##List;                  \
  typedef std::unordered_set<TypeName> TypeName##UnorderedSet; \
  typedef std::pair<TypeName, TypeName> TypeName##Pair;

#define CONTAINER_TYPEDEFS(TypeName)                 \
  typedef std::vector<TypeName> Vector;              \
  typedef std::set<TypeName> Set;                    \
  typedef std::list<TypeName> List;                  \
  typedef std::unordered_set<TypeName> UnorderedSet; \
  typedef std::pair<TypeName, TypeName> Pair;

#define PTR_CONTAINER_TYPEDEFS(ClassName) \
  CONTAINER_NAMED_TYPEDEFS(Ptr)           \
  CONTAINER_NAMED_TYPEDEFS(ConstPtr)      \
  CONTAINER_NAMED_TYPEDEFS(WeakPtr)       \
  CONTAINER_NAMED_TYPEDEFS(WeakConstPtr)  \
  CONTAINER_NAMED_TYPEDEFS(UniquePtr)

#define PTR_DOWNCAST_OPS(Derived, Base)                                     \
  static inline std::shared_ptr<Derived> Cast(                              \
      const std::shared_ptr<Base>& other) {                                 \
    return std::static_pointer_cast<Derived>(other);                        \
  }                                                                         \
  static inline std::shared_ptr<const Derived> Cast(                        \
      const std::shared_ptr<const Base>& other) {                           \
    return std::static_pointer_cast<const Derived>(other);                  \
  }                                                                         \
  static inline std::weak_ptr<Derived> Cast(                                \
      const std::weak_ptr<Base>& other) {                                   \
    return std::static_pointer_cast<Derived>(std::shared_ptr<Base>(other)); \
  }                                                                         \
  static inline std::weak_ptr<const Derived> Cast(                          \
      const std::weak_ptr<const Base>& other) {                             \
    return std::static_pointer_cast<const Derived>(                         \
        std::shared_ptr<const Base>(other));                                \
  }

#define INHERITANCE_TESTS(Type, Base)              \
  static inline bool InChain(const Base* other) {  \
    return bool(dynamic_cast<const Type*>(other)); \
  }                                                \
  static inline bool InChain(Base* other) {        \
    return bool(dynamic_cast<Type*>(other));       \
  }                                                \
  static inline bool IsType(const Base* other) {   \
    return (typeid(*other) == typeid(Type));       \
  }                                                \
  static inline bool IsType(Base* other) {         \
    return (typeid(*other) == typeid(Type));       \
  }

#define EXTEND_HASH(Type)                                       \
  namespace std {                                               \
  template <>                                                   \
  struct hash<Type> {                                           \
    size_t operator()(const Type& T) const { return T.hash(); } \
  };                                                            \
  }

#define EXTEND_HASH_TEMPLATED(Type, A)                             \
  namespace std {                                                  \
  template <class A>                                               \
  struct hash<Type<A> > {                                          \
    size_t operator()(const Type<A>& T) const { return T.hash(); } \
  };                                                               \
  }