#pragma once

#include <deque>
#include <functional>
#include <unordered_map>

#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>
#include <vtr_pose_graph/utils/hash.hpp>  // hash for std::pair<T1, T2> (for std::unordered_map)

namespace vtr {
namespace pose_graph {
namespace eval {

template <class RVAL>
class EvalBase {
 public:
  using VertexIdType = VertexId;
  using EdgeIdType = EdgeId;

  using SimpleVertex = uint64_t;
  using SimpleEdge = std::pair<uint64_t, uint64_t>;

  using VertexMap = std::unordered_map<SimpleVertex, RVAL>;
  using EdgeMap = std::unordered_map<SimpleEdge, RVAL>;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(EvalBase);

  EvalBase() = default;
  EvalBase(const EvalBase &) = default;
  EvalBase(EvalBase &&) = default;

  virtual ~EvalBase() = default;

  EvalBase &operator=(const EvalBase &) = default;
  EvalBase &operator=(EvalBase &&) = default;

  /// \todo (yuchen) Is there a better way to expose this at the top level
  /// without using a nasty void*?
  virtual void setGraph(void *) = 0;

  // Reference returning operator[]
  virtual RVAL operator[](const SimpleEdge &) = 0;
  virtual RVAL operator[](const SimpleVertex &) = 0;
  virtual RVAL operator[](const EdgeIdType &) = 0;
  virtual RVAL operator[](const VertexIdType &) = 0;

  // Copy returning at()
  virtual RVAL at(const SimpleEdge &) const = 0;
  virtual RVAL at(const SimpleVertex &) const = 0;
  virtual RVAL at(const EdgeIdType &) const = 0;
  virtual RVAL at(const VertexIdType &) const = 0;
};

template <class RVAL, class GRAPH>
class TypedBase : public EvalBase<RVAL> {
 public:
  using Base = EvalBase<RVAL>;

  using VertexType = typename GRAPH::VertexType;
  using EdgeType = typename GRAPH::EdgeType;

  using VertexPtr = typename VertexType::Ptr;
  using EdgePtr = typename EdgeType::Ptr;

  using SimpleVertex = typename Base::SimpleVertex;
  using SimpleEdge = typename Base::SimpleEdge;
  using VertexIdType = typename Base::VertexIdType;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexMap = typename Base::VertexMap;
  using EdgeMap = typename Base::EdgeMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(TypedBase);

  TypedBase() = default;
  TypedBase(GRAPH *graph) : graph_(graph){};
  TypedBase(const TypedBase &) = default;
  TypedBase(TypedBase &&) = default;

  virtual ~TypedBase() {
  }  // NOTE: we intentionally do not delete the graph
     // pointer as we do not own it

  TypedBase &operator=(const TypedBase &) = default;
  TypedBase &operator=(TypedBase &&) = default;

  virtual void setGraph(void *graph) {
    graph_ = reinterpret_cast<GRAPH *>(graph);
  }

  virtual RVAL operator[](const EdgePtr &e) = 0;
  virtual RVAL operator[](const VertexPtr &v) = 0;

  virtual RVAL at(const EdgePtr &e) const = 0;
  virtual RVAL at(const VertexPtr &v) const = 0;

 protected:
  GRAPH *graph_;  // NOTE: we intentionally do not delete the graph pointer as
                  // we do not own it
};

/** \brief Simple evaluator for a fixed constant weight */
template <class RVAL>
class ConstantBase : public EvalBase<RVAL> {
 public:
  using Base = EvalBase<RVAL>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(ConstantBase);

  static Ptr MakeShared(const RVAL &edgeValue = RVAL(),
                        const RVAL &vertexValue = RVAL()) {
    return Ptr(new ConstantBase(edgeValue, vertexValue));
  }

  static typename Base::Ptr MakeBase(const RVAL &edgeValue = RVAL(),
                                     const RVAL &vertexValue = RVAL()) {
    return typename Base::Ptr(new ConstantBase(edgeValue, vertexValue));
  }

  ConstantBase(const RVAL &edgeValue = RVAL(), const RVAL &vertexValue = RVAL())
      : edgeValue_(edgeValue), vertexValue_(vertexValue){};
  ConstantBase(const ConstantBase &) = default;
  ConstantBase(ConstantBase &&) = default;

  virtual ~ConstantBase(){};

  ConstantBase &operator=(const ConstantBase &) = default;
  ConstantBase &operator=(ConstantBase &&) = default;

  virtual void setGraph(void *){};

  virtual RVAL operator[](const SimpleVertex &) {
    return this->vertexValue_;
  }
  virtual RVAL operator[](const SimpleEdge &) {
    return this->edgeValue_;
  }
  virtual RVAL operator[](const VertexIdType &) {
    return this->vertexValue_;
  }
  virtual RVAL operator[](const EdgeIdType &) {
    return this->edgeValue_;
  }

  virtual RVAL at(const SimpleVertex &) const {
    return this->vertexValue_;
  }
  virtual RVAL at(const SimpleEdge &) const {
    return this->edgeValue_;
  }
  virtual RVAL at(const VertexIdType &) const {
    return this->vertexValue_;
  }
  virtual RVAL at(const EdgeIdType &) const {
    return this->edgeValue_;
  }

 protected:
  const RVAL edgeValue_;
  const RVAL vertexValue_;
};

/** \brief Base evaluator for a fixed map */
template <class RVAL>
class MapBase : public EvalBase<RVAL> {
 public:
  using Base = EvalBase<RVAL>;

  using SimpleVertex = typename Base::SimpleVertex;
  using SimpleEdge = typename Base::SimpleEdge;
  using VertexIdType = typename Base::VertexIdType;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexMap = typename Base::VertexMap;
  using EdgeMap = typename Base::EdgeMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(MapBase);

  static Ptr MakeShared(
      const EdgeMapPtr &edgeWeights = EdgeMapPtr(new EdgeMap()),
      const VertexMapPtr &vertexWeights = VertexMapPtr(new VertexMap())) {
    return Ptr(new MapBase(edgeWeights, vertexWeights));
  }

  MapBase(const EdgeMapPtr &edgeWeights = EdgeMapPtr(new EdgeMap()),
          const VertexMapPtr &vertexWeights = VertexMapPtr(new VertexMap()))
      : edgeMap_(edgeWeights), vertexMap_(vertexWeights) {
  }
  MapBase(const MapBase &) = default;
  MapBase(MapBase &&) = default;

  virtual ~MapBase() {
  }

  MapBase &operator=(const MapBase &) = default;
  MapBase &operator=(MapBase &&) = default;

  virtual void setGraph(void *) {
  }

  virtual RVAL operator[](const SimpleVertex &v) {
    return this->vertexMap_->operator[](v);
  }
  virtual RVAL operator[](const SimpleEdge &e) {
    return this->edgeMap_->operator[](e);
  }
  virtual RVAL operator[](const VertexIdType &v) {
    return this->vertexMap_->operator[](SimpleVertex(v));
  }
  virtual RVAL operator[](const EdgeIdType &e) {
    return this->edgeMap_->operator[](SimpleEdge(e));
  }

  virtual RVAL at(const SimpleVertex &v) const {
    return this->vertexMap_->at(v);
  }
  virtual RVAL at(const SimpleEdge &e) const {
    return this->edgeMap_->at(e);
  }
  virtual RVAL at(const VertexIdType &v) const {
    return this->vertexMap_->at(SimpleVertex(v));
  }
  virtual RVAL at(const EdgeIdType &e) const {
    return this->edgeMap_->at(SimpleEdge(e));
  }

  virtual RVAL &ref(const SimpleVertex &v) {
    return this->vertexMap_->operator[](v);
  }
  virtual RVAL &ref(const SimpleEdge &e) {
    return this->edgeMap_->operator[](e);
  }
  virtual RVAL &ref(const VertexIdType &v) {
    return this->vertexMap_->operator[](SimpleVertex(v));
  }
  virtual RVAL &ref(const EdgeIdType &e) {
    return this->edgeMap_->operator[](SimpleEdge(e));
  }

 protected:
  EdgeMapPtr edgeMap_;
  VertexMapPtr vertexMap_;
};

/** \brief Evaluator for a function on edge/vertex data */
template <class RVAL, class GRAPH>
class DirectBase : public TypedBase<RVAL, GRAPH> {
 public:
  using Base = TypedBase<RVAL, GRAPH>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(DirectBase);

  DirectBase() = default;
  DirectBase(GRAPH *graph) : Base(graph) {
  }
  DirectBase(const DirectBase &) = default;
  DirectBase(DirectBase &&) = default;

  DirectBase &operator=(const DirectBase &) = default;
  DirectBase &operator=(DirectBase &&) = default;

  virtual ~DirectBase() {
  }

  virtual RVAL operator[](const SimpleVertex &v) {
    return this->operator[](this->graph_->at(v));
  }
  virtual RVAL operator[](const SimpleEdge &e) {
    return this->operator[](this->graph_->at(e));
  }
  virtual RVAL operator[](const VertexIdType &v) {
    return this->operator[](this->graph_->at(v));
  }
  virtual RVAL operator[](const EdgeIdType &e) {
    return this->operator[](this->graph_->at(e));
  }
  virtual RVAL operator[](const VertexPtr &v) {
    return this->computeVertex(v);
  }
  virtual RVAL operator[](const EdgePtr &e) {
    return this->computeEdge(e);
  }

  virtual RVAL at(const SimpleVertex &v) const {
    return this->at(this->graph_->at(v));
  }
  virtual RVAL at(const SimpleEdge &e) const {
    return this->at(this->graph_->at(e));
  }
  virtual RVAL at(const VertexIdType &v) const {
    return this->at(this->graph_->at(v));
  }
  virtual RVAL at(const EdgeIdType &e) const {
    return this->at(this->graph_->at(e));
  }
  virtual RVAL at(const VertexPtr &v) const {
    return this->computeVertex(v);
  }
  virtual RVAL at(const EdgePtr &e) const {
    return this->computeEdge(e);
  }

 protected:
  virtual RVAL computeVertex(const VertexPtr &v) const = 0;
  virtual RVAL computeVertex(const VertexPtr &v) {
    return const_cast<const DirectBase *>(this)->computeVertex(v);
  }
  virtual RVAL computeEdge(const EdgePtr &e) const = 0;
  virtual RVAL computeEdge(const EdgePtr &e) {
    return const_cast<const DirectBase *>(this)->computeEdge(e);
  }
};

/** \brief Evaluator for a runtime-provided function on edge/vertex data */
template <class RVAL, class GRAPH>
class LambdaBase : public DirectBase<RVAL, GRAPH> {
 public:
  using Base = DirectBase<RVAL, GRAPH>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  using EdgeFunction = std::function<RVAL(const EdgePtr &)>;
  using VertexFunction = std::function<RVAL(const VertexPtr &)>;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(LambdaBase);

  static inline RVAL edgeTrue(const EdgePtr &v) {
    return true;
  }
  static inline RVAL vertexTrue(const VertexPtr &v) {
    return true;
  }

  LambdaBase() = delete;
  LambdaBase(const EdgeFunction &edge_function,
             const VertexFunction &vertex_function, GRAPH *graph = nullptr)
      : edge_function_(edge_function),
        vertex_function_(vertex_function),
        Base(graph) {
  }
  LambdaBase(const EdgeFunction &edge_function, GRAPH *graph = nullptr)
      : edge_function_(edge_function),
        vertex_function_(vertexTrue),
        Base(graph) {
  }
  LambdaBase(const VertexFunction &vertex_function, GRAPH *graph = nullptr)
      : edge_function_(edgeTrue),
        vertex_function_(vertex_function),
        Base(graph) {
  }
  LambdaBase(const LambdaBase &) = default;
  LambdaBase(LambdaBase &&) = default;

  LambdaBase &operator=(const LambdaBase &) = default;
  LambdaBase &operator=(LambdaBase &&) = default;

  virtual ~LambdaBase() {
  }

 protected:
  inline RVAL computeEdge(const EdgePtr &e) const override {
    return edge_function_(e);
  }
  inline RVAL computeVertex(const VertexPtr &v) const override {
    return vertex_function_(v);
  }

  EdgeFunction edge_function_;
  VertexFunction vertex_function_;
  using Base::Base::graph_;
};

/** \brief Evaluator for a function on edge/vertex data, with caching */
template <class RVAL, class GRAPH>
class CachingBase : public virtual DirectBase<RVAL, GRAPH> {
 public:
  using Base = DirectBase<RVAL, GRAPH>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(CachingBase);

  CachingBase()
      : DirectBase<RVAL, GRAPH>(),
        edgeMap_(new EdgeMap()),
        vertexMap_(new VertexMap) {
  }
  CachingBase(const CachingBase &) = default;
  // NOTE: this does not move-construct any base classes
  CachingBase(CachingBase &&other)
      : edgeMap_(std::move(other.edgeMap_)),
        vertexMap_(std::move(other.vertexMap_)) {
  }

  virtual ~CachingBase() {
  }

  CachingBase &operator=(const CachingBase &) = default;
  // NOTE: this does not move-assign any base classes
  CachingBase &operator=(CachingBase &&other) {
    this->edgeMap_ = std::move(other.edgeMap_);
    this->vertexMap_ = std::move(other.vertexMap_);
    return *this;
  }

  virtual RVAL operator[](const SimpleVertex &v);
  virtual RVAL operator[](const SimpleEdge &e);
  virtual RVAL operator[](const VertexIdType &v) {
    return this->operator[](SimpleVertex(v));
  }

  virtual RVAL operator[](const EdgeIdType &e) {
    return this->operator[](SimpleEdge(e));
  }
  virtual RVAL operator[](const VertexPtr &v);
  virtual RVAL operator[](const EdgePtr &e);

  virtual RVAL at(const SimpleVertex &v) const {
    return this->vertexMap_->at(v);
  }
  virtual RVAL at(const SimpleEdge &e) const {
    return this->edgeMap_->at(e);
  }
  virtual RVAL at(const VertexIdType &v) const {
    return this->vertexMap_->at(SimpleVertex(v));
  }
  virtual RVAL at(const EdgeIdType &e) const {
    return this->edgeMap_->at(SimpleEdge(e));
  }
  virtual RVAL at(const VertexPtr &v) const {
    return this->vertexMap_->at(v->simpleId());
  }
  virtual RVAL at(const EdgePtr &e) const {
    return this->edgeMap_->at(e->simpleId());
  }

 protected:
  EdgeMapPtr edgeMap_;
  VertexMapPtr vertexMap_;
};

/**
 * \brief Evaluator for a function on edge/vertex data, with a fixed size cache
 */
template <class RVAL, class GRAPH>
class WindowedBase : public virtual CachingBase<RVAL, GRAPH> {
 public:
  using Base = CachingBase<RVAL, GRAPH>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  PTR_NAMED_TYPEDEFS(VertexMap);
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_TYPEDEFS(WindowedBase);

  WindowedBase(const size_t &cacheSize = 500)
      : edgeQueue_(cacheSize, SimpleEdge(-1, -1)),
        vertexQueue_(cacheSize, SimpleVertex(-1)) {
  }
  WindowedBase(const WindowedBase &) = default;
  // NOTE: this does not move-construct any base classes
  WindowedBase(WindowedBase &&other)
      : edgeQueue_(std::move(other.edgeQueue_)),
        vertexQueue_(std::move(other.vertexQueue_)) {
  }

  virtual ~WindowedBase(){};

  WindowedBase &operator=(const WindowedBase &) = default;
  // NOTE: this does not move-assign any base classes
  WindowedBase &operator=(WindowedBase &&other) {
    this->edgeQueue_ = std::move(other.edgeQueue_);
    this->vertexQueue_ = std::move(other.vertexQueue_);
    return *this;
  }

  virtual RVAL operator[](const SimpleVertex &v);
  virtual RVAL operator[](const SimpleEdge &e);
  virtual RVAL operator[](const VertexIdType &v) {
    return this->operator[](SimpleVertex(v));
  }
  virtual RVAL operator[](const EdgeIdType &e) {
    return this->operator[](SimpleEdge(e));
  }
  virtual RVAL operator[](const VertexPtr &v);
  virtual RVAL operator[](const EdgePtr &e);

 protected:
  std::deque<SimpleEdge> edgeQueue_;
  std::deque<SimpleVertex> vertexQueue_;
};

/** \brief Convenience typedef of all evaluator base */
template <class RVAL>
struct Base {
  //  using Base =  EvalBase<RVAL>;
  using Const = ConstantBase<RVAL>;
  using Map = MapBase<RVAL>;

  template <class GRAPH>
  struct Typed {
    using Base = TypedBase<RVAL, GRAPH>;
    using Direct = DirectBase<RVAL, GRAPH>;
    using Lambda = LambdaBase<RVAL, GRAPH>;
    using Caching = CachingBase<RVAL, GRAPH>;
    using Windowed = WindowedBase<RVAL, GRAPH>;
  };
};

/** \brief Macro to create a new evaluator base type */
#define NEW_EVALUATOR_TYPE(Name, ScalarType)                \
  using Name##Eval = EvalBase<ScalarType>;                  \
  namespace Name {                                          \
  using ReturnType = ScalarType;                            \
  using Base = eval::EvalBase<ScalarType>;                  \
  using Const = eval::ConstantBase<ScalarType>;             \
  using Map = eval::MapBase<ScalarType>;                    \
  PTR_TYPEDEFS(Base);                                       \
                                                            \
  template <class GRAPH>                                    \
  struct Typed {                                            \
    using Base = eval::TypedBase<ScalarType, GRAPH>;        \
    using Direct = eval::DirectBase<ScalarType, GRAPH>;     \
    using Lambda = eval::LambdaBase<ScalarType, GRAPH>;     \
    using Caching = eval::CachingBase<ScalarType, GRAPH>;   \
    using Windowed = eval::WindowedBase<ScalarType, GRAPH>; \
  };                                                        \
  }

/**
 * \brief Macros to create explicit template instantiations (prevents
 * recompilation)
 */
#define EVAL_BASE_DECLARE_EXTERN(ScalarType)            \
  extern template class eval::EvalBase<ScalarType>;     \
  extern template class eval::ConstantBase<ScalarType>; \
  extern template class eval::MapBase<ScalarType>;

#define EVAL_BASE_EXPLICIT_INSTANTIATE(ScalarType) \
  template class eval::EvalBase<ScalarType>;       \
  template class eval::ConstantBase<ScalarType>;   \
  template class eval::MapBase<ScalarType>;

#define EVAL_TYPED_DECLARE_EXTERN(ScalarType, GRAPH)          \
  extern template class eval::TypedBase<ScalarType, GRAPH>;   \
  extern template class eval::DirectBase<ScalarType, GRAPH>;  \
  extern template class eval::LambdaBase<ScalarType, GRAPH>;  \
  extern template class eval::CachingBase<ScalarType, GRAPH>; \
  extern template class eval::WindowedBase<ScalarType, GRAPH>;

#define EVAL_TYPED_EXPLICIT_INSTANTIATE(ScalarType, GRAPH) \
  template class eval::TypedBase<ScalarType, GRAPH>;       \
  template class eval::DirectBase<ScalarType, GRAPH>;      \
  template class eval::LambdaBase<ScalarType, GRAPH>;      \
  template class eval::CachingBase<ScalarType, GRAPH>;     \
  template class eval::WindowedBase<ScalarType, GRAPH>;

////////////////////////////////////////////////////////////////////////////////
/// Macros to extend an existing evaluator base type
////////////////////////////////////////////////////////////////////////////////

#define DEFAULT_COPY(Name)      \
  Name(const Name &) = default; \
  Name &operator=(const Name &) = default;

#define DEFAULT_MOVE(Name) \
  Name(Name &&) = default; \
  Name &operator=(Name &&) = default;

/**
 * \brief Default move operations for virtual inheritance can cause double-move
 * of data
 */
#define EXPLICIT_VIRTUAL_MOVE(Name, Left, Right)                         \
  Name(Name &&other) : Left(std::move(other)), Right(std::move(other)) { \
  }                                                                      \
  Name &operator=(Name &&other) {                                        \
    Left::operator=(std::move(other));                                   \
    Right::operator=(std::move(other));                                  \
    return *this;                                                        \
  }

/**
 * \brief Define a direct evaluator
 * \details Used as: DIRECT_EVAL(<Name>) { <Implementation> }
 */
#define DIRECT_EVAL(Name) \
  template <class GRAPH>  \
  class Name##Direct : public virtual Typed<GRAPH>::Direct

/**
 * \brief Define a caching evaluator
 * \details Used as: CACHING_EVAL(<Name>) { <Implementation> }
 */
#define CACHING_EVAL(Name)                                    \
  template <class GRAPH>                                      \
  class Name##Caching : public virtual Typed<GRAPH>::Caching, \
                        public virtual Name##Direct<GRAPH>

/**
 * \brief Define a windowed evaluator
 * \details Used as: WINDOWED_EVAL(<Name>) { <Implementation> }
 */
#define WINDOWED_EVAL(Name)                                     \
  template <class GRAPH>                                        \
  class Name##Windowed : public virtual Typed<GRAPH>::Windowed, \
                         public virtual Name##Caching<GRAPH>

/** \brief Internal typedefs for a direct evaluator */
#define DIRECT_PREAMBLE(Name)                         \
  using AbstractBase = typename Typed<GRAPH>::Direct; \
  PTR_TYPEDEFS(Name##Direct);                         \
  DEFAULT_COPY(Name##Direct);                         \
  DEFAULT_MOVE(Name##Direct);

/** \brief Internal typedefs for a caching evaluator */
#define CACHING_PREAMBLE(Name)                         \
  using AbstractBase = typename Typed<GRAPH>::Caching; \
  using DirectBase = Name##Direct<GRAPH>;              \
  PTR_TYPEDEFS(Name##Caching);                         \
  DEFAULT_COPY(Name##Caching);                         \
  EXPLICIT_VIRTUAL_MOVE(Name##Caching, AbstractBase, DirectBase);

/** \brief Internal typedefs for a windowed evaluator */
#define WINDOWED_PREAMBLE(Name)                         \
  using AbstractBase = typename Typed<GRAPH>::Windowed; \
  using DirectBase = Name##Direct<GRAPH>;               \
  using CachingBase = Name##Caching<GRAPH>;             \
  PTR_TYPEDEFS(Name##Windowed);                         \
  DEFAULT_COPY(Name##Windowed);                         \
  EXPLICIT_VIRTUAL_MOVE(Name##Windowed, AbstractBase, DirectBase);

/**
 * \brief Defines the prototype for a constructor and the corresponding
 * MakeShared function
 * \details Used as:
 * EVAL_CONSTRUCTOR(<Name>,<Type>,<ProtoArgs>,<BasicArgs>) { <Implementation> }
 */
#define EVAL_CONSTRUCTOR(Name, Type, PrototypeArgs, BasicArgs) \
  static Ptr MakeShared PrototypeArgs {                        \
    return Ptr(new Name##Type BasicArgs);                      \
  }                                                            \
  Name##Type PrototypeArgs

/**
 * \brief Prototype for a generic evaluator destructor
 * \details Used as: EVAL_DESTRUCTOR(<Name>,<Type>) { <Implementation> }
 */
#define EVAL_DESTRUCTOR(Name, Type) virtual ~Name##Type()

/**
 * \brief Prototype for the overridden edge compute function
 * \details Used as: EVAL_COMPUTE_EDGE { <Implementation> }
 */
#define EVAL_COMPUTE_EDGE         \
  virtual ReturnType computeEdge( \
      const typename Typed<GRAPH>::Direct::EdgePtr &e)

/**
 * \brief Prototype for the overridden vertex compute function
 * \details Used as: EVAL_COMPUTE_VERTEX { <Implementation> }
 */
#define EVAL_COMPUTE_VERTEX         \
  virtual ReturnType computeVertex( \
      const typename Typed<GRAPH>::Direct::VertexPtr &v)

/*
 * \brief Convenience typedefs
 */
#define EVAL_TYPEDEFS(Name)                 \
  template <class GRAPH>                    \
  struct Name {                             \
    using Direct = Name##Direct<GRAPH>;     \
    using Caching = Name##Caching<GRAPH>;   \
    using Windowed = Name##Windowed<GRAPH>; \
  };

/**
 * \brief Macros to extend an existing evaluator base type when things are
 * simple.
 */
#define EVAL_SIMPLE_DEFINE(Name)                                               \
  template <class GRAPH>                                                       \
  class Name##Direct : public virtual Typed<GRAPH>::Direct {                   \
   public:                                                                     \
    using Base = typename Typed<GRAPH>::Direct;                                \
    using Base::graph_;                                                        \
                                                                               \
    PTR_TYPEDEFS(Name##Direct);                                                \
    DEFAULT_COPY(Name##Direct);                                                \
    DEFAULT_MOVE(Name##Direct);                                                \
                                                                               \
    static Ptr MakeShared() {                                                  \
      return Ptr(new Name##Direct());                                          \
    }                                                                          \
                                                                               \
    Name##Direct() {                                                           \
    }                                                                          \
    virtual ~Name##Direct() {                                                  \
    }                                                                          \
                                                                               \
   protected:                                                                  \
    virtual ReturnType computeEdge(const typename Base::EdgePtr &e) const;     \
    virtual ReturnType computeVertex(const typename Base::VertexPtr &v) const; \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  class Name##Caching : public virtual Typed<GRAPH>::Caching,                  \
                        public virtual Name##Direct<GRAPH> {                   \
   public:                                                                     \
    PTR_TYPEDEFS(Name##Caching);                                               \
    DEFAULT_COPY(Name##Caching);                                               \
                                                                               \
    static Ptr MakeShared() {                                                  \
      return Ptr(new Name##Caching());                                         \
    }                                                                          \
                                                                               \
    Name##Caching() {                                                          \
    }                                                                          \
    virtual ~Name##Caching() {                                                 \
    }                                                                          \
    EXPLICIT_VIRTUAL_MOVE(Name##Caching, Typed<GRAPH>::Caching,                \
                          Name##Direct<GRAPH>);                                \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  class Name##Windowed : public virtual Typed<GRAPH>::Windowed,                \
                         public virtual Name##Caching<GRAPH> {                 \
   public:                                                                     \
    PTR_TYPEDEFS(Name##Windowed);                                              \
    DEFAULT_COPY(Name##Windowed);                                              \
                                                                               \
    static Ptr MakeShared(const size_t &cacheSize) {                           \
      return Ptr(new Name##Windowed(cacheSize));                               \
    }                                                                          \
                                                                               \
    Name##Windowed(const size_t &cacheSize)                                    \
        : Typed<GRAPH>::Windowed(cacheSize) {                                  \
    }                                                                          \
    virtual ~Name##Windowed() {                                                \
    }                                                                          \
    EXPLICIT_VIRTUAL_MOVE(Name##Windowed, Typed<GRAPH>::Windowed,              \
                          Name##Caching<GRAPH>);                               \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  struct Name {                                                                \
    using Direct = Name##Direct<GRAPH>;                                        \
    using Caching = Name##Caching<GRAPH>;                                      \
    using Windowed = Name##Windowed<GRAPH>;                                    \
  };

#define EVAL_SIMPLE_RECURSIVE_DEFINE(Name)                                     \
  template <class GRAPH>                                                       \
  class Name##Direct : public virtual Typed<GRAPH>::Direct {                   \
   public:                                                                     \
    using Base = typename Typed<GRAPH>::Direct;                                \
    using Base::graph_;                                                        \
                                                                               \
    PTR_TYPEDEFS(Name##Direct);                                                \
    DEFAULT_COPY(Name##Direct);                                                \
    DEFAULT_MOVE(Name##Direct);                                                \
                                                                               \
    static Ptr MakeShared() {                                                  \
      return Ptr(new Name##Direct());                                          \
    }                                                                          \
                                                                               \
    Name##Direct() {                                                           \
    }                                                                          \
    virtual ~Name##Direct() {                                                  \
    }                                                                          \
                                                                               \
   protected:                                                                  \
    virtual ReturnType computeEdge(const typename Base::EdgePtr &e) const;     \
    virtual ReturnType computeVertex(const typename Base::VertexPtr &v) const; \
                                                                               \
    virtual ReturnType computeEdge(const typename Base::EdgePtr &e);           \
    virtual ReturnType computeVertex(const typename Base::VertexPtr &v);       \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  class Name##Caching : public virtual Typed<GRAPH>::Caching,                  \
                        public virtual Name##Direct<GRAPH> {                   \
   public:                                                                     \
    PTR_TYPEDEFS(Name##Caching);                                               \
    DEFAULT_COPY(Name##Caching);                                               \
                                                                               \
    static Ptr MakeShared() {                                                  \
      return Ptr(new Name##Caching());                                         \
    }                                                                          \
                                                                               \
    Name##Caching() {                                                          \
    }                                                                          \
    virtual ~Name##Caching() {                                                 \
    }                                                                          \
    EXPLICIT_VIRTUAL_MOVE(Name##Caching, Typed<GRAPH>::Caching,                \
                          Name##Direct<GRAPH>);                                \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  class Name##Windowed : public virtual Typed<GRAPH>::Windowed,                \
                         public virtual Name##Caching<GRAPH> {                 \
   public:                                                                     \
    PTR_TYPEDEFS(Name##Windowed);                                              \
    DEFAULT_COPY(Name##Windowed);                                              \
                                                                               \
    static Ptr MakeShared(const size_t &cacheSize) {                           \
      return Ptr(new Name##Windowed(cacheSize));                               \
    }                                                                          \
                                                                               \
    Name##Windowed(const size_t &cacheSize)                                    \
        : Typed<GRAPH>::Windowed(cacheSize) {                                  \
    }                                                                          \
    virtual ~Name##Windowed() {                                                \
    }                                                                          \
    EXPLICIT_VIRTUAL_MOVE(Name##Windowed, Typed<GRAPH>::Windowed,              \
                          Name##Caching<GRAPH>);                               \
  };                                                                           \
                                                                               \
  template <class GRAPH>                                                       \
  struct Name {                                                                \
    using Direct = Name##Direct<GRAPH>;                                        \
    using Caching = Name##Caching<GRAPH>;                                      \
    using Windowed = Name##Windowed<GRAPH>;                                    \
  };

#define EVAL_SIMPLE_COMPUTE_EDGE(Name) \
  template <class GRAPH>               \
  ReturnType Name##Direct<GRAPH>::computeEdge(const typename Base::EdgePtr &e)

#define EVAL_SIMPLE_COMPUTE_VERTEX(Name)         \
  template <class GRAPH>                         \
  ReturnType Name##Direct<GRAPH>::computeVertex( \
      const typename Base::VertexPtr &v)

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr

#define EVAL_EXPLICIT_DECLARE(EvalType, GraphType)    \
  extern template class EvalType##Direct<GraphType>;  \
  extern template class EvalType##Caching<GraphType>; \
  extern template class EvalType##Windowed<GraphType>;

#define EVAL_EXPLICIT_INSTANTIATE(EvalType, GraphType) \
  template class EvalType##Direct<GraphType>;          \
  template class EvalType##Caching<GraphType>;         \
  template class EvalType##Windowed<GraphType>;

#include <vtr_pose_graph/evaluator/evaluator_base.inl>
