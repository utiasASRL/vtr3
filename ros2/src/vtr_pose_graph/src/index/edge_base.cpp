#include <vtr_pose_graph/index/edge_base.hpp>

namespace vtr {
namespace pose_graph {

const int EdgeBase::transform_rows;
const int EdgeBase::transform_cols;
const int EdgeBase::transform_vdim;

EdgeBase::Ptr EdgeBase::MakeShared() {
  return Ptr(new EdgeBase());
}

EdgeBase::Ptr EdgeBase::MakeShared(const IdType& id) {
  return Ptr(new EdgeBase(id));
}

EdgeBase::Ptr EdgeBase::MakeShared(const IdType& id, const VertexIdType& fromId,
                                   const VertexIdType& toId, bool manual) {
  return Ptr(new EdgeBase(id, fromId, toId, manual));
}

EdgeBase::Ptr EdgeBase::MakeShared(const IdType& id, const VertexIdType& fromId,
                                   const VertexIdType& toId,
                                   const TransformType& T_to_from,
                                   bool manual) {
  return Ptr(new EdgeBase(id, fromId, toId, T_to_from, manual));
}

EdgeBase::EdgeBase()
    : id_(IdType()),
      from_(VertexIdType()),
      to_(VertexIdType()),
      T_to_from_(TransformType()),
      manual_(false),
      modified_(false) {
}

EdgeBase::EdgeBase(const IdType& id)
    : id_(id),
      from_(VertexIdType()),
      to_(VertexIdType()),
      T_to_from_(TransformType()),
      manual_(false),
      modified_(false) {
}

EdgeBase::EdgeBase(const IdType id, const VertexIdType& fromId,
                   const VertexIdType& toId, bool manual)
    : id_(id),
      from_(fromId),
      to_(toId),
      T_to_from_(TransformType()),
      manual_(manual),
      modified_(false) {
}

EdgeBase::EdgeBase(const IdType id, const VertexIdType& fromId,
                   const VertexIdType& toId, const TransformType& T_to_from,
                   bool manual)
    : id_(id),
      from_(fromId),
      to_(toId),
      T_to_from_(T_to_from),
      manual_(manual),
      modified_(false) {
}

EdgeBase::IdType EdgeBase::id() const {
  return id_;
}
EdgeBase::SimpleIdType EdgeBase::simpleId() const {
  return id_;
}
EdgeBase::IdType::Type EdgeBase::type() const {
  return id_.type();
}
size_t EdgeBase::idx() const {
  return id_.idx();
}

EdgeBase::VertexIdType::Pair EdgeBase::incident() const {
  return VertexIdType::Pair(from_, to_);
}

EdgeBase::VertexIdType EdgeBase::from() const {
  return from_;
}

void EdgeBase::setFrom(const VertexIdType& fromId) {
  from_ = fromId;
  id_ = IdType(from_, to_, id_.type());
  modified_ = true;
}

EdgeBase::VertexIdType EdgeBase::to() const {
  return to_;
}

void EdgeBase::setTo(const VertexIdType& toId) {
  to_ = toId;
  id_ = IdType(from_, to_, id_.type());
  modified_ = true;
}

bool EdgeBase::isManual() const {
  return manual_;
}

void EdgeBase::setManual(bool manual) {
  manual_ = manual;
  modified_ = true;
}

bool EdgeBase::isAutonomous() const {
  return !manual_;
}

void EdgeBase::setAutonomous(bool autonomous) {
  manual_ = !autonomous;
  modified_ = true;
}

bool EdgeBase::isTemporal() const {
  return id_.type() == IdType::Type::Temporal;
}

bool EdgeBase::isSpatial() const {
  return id_.type() == IdType::Type::Spatial;
}

bool EdgeBase::isIncident(const VertexIdType& v) const {
  return (from_ == v) || (to_ == v);
}

bool EdgeBase::isModified() const {
  return modified_;
}

void EdgeBase::setModified(bool modified) {
  modified_ = modified;
}

EdgeBase::TransformType EdgeBase::T() const {
  return T_to_from_;
}

void EdgeBase::setTransform(const TransformType& transform) {
  T_to_from_ = transform;
  modified_ = true;
}

std::ostream& operator<<(std::ostream& out, const EdgeBase& e) {
  if (e.type() == EdgeBase::IdType::Type::Spatial)
    return out << "{" << e.from() << "--" << e.to() << "}";
  else if (e.type() == EdgeBase::IdType::Type::Temporal)
    return out << "{" << e.from() << "==" << e.to() << "}";
  else
    return out << "{" << e.from() << "??" << e.to() << "}";
}

}  // namespace pose_graph
}  // namespace vtr
