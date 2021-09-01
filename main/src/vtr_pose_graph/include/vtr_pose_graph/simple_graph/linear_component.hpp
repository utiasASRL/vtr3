// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file linear_component.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

namespace vtr {
namespace pose_graph {
namespace simple {

template <class element_t>
class LinearComponent {
 public:
  using ComponentList = std::list<LinearComponent>;
  using ElementList = std::list<element_t>;

  template <class other_element_t>
  LinearComponent(const std::list<other_element_t> &elements);

  template <class other_element_t>
  LinearComponent(const LinearComponent<other_element_t> &other);

  LinearComponent(const LinearComponent &) = default;
  LinearComponent(LinearComponent &&) = default;

  LinearComponent &operator=(const LinearComponent &) = default;
  LinearComponent &operator=(LinearComponent &&) = default;

  size_t size() const {
    return isCyclic() ? elements_.size() - 1 : elements_.size();
  }

  element_t from() const { return elements_.front(); }
  element_t to() const { return elements_.back(); }
  const ElementList &elements() const { return elements_; }

  bool isCyclic() const { return elements_.front() == elements_.back(); }
  void reverse() { elements_.reverse(); }

  auto begin() const -> decltype(std::declval<ElementList const>().begin()) {
    return elements_.begin();
  }
  auto end() const -> decltype(std::declval<ElementList const>().end()) {
    return elements_.end();
  }
  auto rbegin() const -> decltype(std::declval<ElementList const>().rbegin()) {
    return elements_.rbegin();
  }
  auto rend() const -> decltype(std::declval<ElementList const>().rend()) {
    return elements_.rend();
  }

  ComponentList splitAtElement(const element_t &mid) const;
  ComponentList split(const size_t &n = 2) const;

 protected:
  template <class other_element_t>
  struct convert_t {
    other_element_t operator()(const other_element_t &val) {
      return element_t(val);
    }
  };

  ElementList elements_;
};

template <class element_t>
template <typename other_element_t>
LinearComponent<element_t>::LinearComponent(
    const std::list<other_element_t> &elements) {
  std::transform(std::begin(elements), std::end(elements),
                 std::back_inserter(elements_), convert_t<other_element_t>());
}

template <class element_t>
template <typename other_element_t>
LinearComponent<element_t>::LinearComponent(
    const LinearComponent<other_element_t> &other) {
  std::transform(std::begin(other.elements()), std::end(other.elements()),
                 std::back_inserter(elements_), convert_t<other_element_t>());
}

template <class element_t>
auto LinearComponent<element_t>::splitAtElement(const element_t &mid) const
    -> ComponentList {
  auto mid_iter = std::find(elements_.begin(), elements_.end(), mid);

  if (mid_iter == elements_.end()) {
    throw std::invalid_argument(
        "[LinearComponent::split] Vertex not in component");
  }

  return {Component(ElementList(elements_.begin(), mid_iter + 1)),
          Component(ElementList(mid_iter, elements_.end()))};
}

template <class element_t>
auto LinearComponent<element_t>::split(const size_t &n) const -> ComponentList {
  if (n > elements_.size()) {
    throw std::invalid_argument(
        "[LinearComponent::split] Number of splits exceeds vertex count");
  }

  double step = double(elements_.size() - 1) / n;
  auto elast = elements_.begin();
  ComponentList parts;

  for (unsigned int i = 0; i < n; ++i) {
    int diff = int((i + 1) * step) - int(i * step);
    parts.push_back(
        LinearComponent(ElementList(elast, std::next(elast, diff + 1))));
    elast = std::next(elast, diff);
  }

  return parts;
}

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
