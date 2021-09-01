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
 * \file example.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

/**
 * \brief VTR namespace.
 * \details VTR namespace detailed description.
 */
namespace vtr {
/**
 * \brief VTR documentation namespace.
 * \details This is the namespace containing the documentation.
 */
namespace documentation {
/**
 * \brief class brief description.
 * \details Detailed class description with a reference to \ref
 * example_tutorial.
 */
class ExampleClass {
 public:
  /**
   * \brief Default constructor.
   * \details The default constructor throws a table (╯°o°）╯︵ ┻━┻.
   */
  ExampleClass();

 protected:
  /**
   * \brief function brief description.
   * \details Detailed function description.
   * \param[in] var1 description of var1
   * \param[in] var2 description of var2
   * \param[out] var3 description of var3
   */
  int exampleFunction1(const int& var1, const int& var2, int& var3);

  /**
   * \brief function brief description.
   * \details Detailed function description.
   * \param[in] var1 description of var1
   * \param[in] var2 description of var2
   * \param[out] var3 description of var3
   */
  int exampleFunction2(const int& var1, const int& var2, int& var3);
};

}  // namespace documentation
}  // namespace vtr