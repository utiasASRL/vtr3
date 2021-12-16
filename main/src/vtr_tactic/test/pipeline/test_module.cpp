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
 * \file test_query_buffer.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/modules/factory.hpp"
#include "vtr_tactic/pipelines/factory.hpp"

// include the modules
#include "vtr_tactic/modules/template_module.hpp"
#include "vtr_tactic/pipelines/template_pipeline.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

TEST(Module, module_factory_make) {
  auto module_factory = std::make_shared<ModuleFactoryV2>();

  QueryCache qdata;
  OutputCache output;

  // default construct
  auto module1 = module_factory->make("template");
  module1->run(qdata, output, nullptr, nullptr);

  // non-default construct
  auto config = std::make_shared<TemplateModule::Config>();
  config->parameter = "non-default value";
  auto module2 = module_factory->make("template", config);
  module2->run(qdata, output, nullptr, nullptr);
}

TEST(Module, module_factory_get) {
  auto module_factory = std::make_shared<ModuleFactoryV2>();

  QueryCache qdata;
  OutputCache output;

  // contruct on first get
  auto module1 = module_factory->get("template");
  module1->run(qdata, output, nullptr, nullptr);

  // subsequent get should return the same module (the default one)
  auto config = std::make_shared<TemplateModule::Config>();
  config->parameter = "non-default value";
  auto module2 = module_factory->get("template");
  module2->run(qdata, output, nullptr, nullptr);
}

TEST(Pipeline, pipeline_factory_basics) {
  auto pipeline_factory = std::make_shared<PipelineFactoryV2>();
  auto qdata = std::make_shared<QueryCache>();

  // default construct
  {
    auto template_pipeline = pipeline_factory->make("template");
    template_pipeline->preprocess(qdata, nullptr, nullptr, nullptr);
  }

  // non-default construct
  {
    auto config = std::make_shared<TemplatePipeline::Config>();
    config->parameter = "non-default value";
    auto template_pipeline = pipeline_factory->make("template", config);
    template_pipeline->preprocess(qdata, nullptr, nullptr, nullptr);
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}