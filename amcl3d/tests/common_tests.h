/*!
 * @file common_tests.h
 * @copyright Copyright (c) 2019, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <gmock/gmock.h>

#define ASSERT_THROW_WHAT(statement, expected_exception, expected_what) \
  ASSERT_THROW(                                                         \
    try {                                                               \
      statement;                                                        \
    } catch (const expected_exception& e) {                             \
      ASSERT_EQ(std::string(expected_what), std::string(e.what()));     \
      throw;                                                            \
    }, expected_exception)

#define ASSERT_THROW_NO_WHAT(statement, expected_exception, expected_what)  \
  ASSERT_THROW(                                                             \
    try {                                                                   \
      statement;                                                            \
    } catch (const expected_exception& e) {                                 \
      ASSERT_NE(std::string(expected_what), std::string(e.what()));         \
      throw;                                                                \
    }, expected_exception)

static const std::string DATA_DIR = std::string(PROJECT_SOURCE_DIR) + std::string("/data");
static const std::string TESTSDATA_DIR = std::string(PROJECT_SOURCE_DIR) + std::string("/tests/data");

static const double DEFAULT_SENSOR_DEV = 0.05;
