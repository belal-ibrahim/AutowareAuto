// Copyright 2017-2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
// All rights reserved.
#ifndef AUTOWARE_TEST_TOOLS__AUTOWARE_TEST_TOOLS_HPP_
#define AUTOWARE_TEST_TOOLS__AUTOWARE_TEST_TOOLS_HPP_

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <osrf_testing_tools_cpp/memory_tools/testing_helpers.hpp>
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <osrf_testing_tools_cpp/scope_exit.hpp>

namespace autoware_test_tools
{
namespace memory_test
{

inline void resume(void)
{
  osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
  osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();
  osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
  osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
}

inline void pause(void)
{
  osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();
  osrf_testing_tools_cpp::memory_tools::expect_no_realloc_end();
  osrf_testing_tools_cpp::memory_tools::expect_no_calloc_end();
  osrf_testing_tools_cpp::memory_tools::expect_no_free_end();
}

inline void start(void)
{
  osrf_testing_tools_cpp::memory_tools::initialize();
  // create a callback for "unexpected" mallocs that does a non-fatal gtest
  // failure and then register it with memory tools
  auto on_unexpected_malloc =
    [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
      ADD_FAILURE() << "autoware_test_tools::memory_test: Unexpected malloc";
      // this will cause a bracktrace to be printed for each unexpected malloc
      service.print_backtrace();
    };
  osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_malloc);

  auto on_unexpected_realloc =
    [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
      ADD_FAILURE() << "autoware_test_tools::memory_test: Unexpected realloc";
      // this will cause a bracktrace to be printed for each unexpected realloc
      service.print_backtrace();
    };
  osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_realloc);

  auto on_unexpected_calloc =
    [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
      ADD_FAILURE() << "autoware_test_tools::memory_test: Unexpected calloc";
      // this will cause a bracktrace to be printed for each unexpected calloc
      service.print_backtrace();
    };
  osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(on_unexpected_calloc);

  auto on_unexpected_free =
    [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
      ADD_FAILURE() << "autoware_test_tools::memory_test: Unexpected free";
      // this will cause a bracktrace to be printed for each unexpected free
      service.print_backtrace();
    };
  osrf_testing_tools_cpp::memory_tools::on_unexpected_free(on_unexpected_free);

  osrf_testing_tools_cpp::memory_tools::enable_monitoring();
  autoware_test_tools::memory_test::resume();
}

inline void start_paused(void)
{
  autoware_test_tools::memory_test::start();
  autoware_test_tools::memory_test::pause();
}

inline void stop(void)
{
  autoware_test_tools::memory_test::pause();
  (void)osrf_testing_tools_cpp::memory_tools::disable_monitoring_in_all_threads();
  (void)osrf_testing_tools_cpp::memory_tools::uninitialize();
}

}  // namespace memory_test
}  // namespace autoware_test_tools

#endif  // AUTOWARE_TEST_TOOLS__AUTOWARE_TEST_TOOLS_HPP_
