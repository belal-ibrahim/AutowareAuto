// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.
#include <rcutils/cmdline_parser.h>
//lint -e537 NOLINT // cpplint vs pclint
#include <memory>
//lint -e537 NOLINT // cpplint vs pclint
#include <string>
#include "ndt_matching_nodes/ndt_matching_node.hpp"


int32_t main(const int32_t argc, char8_t * argv[])
{
  int32_t ret = 0;
  try {
    uint32_t proc_cpu_mask = 0U;
    const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--proc_cpu_mask");
    if (nullptr != arg) {
      proc_cpu_mask = static_cast<uint32_t>(std::stoul(arg));
    }
    int32_t proc_prio = 0;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--proc_prio");
    if (nullptr != arg) {
      proc_prio = std::stoi(arg);
    }
    size64_t proc_max_mem = 0U;
    arg = rcutils_cli_get_option(argv, &argv[argc], "--proc_max_mem");
    if (nullptr != arg) {
      proc_max_mem = std::stoul(arg);
    }
    proc_max_mem = proc_max_mem * 1028U * 1028U;
    using apex::Scheduler;
    const Scheduler proc_scheduler = (proc_prio > 0) ? Scheduler::fifo : Scheduler::other;
    const apex::Settings proc_settings{proc_prio, proc_scheduler, proc_cpu_mask, proc_max_mem};

    if (apex::pre_init("ndt_matching_node", argc, argv, proc_settings) != APEX_RET_OK) {
      throw std::runtime_error("Can't pre-init Apex");
    }
    using autoware_bridge::ndt_matching_nodes::NdtMatchingNode;
    const auto nd_ptr = std::make_shared<NdtMatchingNode>(
      "ndt_matching_node",
      std::chrono::milliseconds(100LL));  // TODO(y.tsuji): should be configured?
    if (apex::post_init() != APEX_RET_OK) {
      throw std::runtime_error("Can't post-init Apex");
    }
    nd_ptr->run();

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100LL));
    }

    nd_ptr->stop();
    nd_ptr->join();
    if (!rclcpp::shutdown()) {
      throw std::runtime_error{"Could not shutdown rclcpp"};
    }
  } catch (const std::exception & e) {
    APEX_PRINT(e.what());
    ret = __LINE__;
  } catch (...) {
    APEX_PRINT("Unknown error occured");
    ret = __LINE__;
  }

  return ret;
}
