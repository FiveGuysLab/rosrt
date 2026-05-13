// Copyright 2026 Open Source Robotics Foundation, Inc.
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

// Compile-time validation that publish_as_source, publish_as_intermediate, and
// the typed publish(..., chain_id) overloads instantiate cleanly for every
// supported message carrier type (unique_ptr, lvalue reference, LoanedMessage).
// The body never executes — its purpose is purely to force template
// instantiation so that any type mismatch in set_chain_id / get_chain_id
// dispatch is caught at compile time inside this build.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

TEST(TestPublisherChainIdInstantiation, all_carrier_types_compile)
{
  if (false) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_publisher_chain_id_instantiation");
    auto pub = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
    auto timer = node->create_wall_timer(1s, []() {});

    // publish_as_source — exercises each carrier type for the outgoing message
    {
      auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
      pub->publish_as_source(std::move(unique_msg), timer);
    }
    {
      test_msgs::msg::Empty ref_msg;
      pub->publish_as_source(ref_msg, timer);
    }
    {
      auto loaned = pub->borrow_loaned_message();
      pub->publish_as_source(std::move(loaned), timer);
    }

    // publish_as_intermediate — each overload (inherited unique_ptr / const ref / shared_ptr)
    {
      auto inherited = std::make_unique<test_msgs::msg::Empty>();
      test_msgs::msg::Empty out;
      pub->publish_as_intermediate(out, std::move(inherited));
    }
    {
      const test_msgs::msg::Empty inherited;
      test_msgs::msg::Empty out;
      pub->publish_as_intermediate(out, inherited);
    }
    {
      auto inherited = std::make_shared<test_msgs::msg::Empty>();
      test_msgs::msg::Empty out;
      pub->publish_as_intermediate(out, inherited);
    }

    // Typed publish overloads accepting chain_id directly
    {
      auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
      pub->publish(std::move(unique_msg), 1234u);
    }
    {
      test_msgs::msg::Empty ref_msg;
      pub->publish(ref_msg, 1234u);
    }
    {
      auto loaned = pub->borrow_loaned_message();
      pub->publish(std::move(loaned), 1234u);
    }

    rclcpp::shutdown();
  }
  SUCCEED();
}
