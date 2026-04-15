// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__CHAIN_PRIORITY_ALLOCATOR_HPP_
#define RCLCPP__DETAIL__CHAIN_PRIORITY_ALLOCATOR_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/chain_yaml_parser.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{

struct ChainPriorityAllocation
{
  // callback_name -> (incoming_chain_id -> outgoing_chain_id)
  std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>> routing_map;
  // chain_id -> SCHED_FIFO priority (higher value = higher priority, tightest deadline = 98)
  std::unordered_map<uint32_t, uint16_t> chain_priority_map;
  // chain_name -> chain_id (0 reserved for source/unresolved)
  std::unordered_map<std::string, uint32_t> chain_name_to_id;
  // callback_name -> init-time SCHED_FIFO priority (max across all chains callback appears in)
  std::unordered_map<std::string, uint16_t> callback_init_priorities;
  // callback_name -> period from the chain with the tightest deadline for this callback
  std::unordered_map<std::string, std::uint32_t> callback_periods;
  // callback_name -> chain_id of the tightest-deadline chain (for source timer stamping)
  std::unordered_map<std::string, uint32_t> callback_source_chain_ids;
};

class RCLCPP_PUBLIC ChainPriorityAllocator
{
public:
  explicit ChainPriorityAllocator(
    std::shared_ptr<const std::unordered_map<std::string, userChain>> user_chains);

  ChainPriorityAllocation allocate(
    const std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> &
    callback_groups_by_name);

private:
  std::shared_ptr<const std::unordered_map<std::string, userChain>> user_chains_;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__CHAIN_PRIORITY_ALLOCATOR_HPP_
