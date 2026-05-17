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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>
#include <unordered_map>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/chain_yaml_parser.hpp"
#include "rclcpp/detail/chain_priority_allocator.hpp"

namespace
{

class TempYamlFile
{
public:
  explicit TempYamlFile(const std::string & contents)
  {
    const auto unique_suffix = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    path_ = std::filesystem::temp_directory_path() /
      ("rclcpp_chain_priority_allocator_" + unique_suffix + ".yaml");

    std::ofstream stream(path_);
    stream << contents;
  }

  ~TempYamlFile()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  const std::string path() const
  {
    return path_.string();
  }

private:
  std::filesystem::path path_;
};

std::shared_ptr<std::unordered_map<std::string, rclcpp::userChain>>
parse_user_chains(const std::string & yaml)
{
  TempYamlFile yaml_file(yaml);
  rclcpp::ChainYamlParser parser;
  parser.load_yaml_file(yaml_file.path());
  const bool parsed = parser.parse();
  EXPECT_TRUE(parsed);
  if (!parsed) {
    return std::make_shared<std::unordered_map<std::string, rclcpp::userChain>>();
  }
  return parser.get_user_chains();
}

std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr>
make_callback_groups(
  const std::shared_ptr<const std::unordered_map<std::string, rclcpp::userChain>> & user_chains)
{
  std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> callback_groups;
  for (const auto & [chain_name, chain] : *user_chains) {
    (void)chain_name;
    for (const auto & callback_name : chain.callbacks) {
      callback_groups.emplace(
        callback_name,
        std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::Reentrant));
    }
  }
  return callback_groups;
}

rclcpp::detail::ChainPriorityAllocation
allocate_from_yaml(const std::string & yaml)
{
  auto user_chains = parse_user_chains(yaml);
  rclcpp::detail::ChainPriorityAllocator allocator(user_chains);
  auto callback_groups = make_callback_groups(user_chains);
  return allocator.allocate(callback_groups);
}

TEST(TestChainPriorityAllocator, LinearChainPopulatesSingleRoute)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  fast:
    callbacks: [timer_a, sub_a, sub_b]
    deadline: 10
    period: 100
)");

  const auto fast_id = allocation.chain_name_to_id.at("fast");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {fast_id, 98},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_a", 98},
    {"sub_a", 98},
    {"sub_b", 98},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_a", 100u},
    {"sub_a", 100u},
    {"sub_b", 100u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_a", fast_id},
    {"sub_a", fast_id},
    {"sub_b", fast_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"sub_a", {{fast_id, fast_id}}},
    {"sub_b", {{fast_id, fast_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

TEST(TestChainPriorityAllocator, DivergenceRoutesSharedSourceIntoDifferentBranchChains)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  fast:
    callbacks: [timer_root, shared, left]
    deadline: 10
    period: 100
  slow:
    callbacks: [timer_root, shared, right]
    deadline: 20
    period: 200
)");

  const auto fast_id = allocation.chain_name_to_id.at("fast");
  const auto slow_id = allocation.chain_name_to_id.at("slow");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {fast_id, 98},
    {slow_id, 97},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_root", 98},
    {"shared", 98},
    {"left", 98},
    {"right", 97},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_root", 100u},
    {"shared", 100u},
    {"left", 100u},
    {"right", 200u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_root", fast_id},
    {"shared", fast_id},
    {"left", fast_id},
    {"right", slow_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"shared", {{fast_id, fast_id}}},
    {"left", {{fast_id, fast_id}}},
    {"right", {{fast_id, slow_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

TEST(TestChainPriorityAllocator, ConvergenceKeepsDistinctIncomingChainRoutes)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  fast:
    callbacks: [timer_fast, join, sink]
    deadline: 10
    period: 100
  slow:
    callbacks: [timer_slow, join, sink]
    deadline: 20
    period: 200
)");

  const auto fast_id = allocation.chain_name_to_id.at("fast");
  const auto slow_id = allocation.chain_name_to_id.at("slow");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {fast_id, 98},
    {slow_id, 97},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_fast", 98},
    {"timer_slow", 97},
    {"join", 98},
    {"sink", 98},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_fast", 100u},
    {"timer_slow", 200u},
    {"join", 100u},
    {"sink", 100u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_fast", fast_id},
    {"timer_slow", slow_id},
    {"join", fast_id},
    {"sink", fast_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"join", {{fast_id, fast_id}, {slow_id, slow_id}}},
    {"sink", {{fast_id, fast_id}, {slow_id, slow_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

TEST(TestChainPriorityAllocator, DiamondTopologyPreservesBranchIdentityThroughJoin)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  fast:
    callbacks: [timer_root, split, left, join, sink]
    deadline: 10
    period: 100
  slow:
    callbacks: [timer_root, split, right, join, sink]
    deadline: 20
    period: 200
)");

  const auto fast_id = allocation.chain_name_to_id.at("fast");
  const auto slow_id = allocation.chain_name_to_id.at("slow");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {fast_id, 98},
    {slow_id, 97},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_root", 98},
    {"split", 98},
    {"left", 98},
    {"right", 97},
    {"join", 98},
    {"sink", 98},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_root", 100u},
    {"split", 100u},
    {"left", 100u},
    {"right", 200u},
    {"join", 100u},
    {"sink", 100u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_root", fast_id},
    {"split", fast_id},
    {"left", fast_id},
    {"right", slow_id},
    {"join", fast_id},
    {"sink", fast_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"split", {{fast_id, fast_id}}},
    {"left", {{fast_id, fast_id}}},
    {"right", {{fast_id, slow_id}}},
    {"join", {{fast_id, fast_id}, {slow_id, slow_id}}},
    {"sink", {{fast_id, fast_id}, {slow_id, slow_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

TEST(TestChainPriorityAllocator, OverlappingChainsRouteSharedPrefixesAndSharedSinksCorrectly)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  chain_A:
    callbacks: [timer_1, sub_F, sub_M, sub_Q]
    deadline: 20
    period: 200
  chain_B:
    callbacks: [timer_1, sub_F, sub_M, sub_R]
    deadline: 60
    period: 600
  chain_C:
    callbacks: [timer_2, sub_G, sub_N, sub_R]
    deadline: 40
    period: 400
  chain_D:
    callbacks: [timer_2, sub_G, sub_N, sub_S]
    deadline: 50
    period: 500
)");

  const auto chain_a_id = allocation.chain_name_to_id.at("chain_A");
  const auto chain_b_id = allocation.chain_name_to_id.at("chain_B");
  const auto chain_c_id = allocation.chain_name_to_id.at("chain_C");
  const auto chain_d_id = allocation.chain_name_to_id.at("chain_D");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {chain_a_id, 98},
    {chain_c_id, 97},
    {chain_d_id, 96},
    {chain_b_id, 95},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_1", 98},
    {"sub_F", 98},
    {"sub_M", 98},
    {"sub_Q", 98},
    {"timer_2", 97},
    {"sub_G", 97},
    {"sub_N", 97},
    {"sub_R", 97},
    {"sub_S", 96},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_1", 200u},
    {"sub_F", 200u},
    {"sub_M", 200u},
    {"sub_Q", 200u},
    {"timer_2", 400u},
    {"sub_G", 400u},
    {"sub_N", 400u},
    {"sub_R", 400u},
    {"sub_S", 500u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_1", chain_a_id},
    {"sub_F", chain_a_id},
    {"sub_M", chain_a_id},
    {"sub_Q", chain_a_id},
    {"timer_2", chain_c_id},
    {"sub_G", chain_c_id},
    {"sub_N", chain_c_id},
    {"sub_R", chain_c_id},
    {"sub_S", chain_d_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"sub_F", {{chain_a_id, chain_a_id}}},
    {"sub_M", {{chain_a_id, chain_a_id}}},
    {"sub_Q", {{chain_a_id, chain_a_id}}},
    {"sub_G", {{chain_c_id, chain_c_id}}},
    {"sub_N", {{chain_c_id, chain_c_id}}},
    {"sub_R", {{chain_c_id, chain_c_id}, {chain_a_id, chain_b_id}}},
    {"sub_S", {{chain_c_id, chain_d_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

TEST(TestChainPriorityAllocator, XTopologyPreservesIdentityAcrossCrossingMiddleNode)
{
  const auto allocation = allocate_from_yaml(R"(
chains:
  fast:
    callbacks: [timer_left, upper_arm, cross, lower_right]
    deadline: 30
    period: 300
  slow:
    callbacks: [timer_right, lower_arm, cross, upper_left]
    deadline: 70
    period: 700
)");

  const auto fast_id = allocation.chain_name_to_id.at("fast");
  const auto slow_id = allocation.chain_name_to_id.at("slow");
  const std::unordered_map<uint32_t, uint16_t> expected_chain_priority_map = {
    {fast_id, 98},
    {slow_id, 97},
  };
  const std::unordered_map<std::string, uint16_t> expected_callback_init_priorities = {
    {"timer_left", 98},
    {"upper_arm", 98},
    {"cross", 98},
    {"lower_right", 98},
    {"timer_right", 97},
    {"lower_arm", 97},
    {"upper_left", 97},
  };
  const std::unordered_map<std::string, std::uint32_t> expected_callback_periods = {
    {"timer_left", 300u},
    {"upper_arm", 300u},
    {"cross", 300u},
    {"lower_right", 300u},
    {"timer_right", 700u},
    {"lower_arm", 700u},
    {"upper_left", 700u},
  };
  const std::unordered_map<std::string, uint32_t> expected_callback_source_chain_ids = {
    {"timer_left", fast_id},
    {"upper_arm", fast_id},
    {"cross", fast_id},
    {"lower_right", fast_id},
    {"timer_right", slow_id},
    {"lower_arm", slow_id},
    {"upper_left", slow_id},
  };
  const std::unordered_map<std::string, std::unordered_map<uint32_t, uint32_t>>
  expected_routing_map = {
    {"upper_arm", {{fast_id, fast_id}}},
    {"cross", {{fast_id, fast_id}, {slow_id, slow_id}}},
    {"lower_right", {{fast_id, fast_id}}},
    {"lower_arm", {{slow_id, slow_id}}},
    {"upper_left", {{slow_id, slow_id}}},
  };

  EXPECT_EQ(allocation.chain_priority_map, expected_chain_priority_map);
  EXPECT_EQ(allocation.callback_init_priorities, expected_callback_init_priorities);
  EXPECT_EQ(allocation.callback_periods, expected_callback_periods);
  EXPECT_EQ(allocation.callback_source_chain_ids, expected_callback_source_chain_ids);
  EXPECT_EQ(allocation.routing_map, expected_routing_map);
}

}  // namespace
