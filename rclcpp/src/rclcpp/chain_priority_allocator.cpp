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

#include "rclcpp/detail/chain_priority_allocator.hpp"

#include "rclcpp/logging.hpp"

namespace rclcpp
{
namespace detail
{
namespace
{
rclcpp::Logger get_chain_priority_logger()
{
  return rclcpp::get_logger("ChainPriorityAllocator");
}
}  // namespace

ChainPriorityAllocator::ChainPriorityAllocator(
  std::shared_ptr<const std::unordered_map<std::string, userChain>> user_chains)
: user_chains_(std::move(user_chains))
{
}

ChainPriorityAllocation ChainPriorityAllocator::allocate(
  const std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> &
  callback_groups_by_name)
{
  ChainPriorityAllocation allocation;
  const auto logger = get_chain_priority_logger();

  // 1. Assign numeric chain IDs (0 reserved for source/unresolved)
  uint32_t next_id = 1;
  for (const auto & [name, chain] : *user_chains_) {
    allocation.chain_name_to_id[name] = next_id++;
  }

  // 2. Build chain priority map.
  //    SCHED_FIFO: higher value = higher priority (range 1-99).
  //    Tightest deadline -> highest priority, starting at 98 and decrementing.
  //    99 is reserved for executor housekeeping threads.
  std::map<uint32_t, std::vector<uint32_t>> deadline_to_chains;
  for (const auto & [name, chain] : *user_chains_) {
    deadline_to_chains[chain.deadline].push_back(allocation.chain_name_to_id[name]);
  }
  uint16_t prio = 98;
  for (const auto & [deadline, cids] : deadline_to_chains) {
    for (uint32_t cid : cids) {
      allocation.chain_priority_map[cid] = prio;
      if (prio > 1) {
        prio--;
      }
    }
  }

  // 3. Compute per-callback init-time priority, period, and source_chain_id.
  //    Priority: max SCHED_FIFO priority across all chains the callback appears in.
  //    Period: period of the chain with the tightest (min) deadline.
  //    source_chain_id: the chain_id corresponding to the tightest-deadline chain
  //      (used by source timers to stamp outgoing messages).
  std::unordered_map<std::string, uint32_t> callback_min_deadline;
  for (const auto & [name, chain] : *user_chains_) {
    uint32_t cid = allocation.chain_name_to_id[name];
    uint16_t chain_prio = allocation.chain_priority_map[cid];
    for (const auto & cb_name : chain.callbacks) {
      // Init priority: keep max (higher value = higher priority = tighter deadline)
      auto [prio_it, prio_newly_inserted] =
        allocation.callback_init_priorities.emplace(cb_name, chain_prio);
      if (!prio_newly_inserted && chain_prio > prio_it->second) {
        prio_it->second = chain_prio;
      }

      // Period and source_chain_id: track min deadline, use that chain's values
      auto [dl_it, dl_inserted] = callback_min_deadline.emplace(cb_name, chain.deadline);
      if (dl_inserted) {
        allocation.callback_periods.emplace(cb_name, chain.period);
        allocation.callback_source_chain_ids.emplace(cb_name, cid);
      } else if (chain.deadline < dl_it->second) {
        dl_it->second = chain.deadline;
        allocation.callback_periods[cb_name] = chain.period;
        allocation.callback_source_chain_ids[cb_name] = cid;
      }
    }
  }

  // 4. Build routing map: (callback_name, incoming_chain_id) -> outgoing_chain_id
  //    Process chains in deadline order (tightest first) so that shared prefix
  //    entries from higher-priority chains are in the map when looser chains iterate.
  //    Track the propagated chain_id through each chain — at shared prefix callbacks,
  //    the chain_id stays locked to the tighter chain's id. At the diverge point,
  //    the entry maps the propagated id to this chain's id.
  std::vector<std::pair<std::string, uint32_t>> chains_by_deadline;
  for (const auto & [name, chain] : *user_chains_) {
    chains_by_deadline.emplace_back(name, chain.deadline);
  }
  std::sort(chains_by_deadline.begin(), chains_by_deadline.end(),
    [](const auto & a, const auto & b) { return a.second < b.second; });

  for (const auto & [name, deadline] : chains_by_deadline) {
    const auto & chain = user_chains_->at(name);
    uint32_t cid = allocation.chain_name_to_id[name];

    // Simulates what chain_id the message will carry at each position in this
    // chain at runtime. Initialized from the source timer's source_chain_id,
    // then updated as we walk through callbacks. At shared prefix callbacks
    // (already claimed by a tighter-deadline chain in the routing map), it stays
    // locked to that chain's id. At the diverge point or unique callbacks, it
    // flips to this chain's cid.
    const auto & source_cb = chain.callbacks[0];
    auto src_it = allocation.callback_source_chain_ids.find(source_cb);
    if (src_it == allocation.callback_source_chain_ids.end()) {
      RCLCPP_WARN(
        logger,
        "Chain '%s': source callback '%s' has no source_chain_id; skipping chain",
        name.c_str(), source_cb.c_str());
      continue;
    }
    uint32_t expected_msg_chain_id = src_it->second;

    for (size_t i = 1; i < chain.callbacks.size(); ++i) {
      const auto & cb_name = chain.callbacks[i];
      if (callback_groups_by_name.find(cb_name) == callback_groups_by_name.end()) {
        RCLCPP_WARN(
          logger,
          "Chain '%s' references unknown callback '%s'; skipping",
          name.c_str(), cb_name.c_str());
        continue;
      }

      auto & inner = allocation.routing_map[cb_name];
      auto existing = inner.find(expected_msg_chain_id);
      if (existing != inner.end() && existing->second != cid) {
        // Shared prefix: this callback already resolves expected_msg_chain_id
        // to a different chain (tighter deadline, processed earlier). The message
        // will carry that chain's id through this callback unchanged.
        continue;
      }

      // Either new entry or confirms our chain. Insert and update.
      inner.emplace(expected_msg_chain_id, cid);
      expected_msg_chain_id = cid;
    }
  }

  RCLCPP_INFO(logger, "Chain priority allocation complete: %zu chains, %zu routing entries",
    allocation.chain_name_to_id.size(), allocation.routing_map.size());
  for (const auto & [cid, p] : allocation.chain_priority_map) {
    RCLCPP_INFO(logger, "  chain_id %u -> SCHED_FIFO priority %u", cid, p);
  }
  for (const auto & [cb, p] : allocation.callback_init_priorities) {
    RCLCPP_INFO(logger, "  callback '%s' -> init priority %u, period %u ns",
      cb.c_str(), p, allocation.callback_periods[cb]);
  }

  return allocation;
}

}  // namespace detail
}  // namespace rclcpp
