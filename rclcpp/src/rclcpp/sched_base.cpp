#include "rclcpp/sched_base.hpp"

namespace rclcpp {
namespace sched {

static_assert(
    std::atomic<uint64_t>::is_always_lock_free,
    "std::atomic<uint64_t> must be lock-free for signal-handler-safe runtime updates");

static_assert(
    std::atomic<uint32_t>::is_always_lock_free,
    "std::atomic<uint32_t> must be lock-free for signal-handler-safe source_chain_id updates");

void
SchedBase::set_sched_attr(const SchedAttr& sched_attr) {
    this->sched_attr = sched_attr;
    // Keep the runtime atomic in sync with the struct at init time.
    set_policy_priority(sched_attr.sched_policy, sched_attr.sched_priority);
}
};
};


