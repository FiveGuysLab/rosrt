#ifndef RCLCPP__SCHED_BASE_HPP_
#define RCLCPP__SCHED_BASE_HPP_
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <stdint.h>
#include <cstring>
#include <pthread.h>
#include <atomic>
#include <string>

#if defined(__x86_64__) || defined(_M_X64)
    #define PADDING_SIZE 90
#else
    #define PADDING_SIZE 26 // Speculative, not tested
#endif

namespace rclcpp {
// forward declarations
namespace executors {
    class SingleThreadedExecutor;
    class NoExecutor;
}; // rclcpp::executors
namespace sched {

struct pthread_struct {
    void* __unused[PADDING_SIZE];
    pid_t tid;
};

struct SchedAttr {
    uint32_t size = sizeof(SchedAttr);              /* Size of this structure */
    uint32_t sched_policy = SCHED_RR;      /* Policy (SCHED_*) */
    uint64_t sched_flags = 0;       /* Flags */
    int32_t  sched_nice = 0;        /* Nice value (SCHED_OTHER, SCHED_BATCH) */
    uint32_t sched_priority = 10;    /* Staticq priority (SCHED_FIFO, SCHED_RR) */

    /* For SCHED_DEADLINE */
    uint64_t sched_runtime = 0;
    uint64_t sched_deadline = 0;
    uint64_t sched_period = 0;

    /* Utilization hints, unused for our purpose,
       may enable in the future*/
    uint32_t sched_util_min = 0;
    uint32_t sched_util_max = 0;
};


/** Since pthread does not expose pid to us, this is a hack to get the (linux) pid.
 *  This might be dangerous and non-portable.
 */
inline pid_t
get_pid(pthread_t threadid) {
    auto pthread_id = ((pthread_struct*) threadid);
    /* this may occur if the thread is detached from the current thread, use this
       function before calling detach() */
    if (pthread_id == nullptr) {
        printf("nullptr is passed to get_pid!\n");
        return 0;
    }
    return pthread_id->tid;
}

/** Equality for SchedAttr, use it to prevent unnecessary syscall. */
inline bool
operator==(const SchedAttr& lhs, const SchedAttr& rhs) {
    return std::memcmp(&lhs, &rhs, sizeof(SchedAttr)) == 0;
}

/** Inequality for SchedAttr, use it to prevent unnecessary syscall. */
inline bool
operator!=(const SchedAttr& lhs, const SchedAttr& rhs) {
    return !(lhs==rhs);
}

inline long
syscall_sched_setattr(pid_t pid, SchedAttr* sched_attr) {
    /* flags are currently unused, may enable in the future */
    return syscall(SYS_sched_setattr, pid, sched_attr, 0);
}

class SchedBase {
friend class executors::SingleThreadedExecutor;
friend class executors::NoExecutor;
public:
    virtual ~SchedBase() = default;

    /// Init-only: set the full sched_attr struct. At runtime, use set_policy_priority().
    virtual void
    set_sched_attr(const SchedAttr& sched_attr);

    /// Atomic, signal-safe runtime update of (policy, priority).
    virtual void
    set_policy_priority(uint32_t policy, uint32_t priority)
    {
        uint64_t packed = (static_cast<uint64_t>(policy) << 32) |
            static_cast<uint64_t>(priority);
        current_policy_priority_.store(packed, std::memory_order_release);
    }

    /// Build a SchedAttr suitable for sched_setattr() from the immutable struct
    /// + the currently-active policy/priority atomic.
    SchedAttr
    get_current_sched_attr() const
    {
        SchedAttr attr = sched_attr;
        uint64_t packed = current_policy_priority_.load(std::memory_order_acquire);
        attr.sched_policy = static_cast<uint32_t>(packed >> 32);
        attr.sched_priority = static_cast<uint32_t>(packed & 0xFFFFFFFFu);
        return attr;
    }

    uint32_t
    get_current_priority() const
    {
        uint64_t packed = current_policy_priority_.load(std::memory_order_acquire);
        return static_cast<uint32_t>(packed & 0xFFFFFFFFu);
    }

    virtual void
    set_callback_name(const std::string& callback_name)
    {
        callback_name_ = callback_name;
    }

    virtual const std::string&
    get_callback_name() const
    {
        return callback_name_;
    }

    const SchedAttr &
    get_sched_attr() const
    {
        return sched_attr;
    }

    void
    set_cpu_affinity(const cpu_set_t & mask)
    {
        cpu_affinity_mask = mask;
        has_cpu_affinity = true;
    }

    /// Chain ID stamped onto outgoing messages by source timers.
    /// Init: set from allocation. Runtime: updated atomically at MCR time.
    std::atomic<uint32_t> source_chain_id{0};

    /// Only valid when has_cpu_affinity is true.
    cpu_set_t cpu_affinity_mask = {};
    bool has_cpu_affinity = false;

protected:
    /// Kernel-facing scheduling attributes. Init-only writes; immutable at runtime.
    SchedAttr sched_attr;

    /// Packed (policy << 32) | priority for atomic runtime updates.
    std::atomic<uint64_t> current_policy_priority_{0};

    std::string callback_name_;
};



}; // rclcpp::sched
}; //rclcpp

#endif
