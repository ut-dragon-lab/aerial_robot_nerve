#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <rcl/rcl.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/service.h>
#include <rcl/timer.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ros_utils/ros_context.hpp>

#ifndef MAX_SUB_NUM
#define MAX_SUB_NUM   1
#endif

#ifndef MAX_PUB_NUM
#define MAX_PUB_NUM   1
#endif

#ifndef MAX_SRV_NUM
#define MAX_SRV_NUM   1
#endif

#ifndef MAX_TIMER_NUM
#define MAX_TIMER_NUM 1
#endif

struct SubEntry
{
  rcl_subscription_t* sub{nullptr};

  const rosidl_message_type_support_t* type_support{nullptr};
  const char* topic_name{nullptr};

  void* msg_storage{nullptr};
  rclc_subscription_callback_t cb{nullptr};
  rclc_executor_handle_invocation_t trig{ON_NEW_DATA};
};

struct PubEntry
{
  rcl_publisher_t* pub{nullptr};

  const rosidl_message_type_support_t* type_support{nullptr};
  const char* topic_name{nullptr};
};

struct SrvEntry
{
  rcl_service_t* srv{nullptr};

  const rosidl_service_type_support_t* type_support{nullptr};
  const char* service_name{nullptr};

  void* req_storage{nullptr};
  void* res_storage{nullptr};
  rclc_service_callback_t cb{nullptr};
};

struct TimerEntry
{
  rcl_timer_t* timer{nullptr};

  uint64_t period_ns{0};
  rcl_timer_callback_t cb{nullptr};
};

struct RosModuleEntityCapacity
{
  RosModuleEntityCapacity()
  : max_sub_num(MAX_SUB_NUM),
    max_pub_num(MAX_PUB_NUM),
    max_srv_num(MAX_SRV_NUM),
    max_timer_num(MAX_TIMER_NUM)
  {}

  RosModuleEntityCapacity& max_subscriptions(size_t value)
  {
    max_sub_num = value;
    return *this;
  }

  RosModuleEntityCapacity& max_publishers(size_t value)
  {
    max_pub_num = value;
    return *this;
  }

  RosModuleEntityCapacity& max_services(size_t value)
  {
    max_srv_num = value;
    return *this;
  }

  RosModuleEntityCapacity& max_timers(size_t value)
  {
    max_timer_num = value;
    return *this;
  }

  size_t max_sub_num;
  size_t max_pub_num;
  size_t max_srv_num;
  size_t max_timer_num;
};

class RosModuleBase
{
public:
  explicit RosModuleBase(
    const RosModuleEntityCapacity& capacity = RosModuleEntityCapacity())
  : max_sub_num_(capacity.max_sub_num),
    max_pub_num_(capacity.max_pub_num),
    max_srv_num_(capacity.max_srv_num),
    max_timer_num_(capacity.max_timer_num)
  {}

  virtual ~RosModuleBase() = default;

  void reserve_entities()
  {
    sub_entries_.reserve(max_sub_num_);
    pub_entries_.reserve(max_pub_num_);
    srv_entries_.reserve(max_srv_num_);
    timer_entries_.reserve(max_timer_num_);
  }
  
  virtual size_t executor_handles()
  {
    // Publishers are not added to the rclc executor.
    size_t n = sub_entries_.size() + srv_entries_.size() + timer_entries_.size();
    return n;
  }

  void set_ros_mutex_ptr(osMutexId* ros_mutex_ptr) { ros_mutex_ptr_ = ros_mutex_ptr; }

  void set_ros_ready(std::atomic<bool>* ros_ready) { ros_ready_ = ros_ready; }

  bool init_publisher_default(
    rcl_node_t& node,
    rcl_publisher_t& pub,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name)
  {
    if (pub_entries_.size() >= max_pub_num_) return false;
    if (type_support == nullptr || topic_name == nullptr) return false;

    rcl_ret_t rc = rclc_publisher_init_best_effort(
      &pub,
      &node,
      type_support,
      topic_name);

    if (rc != RCL_RET_OK) {
      (void)rcl_publisher_fini(&pub, &node);
      return false;
    }

    PubEntry e;
    e.pub = &pub;
    e.type_support = type_support;
    e.topic_name = topic_name;
    pub_entries_.push_back(e);

    ros_entities_ready_ = true;
    return true;
  }

  bool init_subscription_default(
    rcl_node_t& node,
    rcl_subscription_t& sub,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name,
    void* msg_storage,
    rclc_subscription_callback_t cb,
    rclc_executor_handle_invocation_t trig = ON_NEW_DATA)
  {
    if (sub_entries_.size() >= max_sub_num_) return false;
    if (type_support == nullptr || topic_name == nullptr) return false;
    if (msg_storage == nullptr || cb == nullptr) return false;

    rcl_ret_t rc = rclc_subscription_init_default(
      &sub,
      &node,
      type_support,
      topic_name);

    if (rc != RCL_RET_OK) {
      (void)rcl_subscription_fini(&sub, &node);
      return false;
    }

    SubEntry e;
    e.sub = &sub;
    e.type_support = type_support;
    e.topic_name = topic_name;
    e.msg_storage = msg_storage;
    e.cb = cb;
    e.trig = trig;
    sub_entries_.push_back(e);

    ros_entities_ready_ = true;
    return true;
  }

  bool init_service_default(
    rcl_node_t& node,
    rcl_service_t& srv,
    const rosidl_service_type_support_t* type_support,
    const char* service_name,
    void* req_storage,
    void* res_storage,
    rclc_service_callback_t cb)
  {
    if (srv_entries_.size() >= max_srv_num_) return false;
    if (type_support == nullptr || service_name == nullptr) return false;
    if (req_storage == nullptr || res_storage == nullptr || cb == nullptr) return false;

    rcl_ret_t rc = rclc_service_init_default(
      &srv,
      &node,
      type_support,
      service_name);

    if (rc != RCL_RET_OK) {
      (void)rcl_service_fini(&srv, &node);
      return false;
    }

    SrvEntry e;
    e.srv = &srv;
    e.type_support = type_support;
    e.service_name = service_name;
    e.req_storage = req_storage;
    e.res_storage = res_storage;
    e.cb = cb;
    srv_entries_.push_back(e);

    ros_entities_ready_ = true;
    return true;
  }
  
  bool init_timer_default(
    rclc_support_t& support,
    rcl_timer_t& timer,
    uint64_t period_ns,
    rcl_timer_callback_t cb)
  {
    if (timer_entries_.size() >= max_timer_num_) return false;
    if (period_ns == 0 || cb == nullptr) return false;

    rcl_ret_t rc = rclc_timer_init_default(
      &timer,
      &support,
      period_ns,
      cb);

    if (rc != RCL_RET_OK) {
      (void)rcl_timer_fini(&timer);
      return false;
    }

    TimerEntry e;
    e.timer = &timer;
    e.period_ns = period_ns;
    e.cb = cb;
    timer_entries_.push_back(e);

    ros_entities_ready_ = true;
    return true;
  }

  virtual void create_entities(rcl_node_t& node) = 0;

  virtual void add_to_executor(rclc_executor_t& executor)
  {
    for (size_t i = 0; i < sub_entries_.size(); ++i) {
      const auto& e = sub_entries_[i];
      (void)rclc_executor_add_subscription(
        &executor,
        e.sub,
        e.msg_storage,
        e.cb,
        e.trig);
    }

    for (size_t i = 0; i < srv_entries_.size(); ++i) {
      const auto& e = srv_entries_[i];
      (void)rclc_executor_add_service(
        &executor,
        e.srv,
        e.req_storage,
        e.res_storage,
        e.cb);
    }

    for (size_t i = 0; i < timer_entries_.size(); ++i) {
      const auto& e = timer_entries_[i];
      (void)rclc_executor_add_timer(&executor, e.timer);
    }
  }

  virtual void destroy_entities(rcl_node_t& node)
  {
    ros_entities_ready_ = false;

    for (size_t i = sub_entries_.size(); i-- > 0;) {
      if (sub_entries_[i].sub != nullptr) {
        (void)rcl_subscription_fini(sub_entries_[i].sub, &node);
        *sub_entries_[i].sub = rcl_get_zero_initialized_subscription();
      }
    }
    for (size_t i = pub_entries_.size(); i-- > 0;) {
      if (pub_entries_[i].pub != nullptr) {
        (void)rcl_publisher_fini(pub_entries_[i].pub, &node);
        *pub_entries_[i].pub = rcl_get_zero_initialized_publisher();
      }
    }
    for (size_t i = srv_entries_.size(); i-- > 0;) {
      if (srv_entries_[i].srv != nullptr) {
        (void)rcl_service_fini(srv_entries_[i].srv, &node);
        *srv_entries_[i].srv = rcl_get_zero_initialized_service();
      }
    }
    for (size_t i = timer_entries_.size(); i-- > 0;) {
      if (timer_entries_[i].timer != nullptr) {
        (void)rcl_timer_fini(timer_entries_[i].timer);
        *timer_entries_[i].timer = rcl_get_zero_initialized_timer();
      }
    }

    sub_entries_.clear();
    pub_entries_.clear();
    srv_entries_.clear();
    timer_entries_.clear();
  }

  virtual void update(){};

  virtual void publish(){};

protected:

  bool ros_entities_ready_{false};
  osMutexId* ros_mutex_ptr_ = nullptr;
  std::atomic<bool>* ros_ready_ = nullptr;

  size_t max_sub_num_{MAX_SUB_NUM};
  size_t max_pub_num_{MAX_PUB_NUM};
  size_t max_srv_num_{MAX_SRV_NUM};
  size_t max_timer_num_{MAX_TIMER_NUM};

  std::vector<SubEntry>   sub_entries_;
  std::vector<PubEntry>   pub_entries_;
  std::vector<SrvEntry>   srv_entries_;
  std::vector<TimerEntry> timer_entries_;

  inline void lock_ros_() {
    if (ros_mutex_ptr_ != nullptr && *ros_mutex_ptr_ != nullptr) {
      osMutexWait(*ros_mutex_ptr_, osWaitForever);
    }
  }

  inline void unlock_ros_() {
    if (ros_mutex_ptr_ != nullptr && *ros_mutex_ptr_ != nullptr) {
      osMutexRelease(*ros_mutex_ptr_);
    }
  }
};
