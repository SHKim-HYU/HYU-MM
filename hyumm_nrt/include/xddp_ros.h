#ifndef XDDP_ROS_H
#define XDDP_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <string>

#include "xddp_packet.h"

// ---------------------------------------------------------------------------
// XddpLink — one self-healing /dev/rtpN endpoint to the Xenomai RT controller.
//
// The XDDP char device only works while the RT side has the port bound, and a
// userspace fd held open across an RT restart blocks RT from re-binding. So the
// link is UP only while its owning RT task is present in
// /proc/xenomai/sched/threads: poll() opens the device when the task appears
// and closes it when the task vanishes (releasing the port for RT to re-bind).
// This is the ROS2 port of the HDR-DUET DOWN/UP watchdog — it NEVER calls
// exit(): a missing RT side simply leaves the link DOWN and retrying, so either
// side can start/stop/restart independently.
//
// The transport (open/read/write of the binary packet:: structs on /dev/rtpN)
// is pure POSIX and byte-identical to the Noetic version, so it stays wire-
// compatible with the unchanged RT controller. Only the logging moved to
// rclcpp. Header-only / fully inline so the several hyumm_nrt executables that
// include this header share one definition without an ODR violation.
// ---------------------------------------------------------------------------
class XddpLink {
public:
  XddpLink() : logger_(rclcpp::get_logger("xddp")),
               clock_(RCL_STEADY_TIME) {}
  ~XddpLink() { closeFd(); }

  XddpLink(const XddpLink&) = delete;
  XddpLink& operator=(const XddpLink&) = delete;

  // name    : label for logs
  // port    : opens /dev/rtp<port>
  // rt_task : substring grepped in /proc/xenomai/sched/threads (the RT task that
  //           binds this port); the link is UP only while it is alive
  // enabled : operator gate — when false the link never opens
  void configure(const std::string& name, int port,
                 const std::string& rt_task, bool enabled) {
    name_ = name; port_ = port; rt_task_ = rt_task; enabled_ = enabled;
  }

  // Optional: route logs through the owning node's logger.
  void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

  bool enabled() const { return enabled_; }
  bool up()      const { return fd_ >= 0; }

  // Watchdog tick (call ~1-2 Hz, not every control cycle). Idempotent.
  void poll() {
    if (!enabled_) return;
    const bool alive = rtAlive();
    if (fd_ < 0 && alive) {
      char dev[32];
      std::snprintf(dev, sizeof(dev), "/dev/rtp%d", port_);
      int fd = ::open(dev, O_RDWR | O_NONBLOCK);
      if (fd >= 0) {
        fd_ = fd;
        RCLCPP_INFO(logger_, "XDDP link UP: %s on /dev/rtp%d (rt task '%s')",
                    name_.c_str(), port_, rt_task_.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                             "XDDP %s: open /dev/rtp%d failed: %s",
                             name_.c_str(), port_, std::strerror(errno));
      }
    } else if (fd_ >= 0 && !alive) {
      RCLCPP_WARN(logger_,
                  "XDDP link DOWN: %s - rt task '%s' gone, releasing /dev/rtp%d",
                  name_.c_str(), rt_task_.c_str(), port_);
      closeFd();
    }
  }

  // Non-blocking write of one full struct. No-op (false) while DOWN. EAGAIN
  // (RT not draining) drops the sample; a hard error tears the link down so
  // poll() re-establishes it.
  template <typename T>
  bool tryWrite(const T& msg) {
    if (fd_ < 0) return false;
    if (::write(fd_, &msg, sizeof(msg)) == static_cast<ssize_t>(sizeof(msg)))
      return true;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    if (errno == EBADF || errno == ENODEV) closeFd();
    return false;
  }

  // Non-blocking read of one full struct; true iff a whole frame arrived. Call
  // in a while() loop to drain to the latest.
  template <typename T>
  bool tryRead(T& out) {
    if (fd_ < 0) return false;
    const ssize_t n = ::read(fd_, &out, sizeof(out));
    if (n == static_cast<ssize_t>(sizeof(out))) return true;
    if (n == 0) { closeFd(); return false; }                 // EOF: RT closed
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
    if (n < 0 && (errno == EBADF || errno == ENODEV)) closeFd();
    return false;
  }

private:
  bool rtAlive() const {
    std::ifstream f("/proc/xenomai/sched/threads");
    if (!f.is_open()) return false;
    std::string line;
    while (std::getline(f, line))
      if (line.find(rt_task_) != std::string::npos) return true;
    return false;
  }
  void closeFd() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

  std::string name_, rt_task_;
  int  port_    = -1;
  int  fd_      = -1;
  bool enabled_ = false;
  rclcpp::Logger logger_;
  rclcpp::Clock  clock_;
};

#endif  // XDDP_ROS_H
