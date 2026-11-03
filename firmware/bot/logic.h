// Pure logic (no Arduino calls) so it compiles and is fuzz-tested on the laptop against the
// Python model in swarm/sim/firmware_model.py. Keep it that way: hardware access lives in bot.ino.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace swarm {

constexpr int MAX_CMD = 100;
constexpr size_t MAX_PACKET_LEN = 48;
constexpr uint32_t FAILSAFE_MS = 500;    // no valid command for this long -> motors and marker off
constexpr uint32_t MARKER_MAX_MS = 2500; // single marker/laser pulse cap
// Motor duty cap, percent of full PWM. The L298N drops ~2 V, so at a full pack (8.4 V) the motors
// see ~6.4 V at 100% duty; 72% keeps the average near 4.6 V, inside the motors' 3-6 V rating.
// TUNE HERE (and PWM_CAP in swarm/sim/firmware_model.py).
constexpr int PWM_CAP_PERCENT = 72;

struct Command {
  int left = 0;
  int right = 0;
  bool marker = false;
};

struct Outputs {
  int left_permille = 0;   // signed duty, -PWM_CAP*10 .. +PWM_CAP*10
  int right_permille = 0;
  bool marker = false;
};

namespace detail {
inline int clampi(long v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : (int)v); }

// Parses -?[0-9]+ from [p, end). Saturates instead of overflowing; we only ever clamp afterwards.
inline bool parse_int(const char*& p, const char* end, long& out) {
  bool neg = false;
  if (p < end && *p == '-') { neg = true; ++p; }
  if (p >= end || *p < '0' || *p > '9') return false;
  long v = 0;
  while (p < end && *p >= '0' && *p <= '9') {
    if (v < 1000000000L) v = v * 10 + (*p - '0');
    ++p;
  }
  out = neg ? -v : v;
  return true;
}
}  // namespace detail

// "<id>,<seq>,<left>,<right>,<marker>[\r][\n]". Returns false for anything malformed or addressed
// to another bot. Same grammar as swarm/protocol.py.
inline bool parse_packet(const char* data, size_t len, int expected_id, Command& out) {
  if (len == 0 || len > MAX_PACKET_LEN) return false;
  const char* p = data;
  const char* end = data + len;
  long f[5];
  for (int i = 0; i < 5; ++i) {
    if (!detail::parse_int(p, end, f[i])) return false;
    if (i < 4) {
      if (p >= end || *p != ',') return false;
      ++p;
    }
  }
  if (p < end && *p == '\r') ++p;
  if (p < end && *p == '\n') ++p;
  if (p != end) return false;
  if (f[0] != expected_id) return false;
  out.left = detail::clampi(f[2], -MAX_CMD, MAX_CMD);
  out.right = detail::clampi(f[3], -MAX_CMD, MAX_CMD);
  out.marker = detail::clampi(f[4], 0, 1) == 1;
  return true;
}

// Failsafe, marker pulse cap and re-arm rule. Times are millis(); unsigned subtraction survives wrap.
class Safety {
 public:
  void on_command(const Command& c, uint32_t now) {
    cmd_ = c;
    last_rx_ = now;
    have_rx_ = true;
    if (!c.marker) {  // a marker=0 command re-arms the marker for the next fresh marker=1
      latched_off_ = false;
      marker_active_ = false;
    }
  }

  Outputs tick(uint32_t now) {
    Outputs o;
    if (!have_rx_ || (uint32_t)(now - last_rx_) > FAILSAFE_MS) {
      marker_active_ = false;
      return o;
    }
    if (cmd_.marker && !latched_off_) {
      if (!marker_active_) {
        marker_active_ = true;
        marker_start_ = now;
      }
      if ((uint32_t)(now - marker_start_) < MARKER_MAX_MS) {
        o.marker = true;
      } else {
        latched_off_ = true;  // pulse used up; needs marker=0 then marker=1 to fire again
        marker_active_ = false;
      }
    }
    o.left_permille = to_permille(cmd_.left);
    o.right_permille = to_permille(cmd_.right);
    return o;
  }

 private:
  // cmd (-100..100) -> permille duty, rounded half away from zero (no exact halves occur at 72%).
  static int to_permille(int cmd) {
    int v = cmd * PWM_CAP_PERCENT;  // cmd * 72 -> permille is v / 10
    return (v + (v >= 0 ? 5 : -5)) / 10;
  }

  Command cmd_;
  uint32_t last_rx_ = 0;
  uint32_t marker_start_ = 0;
  bool have_rx_ = false;
  bool marker_active_ = false;
  bool latched_off_ = false;
};

}  // namespace swarm
