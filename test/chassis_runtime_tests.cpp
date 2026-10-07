// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/chassis_runtime.hpp"
#include "swerve_gazebo_sim/command_write.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace swerve_gazebo_sim;
void require(bool v, const char *message) {
  if (!v)
    throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a - b) < 1e-7; }
struct Fixture {
  ChassisRuntime runtime;
  JointTargets measured;
  Command command;
  int tick = 0;
  Fixture() {
    command.session = 1;
    command.reset = true;
    step();
    command.reset = false;
  }
  JointTargets step(bool follow = true, bool send = true) {
    const double now = ++tick * .01;
    command.stamp = now;
    command.wall = now;
    ++command.sequence;
    ++command.receipt;
    auto out = runtime.update(now, now, measured, send ? &command : nullptr);
    if (follow)
      measured = out;
    return out;
  }
  void run(int n) {
    for (int i = 0; i < n; ++i)
      step();
  }
  void change(Mode mode, Twist entry) {
    command.velocity = {};
    command.mode = mode;
    command.entry = entry;
    ++command.request;
    step();
    for (int i = 0;
         i < 450 && !runtime.confirmed() && runtime.fault() == Fault::None; ++i)
      step();
    require(runtime.confirmed() && runtime.actual_mode() == mode,
            "mode confirmation");
  }
};
int main() {
  try {
    const std::array<Twist, 3> moves = {Twist{.3, 0, .15}, Twist{0, 0, .4},
                                        Twist{0, .3, 0}};
    for (int from = 0; from < 3; ++from)
      for (int to = 0; to < 3; ++to)
        if (from != to) {
          Fixture f;
          f.change(static_cast<Mode>(from), moves[from]);
          f.command.velocity = moves[from];
          f.run(100);
          require(f.runtime.fault() == Fault::None, "drive before switch");
          const auto old = f.measured.steering;
          f.command.velocity = {};
          f.command.mode = static_cast<Mode>(to);
          f.command.entry = moves[to];
          ++f.command.request;
          auto out = f.step(false);
          require(out.steering == old, "must brake before steering");
          f.measured = out;
          for (int i = 0; i < 450 && !f.runtime.confirmed() &&
                          f.runtime.fault() == Fault::None;
               ++i)
            f.step();
          if (!f.runtime.confirmed())
            std::cerr << "transition " << from << " -> " << to
                      << " phase=" << int(f.runtime.phase())
                      << " fault=" << int(f.runtime.fault())
                      << " angle=" << f.measured.steering[0]
                      << " target=" << f.runtime.entry_angles()[0] << "\n";
          require(f.runtime.confirmed() &&
                      f.runtime.actual_mode() == static_cast<Mode>(to),
                  "all six switches");
          f.command.velocity = moves[to];
          f.run(100);
          require(f.runtime.fault() == Fault::None, "drive after switch");
          const auto angles = f.measured.steering;
          f.command.velocity = {};
          f.run(100);
          require(f.measured.steering == angles &&
                      f.runtime.actual_mode() == static_cast<Mode>(to),
                  "zero retains mode/steering");
        }
    {
      Fixture f;
      f.change(Mode::Crab, {.3, .2, 0});
      f.command.velocity = {.3, .2, 0};
      f.run(100);
      auto v = f.runtime.forward(f.measured);
      require(near(v.vx, .3) && near(v.vy, .2), "crab kinematics");
      f.command.velocity = {-.3, -.2, 0};
      f.run(200);
      v = f.runtime.forward(f.measured);
      require(near(v.vx, -.3) && near(v.vy, -.2), "reverse");
      f.command.velocity = {0, -.3, 0};
      f.run(300);
      require(f.runtime.fault() == Fault::None,
              "same-mode realignment accepts velocity refresh");
      const auto receipt = f.runtime.entry_angles();
      f.command.velocity = {.3, 0, 0};
      f.run(300);
      require(f.runtime.entry_angles() == receipt,
              "entry receipt is immutable");
    }
    {
      Fixture f;
      f.command.velocity = {100, 0, 0};
      double previous = 0;
      for (int i = 0; i < 300; ++i) {
        auto out = f.step();
        require(out.wheels[0] - previous <= .4 + 1e-9, "wheel slew");
        require(std::abs(out.wheels[0]) <= 20, "wheel bound");
        previous = out.wheels[0];
      }
      require(f.runtime.forward(f.measured).vx <= .8 + 1e-9,
              "body target bound");
      for (int i = 0; i < 60; ++i)
        f.step(true, false);
      require(f.runtime.fault() == Fault::Timeout, "publisher silence");
      f.run(5);
      require(f.runtime.phase() == Phase::Fault,
              "ordinary command cannot recover");
      f.command.velocity = {};
      f.command.reset = true;
      ++f.command.session;
      f.step();
      require(f.runtime.confirmed(), "explicit stopped recovery");
    }
    {
      Fixture f;
      f.command.mode = Mode::Crab;
      f.command.velocity = {.1, 0, 0};
      f.step();
      require(f.runtime.fault() == Fault::Command,
              "mode cannot change without request");
    }
    {
      Fixture f;
      f.command.mode = Mode::Spin;
      f.command.entry = {0, 0, 1};
      ++f.command.request;
      f.step(false);
      f.command.entry.wz = -1;
      f.step(false);
      require(f.runtime.fault() == Fault::Command, "request mutation");
    }
    {
      Fixture f;
      f.command.velocity.vx = std::numeric_limits<double>::quiet_NaN();
      auto out = f.step();
      require(f.runtime.fault() == Fault::Command && out.wheels[0] == 0,
              "NaN command");
    }
    {
      Fixture f;
      f.command.velocity = {.2, 0, 0};
      f.run(20);
      auto p = f.command;
      p.receipt++;
      p.sequence = f.runtime.sequence();
      p.stamp = (f.tick + 1) * .01;
      p.wall = p.stamp;
      f.runtime.update(p.stamp, p.wall, f.measured, &p);
      require(f.runtime.fault() == Fault::Command, "sequence replay");
    }
    {
      Fixture f;
      auto p = f.command;
      p.receipt++;
      p.stamp = -1;
      f.runtime.update(.02, .02, f.measured, &p);
      require(f.runtime.fault() == Fault::Command, "invalid source stamp");
    }
    {
      Fixture f;
      auto p = f.command;
      p.receipt++;
      p.stamp = 1;
      f.runtime.update(.02, .02, f.measured, &p);
      require(f.runtime.fault() == Fault::Command, "future source stamp");
    }
    {
      Fixture f;
      f.run(20);
      f.runtime.update(.01, 1, f.measured);
      require(f.runtime.fault() == Fault::Clock, "clock rollback");
    }
    {
      Fixture f;
      f.runtime.update(.01, 1, f.measured);
      require(f.runtime.fault() == Fault::Clock, "wall pause gap");
    }
    {
      Fixture f;
      f.measured.steering[0] = std::numeric_limits<double>::quiet_NaN();
      auto out = f.step(false);
      require(f.runtime.fault() == Fault::Feedback &&
                  std::isfinite(out.steering[0]),
              "invalid feedback stop");
    }
    {
      Fixture f;
      f.command.mode = Mode::Spin;
      f.command.entry = {0, 0, 1};
      ++f.command.request;
      for (int i = 0; i < 510; ++i)
        f.step(false);
      require(f.runtime.fault() == Fault::Transition,
              "measured alignment required");
    }
    {
      Fixture f;
      f.command.velocity = {.2, 0, 0};
      f.run(20);
      f.command.reset = true;
      f.command.velocity = {};
      ++f.command.session;
      f.step(false);
      require(f.runtime.fault() == Fault::Command, "moving reset rejected");
    }
    Config bad;
    bad.max_steering_rate = 0;
    bool threw = false;
    try {
      ChassisRuntime r(bad);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    require(threw, "configuration validation");
    std::cout << "Independent chassis kinematics, six transitions, timing and "
                 "recovery passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
