// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include "swerve_chassis_core/setpoint_generator.hpp"
using namespace swerve_chassis_core;
void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
double error(Twist a, Twist b)
{
  return std::hypot(a.vx - b.vx, a.vy - b.vy) + std::abs(a.wz - b.wz);
}
int main()
{
  Geometry g;
  Limits l;
  Generator generator;
  State state;
  Joints actual;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> random(-0.5, 0.5);
  for (int n = 0; n < 10000; ++n) {
    Twist t{random(rng), random(rng), random(rng)};
    auto j = inverse(g, t);
    check(error(forward(g, j), t) < 1e-12, "inverse/forward");
    check(residual(g, j) < 1e-12, "consistent inverse");
  }
  for (Twist target :
       {Twist{0, 0.3, 0}, Twist{0.4, 0.2, 0.3}, Twist{0, 0, 0.4}, Twist{-0.3, 0.01, 0},
        Twist{-0.3, -0.01, 0}, Twist{0.125, -0.15, 0.5}, Twist{}}) {
    for (int n = 0; n < 1500; ++n) {
      auto out = generator.step(target, actual, state, 0.01);
      check(residual(g, out.state.command) < 1e-8, "coordinated command");
      for (int i = 0; i < 4; ++i) {
        check(std::abs(out.state.command.steering[i]) <= pi / 2 + 1e-9, "steering range");
        check(std::abs(out.state.command.speed[i]) <= l.wheel_speed + 1e-9, "speed range");
        check(std::abs(out.state.command.steering[i] - state.command.steering[i]) <= 0.025 + 1e-9,
              "steering rate");
        check(std::abs(out.state.command.speed[i] - state.command.speed[i]) <= 0.4 + 1e-9,
              "drive acceleration");
      }
      auto restored = Generator(g, l).step(target, actual, state, 0.01);
      check(restored.state.command.steering == out.state.command.steering, "snapshot restoration");
      check(restored.state.command.speed == out.state.command.speed, "snapshot drive restoration");
      state = out.state;
      actual = response(actual, state.command, l, 0.01);
    }
    check(error(forward(g, actual), target) < 1e-6, "target convergence");
  }
  // Smooth feasible transitions should not insert a stop.
  state = {inverse(g, {0.3, 0, 0}), true};
  actual = state.command;
  for (int n = 0; n < 1000; ++n) {
    double f = n / 1000.0;
    auto out = generator.step({0.3, 0.1 * f, 0.1 * f}, actual, state, 0.01);
    check(std::hypot(out.velocity.vx, out.velocity.vy) > 0.25, "unnecessary stop");
    state = out.state;
    actual = state.command;
  }
  // Stalled servo must not permit continued drive acceleration.
  state = {};
  actual = {};
  for (int n = 0; n < 300; ++n) {
    auto out = generator.step({0, 0.3, 0}, actual, state, 0.01);
    state = out.state;
    check(error(out.velocity, {}) < 1e-12, "drive while steering stalled");
  }
  Joints inconsistent;
  // A finite servo may lag a stationary presteering target; this must neither
  // stall target progression nor allow driving before encoder alignment.
  state = {};
  actual = {};
  int first_drive = -1;
  for (int n = 0; n < 200; ++n) {
    const auto old = state;
    const auto out = generator.step({0, .3, 0}, actual, state, .01);
    for (int i = 0; i < 4; ++i) {
      check(std::abs(out.state.command.steering[i] - old.command.steering[i]) <= .025 + 1e-9,
            "presteering target rate");
      if (std::abs(out.state.command.speed[i]) > .05) {
        check(std::abs(out.state.command.steering[i] - actual.steering[i]) <=
                l.tracking_tolerance + l.steering_rate * .01 + 1e-9,
              "drive waits for actual steering");
        if (first_drive < 0) first_drive = n;
      }
    }
    state = out.state;
    auto servo = state.command;
    for (int i = 0; i < 4; ++i)
      servo.steering[i] = actual.steering[i] +
        (1 - std::exp(-.01 / .12)) * (servo.steering[i] - actual.steering[i]);
    actual = response(actual, servo, l, .01);
  }
  check(first_drive > 0 && first_drive < 150, "finite servo presteers without staircase waits");
  inconsistent.speed = {1, 2, 3, 4};
  check(residual(g, inconsistent) > 0.01, "inconsistency observable");
  auto p = integrate({}, {1, 0, 1}, pi / 2);
  check(std::abs(p.x - 1) < 1e-12 && std::abs(p.y - 1) < 1e-12, "SE2 integration");
  bool threw = false;
  try {
    generator.step({}, actual, {}, 0);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "invalid dt");
  std::cout << "PASS: randomized kinematics, limits, transitions, restore, "
               "stall, residual, SE2\n";
}
