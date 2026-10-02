// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/endpoint_guard.hpp"
#include <iostream>
#include <limits>
using namespace swerve_gazebo_sim;
void check(bool condition) {
  if (!condition)
    throw std::runtime_error("endpoint regression");
}
EndpointPacket packet(double session, double sequence, bool arm, double sim, double wall,
                      double speed = 0) {
  EndpointPacket p;
  p.data = {session, sequence, arm ? 1.0 : 0.0, sim,   sim + .03, 0,     0,
            0,       0,        speed,           speed, speed,     speed, 1};
  p.received_wall_s = wall;
  p.receipt = static_cast<std::uint64_t>(session * 100 + sequence);
  return p;
}
int main() {
  try {
    EndpointTargets measured;
    EndpointGuard guard;
    auto out = guard.update(1, 10, measured, packet(1, 1, true, 1, 10));
    check(!guard.fault() && out.wheels[0] == 0);
    auto moving = packet(1, 2, false, 1.01, 10.01, 2);
    out = guard.update(1.01, 10.01, measured, moving);
    check(!guard.fault() && out.wheels[0] == 2);
    // Publisher exit: update still runs, wheel targets zero without any callback.
    out = guard.update(1.041, 10.041, measured);
    check(guard.fault() && out.wheels[0] == 0);
    out = guard.update(1.05, 10.05, measured, packet(1, 3, false, 1.05, 10.05, 2));
    check(guard.fault() && out.wheels[0] == 0);
    // Same session cannot re-arm. A moving plant cannot re-arm either.
    guard.update(1.06, 10.06, measured, packet(1, 4, true, 1.06, 10.06));
    check(guard.fault());
    measured.wheels.fill(.06);
    guard.update(1.07, 10.07, measured, packet(2, 1, true, 1.07, 10.07));
    check(guard.fault());
    measured.wheels.fill(0);
    guard.update(1.08, 10.08, measured, packet(2, 2, true, 1.08, 10.08));
    check(!guard.fault());
    // Paused simulation cannot suspend the independent wall watchdog.
    out = guard.update(1.08, 10.19, measured);
    check(guard.fault() && out.wheels[0] == 0);
    // Expiry BEFORE new admission prevents a fresh target hiding the gap.
    guard.update(2, 11, measured, packet(3, 1, true, 2, 11));
    out = guard.update(2.04, 11.04, measured, packet(3, 2, false, 2.04, 11.04, 3));
    check(guard.fault() && out.wheels[0] == 0);
    for (int defect = 0; defect < 10; ++defect) {
      EndpointGuard g;
      g.update(1, 10, measured, packet(1, 1, true, 1, 10));
      auto p = packet(1, 2, false, 1.01, 10.01, 2);
      if (defect == 0)
        p.data[0] = 2; // Session mismatch.
      if (defect == 1)
        p.data[1] = 1; // Replay with a new receipt.
      if (defect == 2)
        p.data[9] = 21;
      if (defect == 3)
        p.data[5] = 2;
      if (defect == 4)
        p.data[3] = 1.02;
      if (defect == 5)
        p.data[4] = 1.009;
      if (defect == 6)
        p.data[4] = 1.05;
      if (defect == 7)
        p.data[9] = std::numeric_limits<double>::quiet_NaN();
      if (defect == 8)
        p.received_wall_s = 9;
      if (defect == 9)
        p.malformed = true;
      out = g.update(1.01, 10.01, measured, p);
      check(g.fault() && out.wheels[0] == 0);
    }
    guard.update(3, 12, measured, packet(4, 1, true, 3, 12));
    guard.deactivate();
    guard.update(3.01, 12.01, measured, packet(4, 2, true, 3.01, 12.01));
    check(guard.fault());
    guard.update(3.02, 12.02, measured, packet(5, 1, true, 3.02, 12.02));
    check(!guard.fault());
    // A reset/backward clock cannot be concealed by an arm in the same tick.
    guard.update(0, 13, measured, packet(6, 1, true, 0, 13));
    check(guard.fault());
    guard.update(.01, 13.01, measured, packet(6, 2, true, .01, 13.01));
    check(!guard.fault());
    measured.steering[0] = std::numeric_limits<double>::quiet_NaN();
    out = guard.update(.02, 13.02, measured);
    check(guard.fault() && std::isfinite(out.steering[0]) && out.wheels[0] == 0);
    std::cout << "endpoint watchdog, replay, recovery, clocks and invalid input passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
