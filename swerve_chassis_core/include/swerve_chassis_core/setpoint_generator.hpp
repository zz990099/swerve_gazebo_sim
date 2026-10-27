// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_chassis_core/kinematics.hpp"
namespace swerve_chassis_core
{
class Generator
{
 public:
  Generator(Geometry geometry = {}, Limits limits = {});
  Result step(Twist desired, const Joints & measured, const State &, double dt) const;

 private:
  Geometry geometry_;
  Limits limits_;
};
}  // namespace swerve_chassis_core
