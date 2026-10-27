// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_chassis_core/types.hpp"
namespace swerve_chassis_core
{
Joints inverse(const Geometry &, Twist, const Wheels & reference = {});
Twist forward(const Geometry &, const Joints &);
double residual(const Geometry &, const Joints &);
Twist scale(Twist, double);
Pose integrate(Pose, Twist, double);
// Nominal rate-limited response; residual must be checked before using forward.
Joints response(const Joints &, const Joints &, const Limits &, double);
}  // namespace swerve_chassis_core
