// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/physics_time.hpp"
#ifdef SWERVE_GAZEBO_HUMBLE
#include <ignition/plugin/Register.hh>
#else
#include <gz/plugin/Register.hh>
#endif
#include <chrono>

namespace swerve_gazebo_sim {
class PhysicsClock : public sim::System,
                     public sim::ISystemConfigure,
                     public sim::ISystemPreUpdate {
public:
  void Configure(const sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 sim::EntityComponentManager &ecm,
                 sim::EventManager &) override {
    model_ = entity;
    ecm.CreateComponent(model_, PhysicsTime(0));
  }
  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override {
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (auto clock = ecm.Component<PhysicsTime>(model_))
      clock->SetData(now, [](double a, double b) { return a == b; });
  }

private:
  sim::Entity model_ = sim::kNullEntity;
};
} // namespace swerve_gazebo_sim
#ifdef SWERVE_GAZEBO_HUMBLE
IGNITION_ADD_PLUGIN(swerve_gazebo_sim::PhysicsClock, sim::System,
                    sim::ISystemConfigure, sim::ISystemPreUpdate)
#else
GZ_ADD_PLUGIN(swerve_gazebo_sim::PhysicsClock, sim::System,
              sim::ISystemConfigure, sim::ISystemPreUpdate)
#endif
