#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// Move a simulator target around a closed path at a fixed simulated speed.
class DeterministicPathPlugin final :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &) final
  {
    this->model = Model(_entity);
    this->entity = _entity;
    if (!this->model.Valid(_ecm))
    {
      gzerr << "DeterministicPathPlugin must be attached to a model.\n";
      return;
    }

    this->speed = _sdf->Get<double>("speed", 0.75).first;
    if (this->speed <= 0.0 || !_sdf->HasElement("waypoints"))
    {
      gzerr << "DeterministicPathPlugin requires a positive speed and waypoints.\n";
      return;
    }

    auto sdf = _sdf->Clone();
    auto waypoint = sdf->GetElement("waypoints")->GetElement("waypoint");
    while (waypoint)
    {
      std::istringstream input(waypoint->Get<std::string>());
      math::Vector2d point;
      if (!(input >> point.X() >> point.Y()))
      {
        gzerr << "DeterministicPathPlugin found an invalid waypoint.\n";
        return;
      }
      this->waypoints.push_back(point);
      waypoint = waypoint->GetNextElement("waypoint");
    }

    if (this->waypoints.size() < 2)
    {
      gzerr << "DeterministicPathPlugin requires at least two waypoints.\n";
      return;
    }

    this->height = worldPose(_entity, _ecm).Pos().Z();
    for (std::size_t index = 0; index < this->waypoints.size(); ++index)
    {
      const auto &start = this->waypoints[index];
      const auto &end = this->waypoints[(index + 1) % this->waypoints.size()];
      const double length = (end - start).Length();
      if (length <= 0.0)
      {
        gzerr << "DeterministicPathPlugin requires distinct consecutive waypoints.\n";
        return;
      }
      this->segmentLengths.push_back(length);
      this->pathLength += length;
    }
    this->valid = true;
  }

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) final
  {
    if (!this->valid || _info.paused)
      return;

    const double elapsed = std::chrono::duration<double>(_info.dt).count();
    if (elapsed < 0.0)
      this->distance = 0.0;
    else
      this->distance = std::fmod(this->distance + this->speed * elapsed,
                                 this->pathLength);

    double remaining = this->distance;
    std::size_t segment = 0;
    while (remaining > this->segmentLengths[segment])
    {
      remaining -= this->segmentLengths[segment];
      segment = (segment + 1) % this->waypoints.size();
    }

    const auto &start = this->waypoints[segment];
    const auto &end = this->waypoints[(segment + 1) % this->waypoints.size()];
    const math::Vector2d direction = end - start;
    const math::Vector2d position =
        start + direction * (remaining / this->segmentLengths[segment]);
    const double yaw = std::atan2(direction.Y(), direction.X());
    this->model.SetWorldPoseCmd(
        _ecm,
        math::Pose3d(position.X(), position.Y(), this->height, 0, 0, yaw));
  }

  private: Entity entity{kNullEntity};
  private: Model model{kNullEntity};
  private: std::vector<math::Vector2d> waypoints;
  private: std::vector<double> segmentLengths;
  private: double speed{0.75};
  private: double height{0.0};
  private: double distance{0.0};
  private: double pathLength{0.0};
  private: bool valid{false};
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::DeterministicPathPlugin,
    gz::sim::System,
    gz::sim::systems::DeterministicPathPlugin::ISystemConfigure,
    gz::sim::systems::DeterministicPathPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::DeterministicPathPlugin,
    "DeterministicPathPlugin")
