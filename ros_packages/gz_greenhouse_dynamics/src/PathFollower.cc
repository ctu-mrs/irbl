// PathFollower: gz-sim (Harmonic) model system that drives a STATIC model along a looping polyline
// (forklifts, carts): it moves the model's Pose component, so cameras / gpu_lidars / depth cameras see
// it move. Visual-only like PlantSway: no physics, no collision with the vehicles (they pass through
// it; the onboard sensors are what has to avoid it).
//
// <model name="forklift_1"> <static>true</static> ...
//   <plugin filename="gz_greenhouse_dynamics" name="gz_greenhouse_dynamics::PathFollower">
//     <waypoints>-17 -16  -17 16</waypoints>   x y pairs, world frame (looped: last -> first)
//     <speed>1.2</speed>                       [m/s] along the segments
//     <turn_rate>0.8</turn_rate>               [rad/s] turning on the spot at each waypoint
//     <dwell>3</dwell>                         [s] stop at each waypoint (loading)
//     <z>0</z>  <yaw_offset>0</yaw_offset>     model height, heading of the model's +x vs. travel
//     <reverse_every_other>false</reverse_every_other>  true: drive back without turning (shuttle)
//     <update_rate>30</update_rate>            [Hz] sim time
//   </plugin>
// </model>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/RegisterMore.hh>  // second plugin of the library (PlantSway.cc has Register.hh)
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace gz_greenhouse_dynamics
{
class PathFollower : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  struct Phase
  {
    double duration;
    double x0, y0, yaw0, x1, y1, yaw1;
  };

 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    entity_ = entity;
    std::vector<double> v;
    std::istringstream in(sdf->Get<std::string>("waypoints", "").first);
    for (double d; in >> d;) v.push_back(d);
    const double speed = std::max(0.05, sdf->Get<double>("speed", 1.0).first);
    const double turn = std::max(0.05, sdf->Get<double>("turn_rate", 0.8).first);
    const double dwell = std::max(0.0, sdf->Get<double>("dwell", 0.0).first);
    const bool shuttle = sdf->Get<bool>("reverse_every_other", false).first;
    z_ = sdf->Get<double>("z", 0.0).first;
    yawOffset_ = sdf->Get<double>("yaw_offset", 0.0).first;
    const double rate = std::max(1.0, sdf->Get<double>("update_rate", 30.0).first);
    period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate));
    const size_t n = v.size() / 2;
    if (n < 2) {
      gzerr << "PathFollower: need at least 2 waypoints (x y pairs)\n";
      return;
    }
    // phases: [dwell] [turn] drive, per segment
    double heading = std::atan2(v[3] - v[1], v[2] - v[0]);
    for (size_t i = 0; i < n; ++i) {
      const double x0 = v[2 * i], y0 = v[2 * i + 1];
      const double x1 = v[2 * ((i + 1) % n)], y1 = v[2 * ((i + 1) % n) + 1];
      double h = std::atan2(y1 - y0, x1 - x0);
      if (shuttle && (i % 2 == 1)) h += M_PI;  // drive this segment backwards
      if (dwell > 0) phases_.push_back({dwell, x0, y0, heading, x0, y0, heading});
      double dh = std::atan2(std::sin(h - heading), std::cos(h - heading));
      if (std::abs(dh) > 1e-3) phases_.push_back({std::abs(dh) / turn, x0, y0, heading, x0, y0, heading + dh});
      heading += dh;
      phases_.push_back({std::hypot(x1 - x0, y1 - y0) / speed, x0, y0, heading, x1, y1, heading});
    }
    for (const auto &p : phases_) cycle_ += p.duration;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (phases_.empty() || info.paused) return;
    if (lastUpdate_.count() > 0 && info.simTime - lastUpdate_ < period_) return;
    lastUpdate_ = info.simTime;
    double t = std::fmod(std::chrono::duration<double>(info.simTime).count(), cycle_);
    for (const auto &p : phases_) {
      if (t > p.duration) {
        t -= p.duration;
        continue;
      }
      const double a = p.duration > 0 ? t / p.duration : 1.0;
      const double s = a * a * (3 - 2 * a);  // ease in / out
      const gz::math::Pose3d pose(p.x0 + (p.x1 - p.x0) * s, p.y0 + (p.y1 - p.y0) * s, z_, 0, 0,
                                  p.yaw0 + (p.yaw1 - p.yaw0) * s + yawOffset_);
      if (auto *c = ecm.Component<gz::sim::components::Pose>(entity_)) {
        c->Data() = pose;
        ecm.SetChanged(entity_, gz::sim::components::Pose::typeId, gz::sim::ComponentState::PeriodicChange);
      }
      return;
    }
  }

 private:
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  std::vector<Phase> phases_;
  double cycle_ = 0, z_ = 0, yawOffset_ = 0;
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration lastUpdate_{0};
};
}  // namespace gz_greenhouse_dynamics

GZ_ADD_PLUGIN(gz_greenhouse_dynamics::PathFollower, gz::sim::System,
              gz_greenhouse_dynamics::PathFollower::ISystemConfigure,
              gz_greenhouse_dynamics::PathFollower::ISystemPreUpdate)
