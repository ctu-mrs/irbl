// PlantSway: gz-sim (Harmonic) world system that makes crop foliage move in a gusty wind.
//
// It animates VISUALS only (the Pose component of visual entities, re-sent to the rendering of
// every sensor and of the GUI), so:
//   - cameras and gpu_lidars see the plants sway / the leaves flutter (dynamic clutter in the clouds),
//   - physics, collisions and the planners' prior map (collision boxes of the world SDF) don't change,
//   - it costs no physics: a few hundred pose updates per render tick.
//
// Visuals are matched by name prefix (anywhere in the world, static models included):
//   <sway_prefix><id>     a whole plant group: rotates about its own origin (put it at the stem base on
//                         the gutter) by a small tilt that follows the wind (mean lean + gusts that
//                         travel downwind across the greenhouse + its own resonant oscillation).
//   <flutter_prefix><id>[_<k>]  leaves of that group (several leaf groups per plant group: _<k>):
//                         follow <sway_prefix><id> rigidly and add a fast, small rotation about their
//                         own origin (put it at the leaves' centroid). No matching sway visual: flutter
//                         only (low crops).
//
// <plugin filename="gz_greenhouse_dynamics" name="gz_greenhouse_dynamics::PlantSway">
//   <sway_prefix>sway_</sway_prefix>        <flutter_prefix>flutter_</flutter_prefix>
//   <wind_direction>1 0.3</wind_direction>  wind heading in the world xy plane
//   <mean_tilt_deg>0.6</mean_tilt_deg>      steady lean downwind
//   <gust_tilt_deg>2.0</gust_tilt_deg>      extra lean at the peak of a gust
//   <gust_period>7.0</gust_period>          [s] main gust period (plus 2 incommensurate harmonics)
//   <gust_speed>3.0</gust_speed>            [m/s] speed at which gust fronts cross the greenhouse
//   <osc_tilt_deg>0.8</osc_tilt_deg>        own oscillation amplitude (scaled by the gust strength)
//   <osc_freq>0.7</osc_freq>                [Hz] own oscillation frequency (+-30 % per group)
//   <flutter_deg>1.5</flutter_deg>          leaf flutter amplitude
//   <flutter_freq>2.5</flutter_freq>        [Hz] leaf flutter frequency (+-40 % per group)
//   <update_rate>30</update_rate>           [Hz] sim-time rate of the pose updates (render rate)
//   <seed>1</seed>
// </plugin>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

#include <chrono>
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace gz_greenhouse_dynamics
{
using gz::math::Pose3d;
using gz::math::Quaterniond;
using gz::math::Vector3d;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;

class PlantSway : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate
{
  struct Sway
  {
    Entity entity;
    Pose3d pose0;      // parent-relative rest pose (origin = pivot)
    Vector3d world0;   // approx. world position (gust phase), = pose0 for top-level static models
    double oscFreq, oscPhase, gain;
    Pose3d current;    // pose set at the last update
  };
  struct Flutter
  {
    Entity entity;
    Pose3d pose0;
    int sway = -1;     // index into sways_ (or -1: flutter only)
    Pose3d rel;        // rest pose relative to its sway visual's rest pose
    double freq, phase, axisAngle;
  };

 public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &, gz::sim::EventManager &) override
  {
    auto get = [&](const char *k, auto def) { return sdf->Get<decltype(def)>(k, def).first; };
    swayPrefix_ = get("sway_prefix", std::string("sway_"));
    flutterPrefix_ = get("flutter_prefix", std::string("flutter_"));
    gz::math::Vector2d d = get("wind_direction", gz::math::Vector2d(1, 0.3));
    if (d.Length() < 1e-6) d = {1, 0};
    d.Normalize();
    windDir_ = {d.X(), d.Y(), 0};
    tiltAxis_ = Vector3d::UnitZ.Cross(windDir_);  // positive rotation about it leans downwind
    meanTilt_ = deg(get("mean_tilt_deg", 0.6));
    gustTilt_ = deg(get("gust_tilt_deg", 2.0));
    gustPeriod_ = std::max(0.5, get("gust_period", 7.0));
    gustSpeed_ = std::max(0.1, get("gust_speed", 3.0));
    oscTilt_ = deg(get("osc_tilt_deg", 0.8));
    oscFreq_ = get("osc_freq", 0.7);
    flutterAmp_ = deg(get("flutter_deg", 1.5));
    flutterFreq_ = get("flutter_freq", 2.5);
    const double rate = std::max(1.0, get("update_rate", 30.0));
    period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate));
    rng_.seed(static_cast<unsigned>(get("seed", 1)));
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (!initialized_) {
      // visuals of the world SDF are all created on the first iteration
      Init(ecm);
      if (!initialized_) return;
    }
    if (info.paused || sways_.empty() && flutters_.empty()) return;
    if (info.simTime - lastUpdate_ < period_ && lastUpdate_.count() > 0) return;
    lastUpdate_ = info.simTime;
    const double t = std::chrono::duration<double>(info.simTime).count();

    for (auto &s : sways_) {
      const double g = Gust(t - windDir_.Dot(s.world0) / gustSpeed_);  // 0..1
      const double tilt = s.gain * (meanTilt_ + gustTilt_ * g +
                                    oscTilt_ * (0.4 + g) * std::sin(2 * M_PI * s.oscFreq * t + s.oscPhase));
      // small extra twist across the wind so groups don't move in perfect lockstep
      const double side = s.gain * 0.25 * oscTilt_ * std::sin(2 * M_PI * 1.3 * s.oscFreq * t + 2 * s.oscPhase);
      const Quaterniond q = Quaterniond(tiltAxis_, tilt) * Quaterniond(windDir_, side);
      // rotate about the pivot (= the visual's origin), expressed in the parent frame
      s.current = Pose3d(s.pose0.Pos(), q * s.pose0.Rot());
      Set(ecm, s.entity, s.current);
    }
    for (auto &f : flutters_) {
      const double a = flutterAmp_ * (0.5 + 0.5 * (f.sway >= 0 ? Gust(t - f.phase) : 1.0)) *
                       std::sin(2 * M_PI * f.freq * t + f.phase);
      const Vector3d axis(std::cos(f.axisAngle), std::sin(f.axisAngle), 0.3);
      const Pose3d local(Vector3d::Zero, Quaterniond(axis.Normalized(), a));
      Set(ecm, f.entity, f.sway >= 0 ? sways_[f.sway].current * f.rel * local : f.pose0 * local);
    }
  }

 private:
  static double deg(double d) { return d * M_PI / 180.0; }

  // smooth gust envelope in [0, 1]: three incommensurate harmonics, sharpened
  double Gust(double t) const
  {
    const double w = 2 * M_PI / gustPeriod_;
    const double v = 0.55 * std::sin(w * t) + 0.3 * std::sin(2.31 * w * t + 1.3) + 0.15 * std::sin(5.17 * w * t + 0.4);
    const double x = 0.5 + 0.5 * v;  // 0..1
    return x * x * (3 - 2 * x);
  }

  static void Set(EntityComponentManager &ecm, Entity e, const Pose3d &p)
  {
    auto *c = ecm.Component<gz::sim::components::Pose>(e);
    if (!c) return;
    c->Data() = p;
    ecm.SetChanged(e, gz::sim::components::Pose::typeId, gz::sim::ComponentState::PeriodicChange);
  }

  // world pose of an entity by walking up its parents (static, so done once)
  static Pose3d WorldPose(const EntityComponentManager &ecm, Entity e)
  {
    Pose3d p;
    while (e != gz::sim::kNullEntity) {
      if (auto *c = ecm.Component<gz::sim::components::Pose>(e)) p = c->Data() * p;
      auto *parent = ecm.Component<gz::sim::components::ParentEntity>(e);
      e = parent ? parent->Data() : gz::sim::kNullEntity;
    }
    return p;
  }

  void Init(EntityComponentManager &ecm)
  {
    std::uniform_real_distribution<double> u(0, 1);
    std::map<std::string, int> swayIndex;
    std::vector<std::pair<std::string, Entity>> flutterVisuals;
    ecm.Each<gz::sim::components::Visual, gz::sim::components::Name, gz::sim::components::Pose>(
        [&](const Entity &e, const gz::sim::components::Visual *, const gz::sim::components::Name *n,
            const gz::sim::components::Pose *p) {
          const std::string &name = n->Data();
          if (name.rfind(swayPrefix_, 0) == 0) {
            Sway s;
            s.entity = e;
            s.pose0 = p->Data();
            s.world0 = WorldPose(ecm, e).Pos();
            s.oscFreq = oscFreq_ * (0.7 + 0.6 * u(rng_));
            s.oscPhase = 2 * M_PI * u(rng_);
            s.gain = 0.75 + 0.5 * u(rng_);
            s.current = s.pose0;
            swayIndex[name.substr(swayPrefix_.size())] = static_cast<int>(sways_.size());
            sways_.push_back(s);
          } else if (name.rfind(flutterPrefix_, 0) == 0) {
            flutterVisuals.emplace_back(name.substr(flutterPrefix_.size()), e);
          }
          return true;
        });
    for (const auto &[id, e] : flutterVisuals) {
      Flutter f;
      f.entity = e;
      f.pose0 = ecm.Component<gz::sim::components::Pose>(e)->Data();
      // <flutter_prefix><id> or <flutter_prefix><id>_<anything>: leaves of <sway_prefix><id>
      auto it = swayIndex.find(id);
      if (const auto us = id.rfind('_'); it == swayIndex.end() && us != std::string::npos)
        it = swayIndex.find(id.substr(0, us));
      if (it != swayIndex.end()) {
        f.sway = it->second;
        f.rel = sways_[f.sway].pose0.Inverse() * f.pose0;
      }
      f.freq = flutterFreq_ * (0.6 + 0.8 * u(rng_));
      f.phase = 2 * M_PI * u(rng_);
      f.axisAngle = 2 * M_PI * u(rng_);
      flutters_.push_back(f);
    }
    // nothing found yet: the world may still be loading -- retry a few iterations
    if (sways_.empty() && flutters_.empty() && ++initTries_ < 50) return;
    initialized_ = true;
    gzmsg << "PlantSway: animating " << sways_.size() << " plant groups (" << swayPrefix_ << "*), "
          << flutters_.size() << " leaf groups (" << flutterPrefix_ << "*)\n";
  }

  std::string swayPrefix_, flutterPrefix_;
  Vector3d windDir_, tiltAxis_;
  double meanTilt_{}, gustTilt_{}, gustPeriod_{}, gustSpeed_{}, oscTilt_{}, oscFreq_{}, flutterAmp_{},
      flutterFreq_{};
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration lastUpdate_{0};
  std::mt19937 rng_;
  std::vector<Sway> sways_;
  std::vector<Flutter> flutters_;
  bool initialized_ = false;
  int initTries_ = 0;
};
}  // namespace gz_greenhouse_dynamics

GZ_ADD_PLUGIN(gz_greenhouse_dynamics::PlantSway, gz::sim::System,
              gz_greenhouse_dynamics::PlantSway::ISystemConfigure,
              gz_greenhouse_dynamics::PlantSway::ISystemPreUpdate)
