// Ideal three-axis gimbal for the planar visibility model.
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Node.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Sensor.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/rendering/Events.hh>
#include <sdf/Element.hh>

#include "CameraCoverage.hh"

namespace solo
{
class WorldStabilizedCamera final : public gz::sim::System,
                                   public gz::sim::ISystemConfigure
{
  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &_events) override
  {
    if (!_sdf->HasElement("camera_name") || !_sdf->HasElement("world_offset"))
    {
      gzerr << "WorldStabilizedCamera requires camera_name and world_offset\n";
      return;
    }
    this->cameraName = _sdf->Get<std::string>("camera_name");
    this->offset = _sdf->Get<gz::math::Vector3d>("world_offset");
    this->coverage.parentName = _sdf->Get<std::string>("coverage_parent");
    this->coverage.offset = this->offset;
    this->coverage.hfov = _sdf->Get<double>("coverage_hfov");
    this->coverage.aspect = _sdf->Get<double>("coverage_aspect");
    this->coverage.color = _sdf->Get<gz::math::Color>("coverage_color");
    this->connection = _events.Connect<gz::sim::events::PreRender>(
        [this]() { this->Stabilize(); });
  }

  private: void Stabilize()
  {
    // All rendering access stays on the render thread. Look up the sensor anew
    // so resets/removal cannot leave a cached camera or scene dangling.
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene)
      return;
    this->coverage.Update(scene);
    auto camera = scene->SensorByName(this->cameraName);
    if (!camera || !camera->Parent())
      return;

    // Sensors emits PreRender AFTER applying the current ECM poses and BEFORE
    // rendering. Using that same scene snapshot avoids a one-step attitude lag.
    // The parent perception link is at the body origin. Keep the virtual mount
    // vertically below it in WORLD axes, preventing a tilted mount lever from
    // moving the footprint off the XY position used by the visibility CBF.
    const auto origin = camera->Parent()->WorldPosition() + this->offset;
    // Gazebo cameras look along +X. Fixed pitch pi/2 looks down world -Z;
    // fixed roll and yaw keep both image axes aligned with the world axes.
    camera->SetWorldPose(gz::math::Pose3d(origin,
        gz::math::Quaterniond(0, GZ_PI_2, 0)));
  }

  private: std::string cameraName;
  private: gz::math::Vector3d offset;
  private: CameraCoverage coverage;
  // Destroy the event connection before the state captured by its callback.
  private: gz::common::ConnectionPtr connection;
};
}

GZ_ADD_PLUGIN(solo::WorldStabilizedCamera, gz::sim::System,
    solo::WorldStabilizedCamera::ISystemConfigure)
