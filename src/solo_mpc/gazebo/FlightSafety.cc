#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

namespace solo
{
class FlightSafety final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(entity);
    this->link = gz::sim::Link(this->model.LinkByName(ecm, "base_link"));
    this->link.EnableVelocityChecks(ecm);
    const auto prefix = "/" + sdf->Get<std::string>("namespace");
    this->twistPublisher = this->node.Advertise<gz::msgs::Twist>(prefix + "/control/twist");
    this->enablePublisher = this->node.Advertise<gz::msgs::Boolean>(prefix + "/control/enable");
    this->motorPublisher = this->node.Advertise<gz::msgs::Actuators>(prefix + "/motor_speed");
    this->faultPublisher = this->node.Advertise<gz::msgs::Boolean>(prefix + "/safety_active");
    this->node.Subscribe(prefix + "/cmd_vel", &FlightSafety::Command, this);
    this->node.Subscribe(prefix + "/enable", &FlightSafety::Enable, this);
    for (std::size_t index = 0; index < this->joints.size(); ++index)
    {
      this->joints[index] = this->model.JointByName(ecm, "rotor_" + std::to_string(index) + "_joint");
      if (!ecm.Component<gz::sim::components::JointVelocity>(this->joints[index]))
        ecm.CreateComponent(this->joints[index], gz::sim::components::JointVelocity());
    }
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused)
      return;
    const double sim = std::chrono::duration<double>(info.simTime).count();
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->received)
    {
      this->commandSim = sim;
      this->received = false;
    }
    const auto pose = this->link.WorldPose(ecm);
    const auto velocity = this->link.WorldLinearVelocity(ecm);
    const bool ground = pose && velocity && pose->Pos().Z() < 0.27
        && std::abs(velocity->Z()) < 0.15 && velocity->Length() < 0.25;
    if (ground)
      this->groundSince = this->groundSince < 0 ? sim : this->groundSince;
    else
      this->groundSince = -1;
    const double wallAge = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - this->commandWall).count();
    if (this->active && (sim < this->commandSim || sim - this->commandSim > 0.5 || wallAge > 3.0))
      this->failsafe = true;
    this->groundSettled = this->groundSince >= 0 && sim - this->groundSince >= 1.0;
    if (this->failsafe && this->groundSettled)
      this->active = false;
    if (info.simTime - this->lastCommandPublish < std::chrono::milliseconds(10))
      return;
    this->lastCommandPublish = info.simTime;
    gz::msgs::Twist command = this->command;
    if (this->failsafe)
    {
      command.Clear();
      if (pose)
      {
        const auto descent = pose->Rot().RotateVectorReverse(gz::math::Vector3d(0, 0, -0.35));
        command.mutable_linear()->set_x(descent.X());
        command.mutable_linear()->set_y(descent.Y());
        command.mutable_linear()->set_z(descent.Z());
      }
    }
    gz::msgs::Boolean enabled;
    enabled.set_data(this->active);
    this->enablePublisher.Publish(enabled);
    this->twistPublisher.Publish(command);
    gz::msgs::Boolean fault;
    fault.set_data(this->failsafe);
    this->faultPublisher.Publish(fault);
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || info.simTime - this->lastTelemetry < std::chrono::milliseconds(50))
      return;
    this->lastTelemetry = info.simTime;
    gz::msgs::Actuators motors;
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    motors.mutable_header()->mutable_stamp()->set_sec(stamp / 1000000000);
    motors.mutable_header()->mutable_stamp()->set_nsec(stamp % 1000000000);
    for (const auto joint : this->joints)
    {
      const auto velocity = ecm.Component<gz::sim::components::JointVelocity>(joint);
      motors.add_velocity(velocity && !velocity->Data().empty() ? std::abs(velocity->Data()[0]) * 10.0 : 0.0);
    }
    this->motorPublisher.Publish(motors);
  }

  private: void Command(const gz::msgs::Twist &message)
  {
    if (!std::isfinite(message.linear().x()) || !std::isfinite(message.linear().y())
        || !std::isfinite(message.linear().z()) || !std::isfinite(message.angular().z()))
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->command = message;
    this->commandWall = std::chrono::steady_clock::now();
    this->received = true;
  }

  private: void Enable(const gz::msgs::Boolean &message)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!message.data())
    {
      if (!this->active || this->groundSettled)
      {
        this->active = false;
        this->failsafe = false;
      }
      else
        this->failsafe = true;
    }
    else if (!this->failsafe)
      this->active = true;
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Link link{gz::sim::kNullEntity};
  private: std::array<gz::sim::Entity, 4> joints{};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher twistPublisher, enablePublisher, motorPublisher, faultPublisher;
  private: std::mutex mutex;
  private: gz::msgs::Twist command;
  private: bool active{false}, failsafe{false}, received{false}, groundSettled{false};
  private: double commandSim{0}, groundSince{-1};
  private: std::chrono::steady_clock::time_point commandWall{std::chrono::steady_clock::now()};
  private: std::chrono::steady_clock::duration lastTelemetry{};
  private: std::chrono::steady_clock::duration lastCommandPublish{};
};
}

GZ_ADD_PLUGIN(solo::FlightSafety, gz::sim::System, solo::FlightSafety::ISystemConfigure,
    solo::FlightSafety::ISystemPreUpdate, solo::FlightSafety::ISystemPostUpdate)
