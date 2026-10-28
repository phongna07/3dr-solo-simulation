// Render-only guides shared by the sensor scene and the independent GUI scene.
#ifndef SOLO_CAMERA_COVERAGE_HH_
#define SOLO_CAMERA_COVERAGE_HH_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

namespace solo
{
class CameraCoverage
{
  // Reserved in world.py; UAV cameras and GPU LiDAR exclude this bit.
  public: static constexpr uint32_t visibility = 0x20;
  public: std::string parentName;
  public: gz::math::Vector3d offset;
  public: gz::math::Color color;
  public: double hfov{0};
  public: double aspect{1};

  // Call only on the scene's render thread, after current poses are applied.
  public: void Update(const gz::rendering::ScenePtr &_scene) const
  {
    const auto parent = _scene->VisualByName(this->parentName);
    if (!parent)
    {
      this->Hide(_scene);
      return;
    }
    const auto origin = parent->WorldPosition() + this->offset;
    if (origin.Z() <= 0.05)
    {
      this->Hide(_scene);
      return;
    }

    // Fixed pitch pi/2: image width spans world Y, image height spans world X.
    // Intersect the four boundary rays with z=0. Lift only the outline to avoid
    // z-fighting with the ground; obstacles still occlude these depth-tested guides.
    const double halfY = origin.Z() * std::tan(this->hfov / 2);
    const double halfX = halfY / this->aspect;
    const std::array<gz::math::Vector3d, 4> corners{{
      {origin.X() - halfX, origin.Y() - halfY, 0.025},
      {origin.X() + halfX, origin.Y() - halfY, 0.025},
      {origin.X() + halfX, origin.Y() + halfY, 0.025},
      {origin.X() - halfX, origin.Y() + halfY, 0.025}}};
    for (size_t i = 0; i < corners.size(); ++i)
    {
      this->Segment(_scene, i, origin, corners[i], 0.018);
      this->Segment(_scene, i + 4, corners[i], corners[(i + 1) % 4], 0.035);
    }
  }

  private: void Hide(const gz::rendering::ScenePtr &_scene) const
  {
    for (size_t i = 0; i < 8; ++i)
    {
      if (auto visual = _scene->VisualByName(this->Name(i)))
        visual->SetVisible(false);
    }
  }

  private: std::string Name(size_t _i) const
  {
    return this->parentName + "::camera_coverage_" + std::to_string(_i);
  }

  private: void Segment(const gz::rendering::ScenePtr &_scene, size_t _i,
      const gz::math::Vector3d &_a, const gz::math::Vector3d &_b,
      double _diameter) const
  {
    // Look up nodes anew so a scene reset cannot leave cached pointers dangling.
    auto visual = _scene->VisualByName(this->Name(_i));
    if (!visual)
    {
      visual = _scene->CreateVisual(this->Name(_i));
      visual->AddGeometry(_scene->CreateCylinder());
      auto material = _scene->CreateMaterial();
      // Ogre2 adds illumination to emissive color even when LightingEnabled is
      // false. Suppress reflected light so pale fleet colors do not clip white
      // in the overhead recording, and retain their hue in building shadows.
      material->SetAmbient(gz::math::Color::Black);
      material->SetDiffuse(gz::math::Color::Black);
      material->SetSpecular(gz::math::Color::Black);
      // The panel palette is sRGB; Ogre2's emissive input is linear RGB.
      const auto linear = [](float channel)
      {
        return channel <= 0.04045f ? channel / 12.92f :
            std::pow((channel + 0.055f) / 1.055f, 2.4f);
      };
      material->SetEmissive({linear(this->color.R()), linear(this->color.G()),
          linear(this->color.B()), this->color.A()});
      material->SetLightingEnabled(false);
      material->SetCastShadows(false);
      material->SetReceiveShadows(false);
      visual->SetMaterial(material, false);
      visual->SetVisibilityFlags(visibility);
      // Keep world-space guides outside the UAV's rotating/scaled hierarchy.
      _scene->RootVisual()->AddChild(visual);
    }
    const auto delta = _b - _a;
    gz::math::Quaterniond rotation;
    rotation.SetFrom2Axes(gz::math::Vector3d::UnitZ, delta);
    visual->SetWorldPose(gz::math::Pose3d((_a + _b) / 2, rotation));
    visual->SetLocalScale({_diameter, _diameter, delta.Length()});
    visual->SetVisible(true);
  }
};
}
#endif
