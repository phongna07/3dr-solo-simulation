// The GUI owns a separate render scene; use the same geometry as the server.
#include <sstream>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>

#include "CameraCoverage.hh"

namespace solo
{
class CameraCoverageGui final : public gz::gui::Plugin
{
  public: void LoadConfig(const tinyxml2::XMLElement *_plugin) override
  {
    for (auto uav = _plugin->FirstChildElement("uav"); uav;
         uav = uav->NextSiblingElement("uav"))
    {
      CameraCoverage coverage;
      coverage.parentName = uav->Attribute("link");
      coverage.hfov = uav->DoubleAttribute("hfov");
      coverage.aspect = uav->DoubleAttribute("aspect", 1);
      std::istringstream(uav->Attribute("offset")) >> coverage.offset;
      std::istringstream(uav->Attribute("color")) >> coverage.color;
      this->fleet.push_back(coverage);
    }
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }

  protected: bool eventFilter(QObject *_object, QEvent *_event) override
  {
    if (_event->type() == gz::gui::events::PreRender::kType)
    {
      // GzSceneManager applies poses on Render. PreRender of the next exposure
      // sees that same snapshot and updates the guides before drawing the camera.
      if (auto scene = gz::rendering::sceneFromFirstRenderEngine())
      {
        for (const auto &coverage : this->fleet)
          coverage.Update(scene);
      }
    }
    return QObject::eventFilter(_object, _event);
  }

  private: std::vector<CameraCoverage> fleet;
};
}

GZ_ADD_PLUGIN(solo::CameraCoverageGui, gz::gui::Plugin)
