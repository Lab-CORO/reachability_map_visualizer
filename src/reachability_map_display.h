#ifndef REACHABILITY_MAP_DISPLAY_H
#define REACHABILITY_MAP_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz_common/message_filter_display.hpp>
#include "reachability_map_visualizer/msg/work_space.hpp"
#endif
#include "../../../../../../../opt/ros/jazzy/include/rviz_common/rviz_common/properties/ros_topic_property.hpp"

// namespace Ogre
// {
// class SceneNode;
// }

namespace rviz_common::properties
{
class EnumProperty;
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace reachability_map_visualizer
{
class ReachMapVisual;
class ReachMapDisplay : public rviz_common::MessageFilterDisplay< reachability_map_visualizer::msg::WorkSpace >
{
  Q_OBJECT
public:
  enum Shape
  {
    Sphere,
    Cylinder,
    Cone,
    Cube,
  };



  ReachMapDisplay();
  virtual ~ReachMapDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlphaArrow();
  void updateArrowSize();

  void updateColorAndAlphaSphere();
  void updateSphereSize();

private:
  void processMessage(reachability_map_visualizer::msg::WorkSpace::ConstSharedPtr msg)override;
  // std::vector< boost::shared_ptr< ReachMapVisual > > visuals_;
  std::vector<std::shared_ptr<ReachMapVisual>> visuals_;
  rviz_common::properties::Property* arrow_category_;
  rviz_common::properties::Property* disect_category_;
  
  rviz_common::properties::Property* sphere_category_;

  rviz_common::properties::BoolProperty* do_display_arrow_;
  rviz_common::properties::ColorProperty* arrow_color_property_;
  rviz_common::properties::FloatProperty* arrow_alpha_property_;
  rviz_common::properties::FloatProperty* arrow_length_property_;
  rviz_common::properties::IntProperty* hight_max_;
  rviz_common::properties::IntProperty* hight_min_;

  rviz_common::properties::BoolProperty* do_display_sphere_;
  rviz_common::properties::ColorProperty* sphere_color_property_;
  rviz_common::properties::FloatProperty* sphere_alpha_property_;
  rviz_common::properties::FloatProperty* sphere_radius_property_;

  rviz_common::properties::IntProperty* lower_bound_reachability_;
  rviz_common::properties::IntProperty* upper_bound_reachability_;
  rviz_common::properties::BoolProperty* is_byReachability_;
  rviz_common::properties::BoolProperty* use_intensity_coloring_;
  rviz_common::properties::EnumProperty* shape_property_;
  rviz_common::properties::EnumProperty* disect_property_;
};

}  // end namespace reachability_map_visualizer
#endif  // REACHABILITY_MAP_DISPLAY_H
