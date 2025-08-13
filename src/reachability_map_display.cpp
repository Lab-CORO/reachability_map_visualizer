#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include "reachability_map_visual.h"

#include "reachability_map_display.h"

namespace reachability_map_visualizer
{
ReachMapDisplay::ReachMapDisplay()
{
  do_display_arrow_ = new rviz_common::properties::BoolProperty("Show Poses", false, "Displays the arrow.", this);
  do_display_sphere_ = new rviz_common::properties::BoolProperty("Show Shape", true, "Displays the spheres.", this);
  is_byReachability_ = new rviz_common::properties::BoolProperty("Color by Reachability", true, "Color transform by Reachability Index", this);



  shape_property_ = new rviz_common::properties::EnumProperty("Shape", "Sphere", "Shape to display the workspace.", this,
                                           SLOT(updateColorAndAlphaArrow()));
  shape_property_->addOption("Sphere", Sphere);
  shape_property_->addOption("Cylinder", Cylinder);
  shape_property_->addOption("Cone", Cone);
  shape_property_->addOption("Cube", Cube);




  disect_category_  = new rviz_common::properties::Property("Discret", QVariant(), "", this);
  disect_property_ =
      new rviz_common::properties::EnumProperty("Axis", "None", "Disection of the workspace", disect_category_, SLOT(updateColorAndAlphaArrow()), this);
  disect_property_->addOption("None", Disect::None);
  disect_property_->addOption("X", Disect::X);
  disect_property_->addOption("Y", Disect::Y);
  disect_property_->addOption("Z", Disect::Z);

  disect_max_  = new rviz_common::properties::FloatProperty("Max", 10, "meter",
                                                  disect_category_, SLOT(updateColorAndAlphaArrow()), this);
  disect_min_  = new rviz_common::properties::FloatProperty("Min", -10, "meter",
                                                  disect_category_, SLOT(updateColorAndAlphaArrow()), this);



  // Arrow Property category
  arrow_category_ = new rviz_common::properties::Property("Poses Property", QVariant(), "", this);

  arrow_color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(204, 51, 204), "Color to draw the Pose arrows.",
                                                  arrow_category_, SLOT(updateColorAndAlphaArrow()), this);

  arrow_alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.2, "0 is fully transparent, 1.0 is fully opaque.",
                                                  arrow_category_, SLOT(updateColorAndAlphaArrow()), this);

  arrow_length_property_ =
      new rviz_common::properties::FloatProperty("Length", 0.01, "Length of the arrows", arrow_category_, SLOT(updateArrowSize()), this);

  // Shape Property category

  sphere_category_ = new rviz_common::properties::Property("Shape Property", QVariant(), "", this);

  sphere_color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(255, 225, 102), "Color to draw the Sphere.",
                                                   sphere_category_, SLOT(updateColorAndAlphaSphere()), this);

  sphere_alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                                   sphere_category_, SLOT(updateColorAndAlphaSphere()), this);

  sphere_radius_property_ =
      new rviz_common::properties::FloatProperty("Size", 0.05, "Size of the sphere", sphere_category_, SLOT(updateSphereSize()), this);

  lower_bound_reachability_ = new rviz_common::properties::IntProperty("Lowest Reachability Index", 0, "Lowest Reachability index.", this);

  upper_bound_reachability_ =
      new rviz_common::properties::IntProperty("Highest Reachability Index", 100, "Highest Reachability index.", this);
}

void ReachMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

ReachMapDisplay::~ReachMapDisplay()
{
}
void ReachMapDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void ReachMapDisplay::updateColorAndAlphaArrow()
{
  float alpha = arrow_alpha_property_->getFloat();
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColorArrow(color.r, color.g, color.b, alpha);
  }
}

void ReachMapDisplay::updateArrowSize()
{
  float length = arrow_length_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setSizeArrow(length);
  }
}

void ReachMapDisplay::updateColorAndAlphaSphere()
{
  float alpha = sphere_alpha_property_->getFloat();
  Ogre::ColourValue color = sphere_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColorSphere(color.r, color.g, color.b, alpha);
  }
}

void ReachMapDisplay::updateSphereSize()
{
  float length = sphere_radius_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setSizeSphere(length);
  }
}

void ReachMapDisplay::processMessage(reachability_map_visualizer::msg::WorkSpace::ConstSharedPtr msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    // RCLCPP_INFO(this->get_logger(),"Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
    //           qPrintable(fixed_frame_));
    return;
  }
  std::shared_ptr< ReachMapVisual > visual;
  visuals_.clear();
  visual.reset(new ReachMapVisual(context_->getSceneManager(), scene_node_, context_));

  visual->setMessage(msg, do_display_arrow_->getBool(), do_display_sphere_->getBool(),
                     lower_bound_reachability_->getInt(), upper_bound_reachability_->getInt(),
                     disect_max_->getFloat(), disect_min_->getFloat(), disect_property_->getOptionInt());

  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float arrow_alpha = arrow_alpha_property_->getFloat();
  Ogre::ColourValue arrow_color = arrow_color_property_->getOgreColor();
  visual->setColorArrow(arrow_color.r, arrow_color.g, arrow_color.b, arrow_alpha);

  float sphere_alpha = sphere_alpha_property_->getFloat();
  Ogre::ColourValue sphere_color = sphere_color_property_->getOgreColor();
  if (is_byReachability_->getBool() == false)
  {
    visual->setColorSphere(sphere_color.r, sphere_color.g, sphere_color.b, sphere_alpha);
  }
  else
  {
    visual->setColorSpherebyRI(sphere_alpha);
  }

  visuals_.push_back(visual);
  //updateArrowSize();
  updateSphereSize();

}

}  // end namespace reachability_map_visualizer
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(reachability_map_visualizer::ReachMapDisplay, rviz_common::Display)