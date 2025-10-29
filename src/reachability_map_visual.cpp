

// #define OGRE_VECTOR2_EXTENSIONS
// #define OGRE_VECTOR3_EXTENSIONS

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <rviz_common/display_context.hpp>
// #include <rviz_common/display_factory.hpp>
#include <rviz_common/factory/factory.hpp>

// #include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <tf2/LinearMath/Quaternion.h>

#include "reachability_map_visual.h"
#include <iterator>
#include <algorithm>

namespace reachability_map_visualizer
{

// Helper function to convert intensity (0-1) to inverted jet colormap (like MATLAB but reversed)
// Red (low) -> Yellow -> Green -> Cyan -> Blue (high)
Ogre::ColourValue intensityToJetColor(float intensity) {
  // Clamp intensity to [0, 1]
  intensity = std::max(0.0f, std::min(1.0f, intensity));

  // Invert intensity to reverse the color scale
  intensity = 1.0f - intensity;

  float r, g, b;

  if (intensity < 0.25f) {
    // Blue to Cyan: B=1, R=0, G increases from 0 to 1
    r = 0.0f;
    g = intensity / 0.25f;  // 0 -> 1
    b = 1.0f;
  } else if (intensity < 0.5f) {
    // Cyan to Green: G=1, B decreases from 1 to 0, R=0
    r = 0.0f;
    g = 1.0f;
    b = 1.0f - (intensity - 0.25f) / 0.25f;  // 1 -> 0
  } else if (intensity < 0.75f) {
    // Green to Yellow: G=1, R increases from 0 to 1, B=0
    r = (intensity - 0.5f) / 0.25f;  // 0 -> 1
    g = 1.0f;
    b = 0.0f;
  } else {
    // Yellow to Red: R=1, G decreases from 1 to 0, B=0
    r = 1.0f;
    g = 1.0f - (intensity - 0.75f) / 0.25f;  // 1 -> 0
    b = 0.0f;
  }

  return Ogre::ColourValue(r, g, b);
}




  ReachMapVisual::ReachMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                               rviz_common::DisplayContext* display_context)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  point_cloud_visual_ = new rviz_rendering::PointCloud();
  point_cloud_visual_->setRenderMode(rviz_rendering::PointCloud::RM_SPHERES);
  point_cloud_visual_->setAlpha(1.0f);
  point_cloud_visual_->setDimensions(0.02f, 0.02f, 0.02f);

  frame_node_->attachObject(point_cloud_visual_);
  // arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ ));
}

ReachMapVisual::~ReachMapVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}



void ReachMapVisual::convertPointsToPointCloud(const reachability_map_visualizer::msg::WorkSpace& points, 
                                                sensor_msgs::msg::PointCloud2& cloud, 
                                                const std::string& frame_id) {

  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = points.ws_spheres.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // Define point fields
  sensor_msgs::msg::PointField field_x;
    // Set the field name (e.g., "x", "y", "z", or "intensity")
    field_x.name = "x";
    // Set the offset in bytes from the start of the point structure
    field_x.offset = 0; // 'x' usually comes first
    // Set the datatype (one of the constants from PointField)
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    // Set the number of elements in the field
    field_x.count = 1;

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4; 
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    sensor_msgs::msg::PointField field_intensity;
    field_intensity.name = "intensity";
    field_intensity.offset = 12;
    field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_intensity.count = 1;

    sensor_msgs::msg::PointField field_rgb;
    field_rgb.name = "rgb";
    field_rgb.offset = 16;
    field_rgb.datatype = sensor_msgs::msg::PointField::UINT32;
    field_rgb.count = 1;

  cloud.fields = {field_x, field_y, field_z, field_intensity, field_rgb};
  cloud.point_step = sizeof(float) * 4  + sizeof(uint32_t); // Include intensity and RGB

  // Fill point cloud data
  std::vector<uint8_t> data(cloud.width * cloud.point_step);
  uint8_t* ptr = data.data();
  for (size_t i = 0; i < points.ws_spheres.size(); ++i) {
    auto& point = points.ws_spheres[i];
    *(reinterpret_cast<float*>(ptr)) = point.point.x;
    *(reinterpret_cast<float*>(ptr + 4)) = point.point.y;
    *(reinterpret_cast<float*>(ptr + 8)) = point.point.z;
    *(reinterpret_cast<float*>(ptr + 12)) = point.ri; // Set intensity value

    // Set color based on intensity
    uint8_t r, g, b;
    if (point.ri *100 >= 90) {
      r = 0; g = 0; b = 255;
    } else if (point.ri*100 < 90 && point.ri*100 >= 50) {
      r = 0; g = 255; b = 255;
    } else if (point.ri*100 < 50 && point.ri*100 >= 30) {
      r = 0; g = 255; b = 0;
    } else if (point.ri *100 < 30 && point.ri*100 >= 5) {
      r = 255; g = 255; b = 0;
    } else {
      r = 255; g = 0; b = 0;
    }

    *(reinterpret_cast<uint8_t*>(ptr + 16)) = r;
    *(reinterpret_cast<uint8_t*>(ptr + 17)) = g;
    *(reinterpret_cast<uint8_t*>(ptr + 18)) = b;

    ptr += cloud.point_step;
  }
  cloud.data = std::move(data);
}


void ReachMapVisual::setMessage(const reachability_map_visualizer::msg::WorkSpace::ConstPtr& msg, bool do_display_arrow, bool do_display_sphere,
                  int low_ri, int high_ri, int disect_max_, int disect_min_, int disect_choice, bool use_intensity_coloring)
{


  point_cloud_visual_->clear();
  point_cloud_visual_->setDimensions(msg->resolution, msg->resolution, msg->resolution);

  // Configure coloring based on mode
  if (use_intensity_coloring) {
    // Use intensity channel - RViz will apply color map (like jet)
    // This allows users to choose color scheme in RViz (Flat Color, Intensity, etc.)
  } else {
    // Use RGB coloring - direct color specification
  }

  std::vector<rviz_rendering::PointCloud::Point> points;
  points.reserve(msg->ws_spheres.size()); // assuming your message has a vector called 'points'

  for (const auto& point : msg->ws_spheres)
  {
    if (point.ri  < low_ri || point.ri  > high_ri){
      continue;
    }

  
    if (disect_choice == Disect::X){
      // convert index to position
      float hight_min = disect_min_ * msg->resolution - msg->origine.x;
      float hight_max = disect_max_ * msg->resolution - msg->origine.x;

      if (point.point.x < hight_min || point.point.x > hight_max){
        continue;
      }
    }

    if (disect_choice == Disect::Y){
      // convert index to position
      float hight_min = disect_min_ * msg->resolution - msg->origine.y;
      float hight_max = disect_max_ * msg->resolution - msg->origine.y;

      if (point.point.y < hight_min || point.point.y > hight_max){
        continue;
      }
    }

    if (disect_choice == Disect::Z){
      // convert index to position
      float hight_min = disect_min_ * msg->resolution - msg->origine.z;
      float hight_max = disect_max_ * msg->resolution - msg->origine.z;

      if (point.point.z < hight_min || point.point.z > hight_max){
        continue;
      }
    }

      rviz_rendering::PointCloud::Point pc;
      pc.position = Ogre::Vector3(point.point.x, point.point.y, point.point.z);

      if (use_intensity_coloring) {
        // Use jet colormap (blue -> cyan -> green -> yellow -> red)
        // Normalize to 0-1 range if point.ri is 0-100
        float intensity = point.ri / 100.0f;
        pc.color = intensityToJetColor(intensity);
      } else {
        // Use categorical RGB coloring based on reachability thresholds
        uint8_t r, g, b;
        if (point.ri >= 90) {
          r = 0; g = 255; b = 0;
        } else if (point.ri < 90 && point.ri >= 50) {
          r = 0; g = 255; b = 255;
        } else if (point.ri < 50 && point.ri >= 30) {
          r = 0; g = 0; b = 255;
        } else if (point.ri < 30 && point.ri >= 5) {
          r = 255; g = 0; b = 255;
        } else {
          r = 255; g = 0; b = 0;
        }
        pc.color = Ogre::ColourValue(r/255.0f, g/255.0f, b/255.0f);
      }

      points.push_back(pc);
  }

  point_cloud_visual_->addPoints(points.begin(), points.end());

}

void ReachMapVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ReachMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void ReachMapVisual::setColorArrow(float r, float g, float b, float a)
{
  for (int i = 0; i < arrow_.size(); ++i)
  {
    arrow_[i]->setColor(r, g, b, a);
  }
}

void ReachMapVisual::setColorSphere(float r, float g, float b, float a)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setColor(r, g, b, a);
  }
}

void ReachMapVisual::setColorSpherebyRI(float alpha)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    if (colorRI_[i] >= 90)
    {
      sphere_[i]->setColor(0, 0, 255, alpha);
    }
    else if (colorRI_[i] < 90 && colorRI_[i] >= 50)
    {
      sphere_[i]->setColor(0, 255, 255, alpha);
    }
    else if (colorRI_[i] < 50 && colorRI_[i] >= 30)
    {
      sphere_[i]->setColor(0, 255, 0, alpha);
    }
    else if (colorRI_[i] < 30 && colorRI_[i] >= 5)
    {
      sphere_[i]->setColor(255, 255, 0, alpha);
    }
    else
    {
      sphere_[i]->setColor(255, 0, 0, alpha);
    }
  }
}

void ReachMapVisual::setSizeArrow(float l)
{
  for (int i = 0; i < arrow_.size(); ++i)
  {
    arrow_[i]->setScale(Ogre::Vector3(l, l, l));
  }
}

void ReachMapVisual::setSizeSphere(float l)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setScale(Ogre::Vector3(l, l, l));
  }
}

}  // end namespace reachability_map_visualizer
