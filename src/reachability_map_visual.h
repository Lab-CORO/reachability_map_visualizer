#ifndef ReachMap_VISUAL_H
#define ReachMap_VISUAL_H

#include "reachability_map_visualizer/msg/work_space.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

// namespace Ogre
// {
// class Vector3;
// class Quaternion;
// }

namespace rviz_rendering
{
class Arrow;
class Shape;
}
enum Disect
{
  None,
  X,
  Y,
  Z,
  // Middle_Slice,
  // End_Slice,
};
namespace reachability_map_visualizer
{
class ReachMapDisplay;
class ReachMapVisual
{
public:
  ReachMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz_common::DisplayContext* display);
  virtual ~ReachMapVisual();
  void setMessage(const reachability_map_visualizer::msg::WorkSpace::ConstPtr& msg, bool do_display_arrow, bool do_display_sphere,
                  int low_ri, int high_ri, float disect_max_, float disect_min_, int disect_choice);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColorArrow(float r, float g, float b, float a);
  void setSizeArrow(float l);

  void setColorSphere(float r, float g, float b, float a);
  void setSizeSphere(float l);
  void setColorSpherebyRI(float alpha);
  void convertPointsToPointCloud(const reachability_map_visualizer::msg::WorkSpace& points, sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id);


private:
  std::vector< std::shared_ptr< rviz_rendering::Arrow > > arrow_;
  std::vector< std::shared_ptr< rviz_rendering::Shape > > sphere_;
  std::vector< int > colorRI_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;

  // save map paramters
  float resolution;

  int size_x; // The number of voxel on the axis X
  int size_y ;
  int size_z;

  geometry_msgs::msg::Point origine;

  rviz_rendering::PointCloud* point_cloud_visual_;

};
}  // end namespace reachability_map_visualizer
#endif  // ReachMap_VISUAL_H
