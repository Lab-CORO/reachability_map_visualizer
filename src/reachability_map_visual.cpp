

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

namespace reachability_map_visualizer
{



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

  // Initialisation optimisation grille fixe
  grid_initialized_ = false;

  // Initialiser la lookup table des couleurs
  initializeColorLUT();

  // arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ ));
}

ReachMapVisual::~ReachMapVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}



// Fonction helper inline pour calculer la couleur selon le RI
inline Ogre::ColourValue ReachMapVisual::getColorForRI(float ri) const
{
  if (ri >= 90.0f) {
    return Ogre::ColourValue(0.0f, 0.0f, 1.0f);  // Bleu
  } else if (ri >= 50.0f) {
    return Ogre::ColourValue(0.0f, 1.0f, 1.0f);  // Cyan
  } else if (ri >= 30.0f) {
    return Ogre::ColourValue(0.0f, 1.0f, 0.0f);  // Vert
  } else if (ri >= 5.0f) {
    return Ogre::ColourValue(1.0f, 1.0f, 0.0f);  // Jaune
  } else {
    return Ogre::ColourValue(1.0f, 0.0f, 0.0f);  // Rouge
  }
}

// Initialise la grille fixe avec toutes les positions des voxels
void ReachMapVisual::initializeFixedGrid(const std::shared_ptr<const reachability_map_visualizer::msg::WorkSpace>& msg)
{
  // Vérifier si la grille a changé de taille ou paramètres
  if (grid_initialized_ &&
      msg->size_x == cached_size_x_ &&
      msg->size_y == cached_size_y_ &&
      msg->size_z == cached_size_z_ &&
      msg->resolution == cached_resolution_ &&
      msg->origine.x == cached_origin_.x &&
      msg->origine.y == cached_origin_.y &&
      msg->origine.z == cached_origin_.z) {
    // Grille déjà initialisée avec les bons paramètres
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("ReachMapVisual"),
    "Initializing fixed grid: %dx%dx%d = %d voxels, resolution=%.4f",
    msg->size_x, msg->size_y, msg->size_z, msg->size_x * msg->size_y * msg->size_z, msg->resolution);

  // Sauvegarder les paramètres de la grille
  cached_size_x_ = msg->size_x;
  cached_size_y_ = msg->size_y;
  cached_size_z_ = msg->size_z;
  cached_resolution_ = msg->resolution;
  cached_origin_ = msg->origine;

  // Pré-allouer le buffer avec la taille complète de la grille
  size_t total_voxels = msg->size_x * msg->size_y * msg->size_z;
  point_buffer_.clear();
  point_buffer_.reserve(total_voxels);

  // Créer TOUTES les positions de la grille (complète)
  for (int x = 0; x < msg->size_x; ++x) {
    for (int y = 0; y < msg->size_y; ++y) {
      for (int z = 0; z < msg->size_z; ++z) {
        rviz_rendering::PointCloud::Point pt;
        pt.position.x = msg->origine.x + x * msg->resolution;
        pt.position.y = msg->origine.y + y * msg->resolution;
        pt.position.z = msg->origine.z + z * msg->resolution;
        pt.color = Ogre::ColourValue(1.0f, 0.0f, 0.0f, 0.0f);  // Alpha = 0 (invisible par défaut)
        point_buffer_.push_back(pt);
      }
    }
  }

  grid_initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ReachMapVisual"), "Grid initialization complete");
}

// Initialiser la lookup table des couleurs pour accès O(1)
void ReachMapVisual::initializeColorLUT() {
  for (int ri = 0; ri <= 100; ++ri) {
    if (ri >= 90) {
      color_lut_[ri] = Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f);  // Bleu
    } else if (ri >= 50) {
      color_lut_[ri] = Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f);  // Cyan
    } else if (ri >= 30) {
      color_lut_[ri] = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f);  // Vert
    } else if (ri >= 5) {
      color_lut_[ri] = Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f);  // Jaune
    } else {
      color_lut_[ri] = Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f);  // Rouge
    }
  }
}

// Version ultra-optimisée avec OpenMP et vectorisation - utilise ri_values array
void ReachMapVisual::updateGridColorsOptimized(const std::shared_ptr<const reachability_map_visualizer::msg::WorkSpace>& msg,
                                               int low_ri, int high_ri, int disect_max, int disect_min, int disect_choice)
{
  // Vérifier que la grille a été initialisée
  if (!grid_initialized_) {
    return;
  }

  // Vérifier que ri_values est présent et de la bonne taille
  size_t expected_size = msg->size_x * msg->size_y * msg->size_z;
  if (msg->ri_values.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ReachMapVisual"), "ri_values is empty, using legacy ws_spheres");
    // Fallback vers ws_spheres si ri_values n'est pas rempli
    updateGridColors(msg, low_ri, high_ri, disect_max, disect_min, disect_choice);
    return;
  }

  if (msg->ri_values.size() != expected_size) {
    RCLCPP_ERROR(rclcpp::get_logger("ReachMapVisual"),
      "ri_values size mismatch: got %zu, expected %zu", msg->ri_values.size(), expected_size);
    return;
  }

  // Dimensions de la grille
  const int size_x = msg->size_x;
  const int size_y = msg->size_y;
  const int size_z = msg->size_z;

  // Pointeur direct vers les données
  const auto& ri_values = msg->ri_values;
  auto& points = point_buffer_;

  // Précalculer les limites de dissection pour les indices de grille
  int disect_min_idx = disect_min;
  int disect_max_idx = disect_max;
  const bool check_dissection = (disect_choice != Disect::None);

  // Boucle parallélisée avec OpenMP sur tous les voxels de la grille
  #ifdef _OPENMP
  #pragma omp parallel for schedule(static) if(expected_size > 1000)
  #endif
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        // Index flat: x * size_y * size_z + y * size_z + z
        size_t idx = x * size_y * size_z + y * size_z + z;

        float ri = ri_values[idx];
        auto& point = points[idx];

        // Filtrage par RI
        if (ri < low_ri || ri > high_ri) {
          point.color.a = 0.0f;  // Invisible
          continue;
        }

        // Filtrage par dissection (basé sur les indices de grille)
        if (check_dissection) {
          int index_to_check = 0;

          switch (disect_choice) {
            case Disect::X:
              index_to_check = x;
              break;
            case Disect::Y:
              index_to_check = y;
              break;
            case Disect::Z:
              index_to_check = z;
              break;
            default:
              break;
          }

          if (index_to_check < disect_min_idx || index_to_check > disect_max_idx) {
            point.color.a = 0.0f;  // Invisible
            continue;
          }
        }

        // Voxel visible : lookup table pour la couleur
        const int ri_index = static_cast<int>(ri);
        const int clamped_ri = (ri_index < 0) ? 0 : (ri_index > 100 ? 100 : ri_index);

        const auto& color = color_lut_[clamped_ri];
        point.color = color;  // Copie vectorisée par le compilateur
      }
    }
  }
}

// Met à jour uniquement les couleurs et alpha selon les filtres
void ReachMapVisual::updateGridColors(const std::shared_ptr<const reachability_map_visualizer::msg::WorkSpace>& msg,
                                      int low_ri, int high_ri, int disect_max, int disect_min, int disect_choice)
{
  // Vérifier que la grille a été initialisée et que les tailles correspondent
  if (!grid_initialized_ || msg->ws_spheres.size() != point_buffer_.size()) {
    return;
  }

  // Précalculer les limites de dissection pour éviter les calculs répétés
  float hight_min = 0.0f, hight_max = 0.0f;
  bool check_dissection = (disect_choice != Disect::None);

  if (check_dissection) {
    switch (disect_choice) {
      case Disect::X:
        hight_min = disect_min * msg->resolution - msg->origine.x;
        hight_max = disect_max * msg->resolution - msg->origine.x;
        break;
      case Disect::Y:
        hight_min = disect_min * msg->resolution - msg->origine.y;
        hight_max = disect_max * msg->resolution - msg->origine.y;
        break;
      case Disect::Z:
        hight_min = disect_min * msg->resolution - msg->origine.z;
        hight_max = disect_max * msg->resolution - msg->origine.z;
        break;
      default:
        check_dissection = false;
        break;
    }
  }

  // Parcourir tous les voxels et mettre à jour couleur + alpha
  for (size_t i = 0; i < msg->ws_spheres.size(); ++i) {
    const auto& voxel = msg->ws_spheres[i];
    auto& point = point_buffer_[i];

    // Filtrage par RI
    if (voxel.ri < low_ri || voxel.ri > high_ri) {
      point.color.a = 0.0f;  // Invisible
      continue;
    }

    // Filtrage par dissection
    if (check_dissection) {
      float value_to_check = 0.0f;
      switch (disect_choice) {
        case Disect::X:
          value_to_check = voxel.point.x;
          break;
        case Disect::Y:
          value_to_check = voxel.point.y;
          break;
        case Disect::Z:
          value_to_check = voxel.point.z;
          break;
        default:
          break;
      }

      if (value_to_check < hight_min || value_to_check > hight_max) {
        point.color.a = 0.0f;  // Invisible
        continue;
      }
    }

    // Voxel visible : mettre à jour la couleur selon RI et alpha = 1
    Ogre::ColourValue color = getColorForRI(voxel.ri);
    point.color.r = color.r;
    point.color.g = color.g;
    point.color.b = color.b;
    point.color.a = 1.0f;  // Visible
  }
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

    sensor_msgs::msg::PointField field_rgb;
    field_rgb.name = "rgb";
    field_rgb.offset = 12; // 'x' usually comes first
    field_rgb.datatype = sensor_msgs::msg::PointField::UINT32;
    field_rgb.count = 1;

  cloud.fields = {field_x, field_y, field_z, field_rgb};
  cloud.point_step = sizeof(float) * 3  + sizeof(uint32_t); // Include RGB

  // Fill point cloud data
  std::vector<uint8_t> data(cloud.width * cloud.point_step);
  uint8_t* ptr = data.data();
  for (size_t i = 0; i < points.ws_spheres.size(); ++i) {
    auto& point = points.ws_spheres[i];
    *(reinterpret_cast<float*>(ptr)) = point.point.x;
    *(reinterpret_cast<float*>(ptr + 4)) = point.point.y;
    *(reinterpret_cast<float*>(ptr + 8)) = point.point.z;

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

    *(reinterpret_cast<uint8_t*>(ptr + 12)) = r;
    *(reinterpret_cast<uint8_t*>(ptr + 13)) = g;
    *(reinterpret_cast<uint8_t*>(ptr + 14)) = b;

    ptr += cloud.point_step;
  }
  cloud.data = std::move(data);
}


void ReachMapVisual::setMessage(const std::shared_ptr<const reachability_map_visualizer::msg::WorkSpace>& msg, bool do_display_arrow, bool do_display_sphere,
                  int low_ri, int high_ri, int disect_max_, int disect_min_, int disect_choice)
{
  // Étape 1 : Initialiser la grille fixe au premier message ou si les paramètres changent
  // Vérifier avec size_x/y/z au lieu de ws_spheres.size()
  size_t expected_size = msg->size_x * msg->size_y * msg->size_z;
  if (!grid_initialized_ || point_buffer_.size() != expected_size) {
    initializeFixedGrid(msg);
  }

  // Étape 2 : Mettre à jour uniquement les couleurs et alpha selon les filtres
  // Utiliser la version optimisée avec OpenMP + vectorisation + ri_values array
  updateGridColorsOptimized(msg, low_ri, high_ri, disect_max_, disect_min_, disect_choice);

  // Étape 3 : Envoyer le buffer au GPU (clear + addPoints, mais le buffer est réutilisé)
  point_cloud_visual_->clear();
  point_cloud_visual_->setDimensions(msg->resolution, msg->resolution, msg->resolution);
  point_cloud_visual_->addPoints(point_buffer_.begin(), point_buffer_.end());
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
