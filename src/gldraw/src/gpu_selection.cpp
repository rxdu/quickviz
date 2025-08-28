/**
 * @file gpu_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-28
 * @brief Implementation of GPU ID-buffer selection system
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/gpu_selection.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/point_cloud.hpp"

#include <iostream>
#include <algorithm>

namespace quickviz {

GPUSelection::GPUSelection(GlSceneManager* scene_manager)
    : scene_manager_(scene_manager) {
  if (!scene_manager_) {
    throw std::invalid_argument("Scene manager cannot be null");
  }
}

GPUSelectionResult GPUSelection::SelectAt(int pixel_x, int pixel_y, int radius) {
  if (!scene_manager_) {
    return GPUSelectionResult::None();
  }
  
  // Render the scene to ID buffer with unique colors for each object/point
  this->RenderIdBuffer();
  
  uint32_t selected_id;
  if (radius > 0) {
    // Try selection with radius tolerance - check multiple pixels
    selected_id = kBackgroundId;
    for (int dy = -radius; dy <= radius && selected_id == kBackgroundId; ++dy) {
      for (int dx = -radius; dx <= radius && selected_id == kBackgroundId; ++dx) {
        if (dx*dx + dy*dy <= radius*radius) { // Circle, not square
          uint32_t test_id = this->ReadPixelId(pixel_x + dx, pixel_y + dy);
          if (test_id != kBackgroundId) {
            selected_id = test_id;
          }
        }
      }
    }
  } else {
    // Exact pixel selection
    selected_id = this->ReadPixelId(pixel_x, pixel_y);
  }
  
  // Process the selection
  GPUSelectionResult result = ProcessSelection(selected_id, pixel_x, pixel_y);
  ApplySelection(result);
  
  return result;
}

GPUSelectionResult GPUSelection::SelectAtScreen(float screen_x, float screen_y,
                                               float screen_width, float screen_height,
                                               int radius) {
  // Convert screen coordinates to OpenGL pixel coordinates
  int pixel_x = static_cast<int>(screen_x);
  int pixel_y = static_cast<int>(screen_height - screen_y); // Flip Y for OpenGL
  
  return SelectAt(pixel_x, pixel_y, radius);
}

GPUSelectionResult GPUSelection::ProcessSelection(uint32_t id, int pixel_x, int pixel_y) {
  if (id == kBackgroundId) {
    return GPUSelectionResult::None();
  }
  
  // Determine if this is a point or object ID
  if (id >= kPointIdBase && id <= kMaxPointId) {
    return FindPointById(id, pixel_x, pixel_y);
  } else if (id >= kObjectIdBase && id <= kMaxObjectId) {
    // Object selection not yet implemented - requires full ID buffer rendering
    std::cout << "Object selection detected but not yet implemented (ID: 0x" 
              << std::hex << id << std::dec << ")" << std::endl;
    return GPUSelectionResult::None();
  }
  
  return GPUSelectionResult::None();
}

GPUSelectionResult GPUSelection::FindPointById(uint32_t point_id, int pixel_x, int pixel_y) {
  // Decode point index
  size_t point_index = point_id - kPointIdBase;
  
  // Find which point cloud this point belongs to
  // For now, use the active point cloud system from GlSceneManager
  PointCloud* active_cloud = scene_manager_->GetActivePointCloud();
  if (!active_cloud || point_index >= active_cloud->GetPointCount()) {
    return GPUSelectionResult::None();
  }
  
  // Get point position
  glm::vec3 point_pos = active_cloud->GetPointPosition(point_index);
  glm::vec2 screen_pos(static_cast<float>(pixel_x), static_cast<float>(pixel_y));
  
  // Find the name of this point cloud
  std::string cloud_name = "active_point_cloud";
  for (const auto& [name, object] : registered_objects_) {
    if (object == active_cloud) {
      cloud_name = name;
      break;
    }
  }
  
  return GPUSelectionResult::Point(cloud_name, active_cloud, point_index, point_pos, screen_pos);
}

GPUSelectionResult GPUSelection::FindObjectById(uint32_t object_id, int pixel_x, int pixel_y) {
  // Find object by ID
  std::string object_name;
  for (const auto& [name, id] : object_id_map_) {
    if (id == object_id) {
      object_name = name;
      break;
    }
  }
  
  if (object_name.empty()) {
    return GPUSelectionResult::None();
  }
  
  auto it = registered_objects_.find(object_name);
  if (it == registered_objects_.end()) {
    return GPUSelectionResult::None();
  }
  
  OpenGlObject* object = it->second;
  
  // Get object's world position (use centroid or center)
  glm::vec3 world_pos{0.0f};
  auto bounds = object->GetBoundingBox();
  if (bounds.first != bounds.second) {
    world_pos = (bounds.first + bounds.second) * 0.5f; // Center of bounding box
  }
  
  glm::vec2 screen_pos(static_cast<float>(pixel_x), static_cast<float>(pixel_y));
  
  return GPUSelectionResult::Object(object_name, object, world_pos, screen_pos);
}

void GPUSelection::ClearSelection() {
  ClearVisualFeedback();
  current_selection_ = GPUSelectionResult::None();
  
  if (callback_) {
    callback_(current_selection_);
  }
}

void GPUSelection::RegisterObject(const std::string& name, OpenGlObject* object) {
  if (!object) return;
  
  registered_objects_[name] = object;
  
  // Assign unique ID for objects that support selection
  if (object->SupportsSelection() && !object->SupportsPointPicking()) {
    object_id_map_[name] = next_object_id_++;
    
    // Ensure we don't exceed the ID space
    if (next_object_id_ > kMaxObjectId) {
      std::cerr << "Warning: Object ID space exhausted!" << std::endl;
    }
  }
}

void GPUSelection::UnregisterObject(const std::string& name) {
  // Clear selection if this object is currently selected
  if (current_selection_.IsValid() && current_selection_.name == name) {
    ClearSelection();
  }
  
  registered_objects_.erase(name);
  object_id_map_.erase(name);
}

void GPUSelection::ApplySelection(const GPUSelectionResult& result) {
  // Clear previous selection feedback
  ClearVisualFeedback();
  
  // Store new selection
  current_selection_ = result;
  
  if (result.IsValid()) {
    if (result.IsObject()) {
      // Highlight the selected object
      if (result.object) {
        result.object->SetHighlighted(true);
        highlighted_object_name_ = result.name;
      }
    } else if (result.IsPoint()) {
      // Highlight the selected point
      if (result.point_cloud) {
        result.point_cloud->HighlightPoint(result.point_index, 
                                          glm::vec3(1.0f, 1.0f, 0.0f),  // Yellow highlight
                                          "gpu_selection",
                                          1.5f);
        highlighted_point_cloud_name_ = result.name;
        highlighted_point_index_ = result.point_index;
      }
    }
  }
  
  // Notify callback
  if (callback_) {
    callback_(current_selection_);
  }
}

void GPUSelection::ClearVisualFeedback() {
  // Clear object highlighting
  if (!highlighted_object_name_.empty()) {
    auto it = registered_objects_.find(highlighted_object_name_);
    if (it != registered_objects_.end()) {
      it->second->SetHighlighted(false);
    }
    highlighted_object_name_.clear();
  }
  
  // Clear point highlighting
  if (!highlighted_point_cloud_name_.empty() && highlighted_point_index_ != SIZE_MAX) {
    auto it = registered_objects_.find(highlighted_point_cloud_name_);
    if (it != registered_objects_.end()) {
      PointCloud* point_cloud = dynamic_cast<PointCloud*>(it->second);
      if (point_cloud) {
        point_cloud->ClearHighlights("gpu_selection");
      }
    }
    highlighted_point_cloud_name_.clear();
    highlighted_point_index_ = SIZE_MAX;
  }
}

// ID encoding/decoding methods
uint32_t GPUSelection::EncodeObjectId(const std::string& object_name) {
  auto it = object_id_map_.find(object_name);
  return (it != object_id_map_.end()) ? it->second : kBackgroundId;
}

uint32_t GPUSelection::EncodePointId(size_t point_index) {
  if (point_index > (kMaxPointId - kPointIdBase)) {
    return kBackgroundId; // Index too large
  }
  return kPointIdBase + static_cast<uint32_t>(point_index);
}

glm::vec3 GPUSelection::IdToColor(uint32_t id) {
  // Convert 24-bit ID to RGB color
  float r = static_cast<float>((id >> 0) & 0xFF) / 255.0f;
  float g = static_cast<float>((id >> 8) & 0xFF) / 255.0f;
  float b = static_cast<float>((id >> 16) & 0xFF) / 255.0f;
  return glm::vec3(r, g, b);
}

uint32_t GPUSelection::ColorToId(const glm::vec3& color) {
  uint8_t r = static_cast<uint8_t>(color.r * 255.0f + 0.5f);
  uint8_t g = static_cast<uint8_t>(color.g * 255.0f + 0.5f);
  uint8_t b = static_cast<uint8_t>(color.b * 255.0f + 0.5f);
  return ColorToId(r, g, b);
}

uint32_t GPUSelection::ColorToId(uint8_t r, uint8_t g, uint8_t b) {
  return (uint32_t(r) << 0) | (uint32_t(g) << 8) | (uint32_t(b) << 16);
}

void GPUSelection::RenderIdBuffer() {
  // Delegate to scene manager - it now handles both points and objects
  // The scene manager's RenderIdBuffer has been extended to render registered objects
  // No need to do anything here - the infrastructure is handled by the scene manager
}

uint32_t GPUSelection::ReadPixelId(int x, int y) {
  // We need to read the actual pixel color from the ID buffer, not use PickPointAtPixel
  // which only returns point cloud indices. For a complete implementation, we need:
  // 1. Access to the ID framebuffer
  // 2. Read the RGB color at (x,y) 
  // 3. Decode the color back to an ID using ColorToId()
  
  // For now, use the existing point cloud infrastructure as a fallback
  size_t point_id = scene_manager_->PickPointAtPixel(x, y);
  
  if (point_id == SIZE_MAX) {
    return kBackgroundId;
  }
  
  // Convert point index to encoded point ID
  return EncodePointId(point_id);
}

}  // namespace quickviz