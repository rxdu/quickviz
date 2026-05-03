/**
 * @file selection_manager.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-28
 * @brief Implementation of interactive selection system for 3D scenes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/selection_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <glad/glad.h>
#include "scene/scene_manager.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "scene/frame_buffer.hpp"

namespace quickviz {

// === MultiSelection Implementation ===

void MultiSelection::Add(const SelectionResult& selection) {
  if (!IsEmpty(selection)) {
    // Check if already exists
    auto it = std::find(selections_.begin(), selections_.end(), selection);
    if (it == selections_.end()) {
      selections_.push_back(selection);
    }
  }
}

void MultiSelection::Remove(const SelectionResult& selection) {
  auto it = std::find(selections_.begin(), selections_.end(), selection);
  if (it != selections_.end()) {
    selections_.erase(it);
  }
}

void MultiSelection::Toggle(const SelectionResult& selection) {
  auto it = std::find(selections_.begin(), selections_.end(), selection);
  if (it != selections_.end()) {
    selections_.erase(it);
  } else {
    Add(selection);
  }
}

glm::vec3 MultiSelection::GetCentroid() const {
  if (selections_.empty()) {
    return glm::vec3(0.0f);
  }
  
  glm::vec3 centroid(0.0f);
  for (const auto& selection : selections_) {
    centroid += GetSelectionWorldPosition(selection);
  }
  return centroid / static_cast<float>(selections_.size());
}

std::pair<glm::vec3, glm::vec3> MultiSelection::GetBounds() const {
  if (selections_.empty()) {
    return {glm::vec3(0.0f), glm::vec3(0.0f)};
  }
  
  glm::vec3 min_bounds = GetSelectionWorldPosition(selections_[0]);
  glm::vec3 max_bounds = min_bounds;
  
  for (size_t i = 1; i < selections_.size(); ++i) {
    glm::vec3 pos = GetSelectionWorldPosition(selections_[i]);
    min_bounds = glm::min(min_bounds, pos);
    max_bounds = glm::max(max_bounds, pos);
  }
  
  return {min_bounds, max_bounds};
}

std::vector<PointSelection> MultiSelection::GetPoints() const {
  std::vector<PointSelection> points;
  for (const auto& selection : selections_) {
    if (std::holds_alternative<PointSelection>(selection)) {
      points.push_back(std::get<PointSelection>(selection));
    }
  }
  return points;
}

std::vector<ObjectSelection> MultiSelection::GetObjects() const {
  std::vector<ObjectSelection> objects;
  for (const auto& selection : selections_) {
    if (std::holds_alternative<ObjectSelection>(selection)) {
      objects.push_back(std::get<ObjectSelection>(selection));
    }
  }
  return objects;
}

// === SelectionManager Implementation ===

SelectionManager::SelectionManager(SceneManager* scene_manager)
    : scene_manager_(scene_manager) {
  if (!scene_manager_) {
    throw std::invalid_argument("Scene manager cannot be null");
  }
}

SelectionResult SelectionManager::Select(float screen_x, float screen_y, const SelectionOptions& options) {
  // Clear previous single selection if not adding to multi-selection
  if (!options.add_to_selection) {
    ClearSelection();
  }
  
  // Render ID buffer for current frame
  RenderIdBuffer();
  
  // Convert screen coordinates to pixel coordinates  
  // CRITICAL FIX: The screen coordinates are relative to ImGui content region,
  // but ID buffer matches the main framebuffer size. We need to ensure both use same size.
  
  if (!id_frame_buffer_ || !scene_manager_->frame_buffer_) {
    return SelectionResult{}; // Can't select without buffers
  }
  
  float id_buffer_width = id_frame_buffer_->GetWidth();
  float id_buffer_height = id_frame_buffer_->GetHeight();
  float main_buffer_width = scene_manager_->frame_buffer_->GetWidth();
  float main_buffer_height = scene_manager_->frame_buffer_->GetHeight();
  
  // Screen coordinates should be relative to the same space as the rendered content
  // If ID buffer matches main buffer, use direct mapping
  int pixel_x = static_cast<int>(std::round(screen_x));
  int pixel_y = static_cast<int>(std::round(screen_y));

  // Read pixel ID (with radius tolerance if specified)
  uint32_t selected_id = 0;
  if (options.radius <= 0) {
    selected_id = ReadPixelId(pixel_x, pixel_y);
  } else {
    // TODO: Implement radius-based selection
    // For now, just use center pixel
    selected_id = ReadPixelId(pixel_x, pixel_y);
  }

  if (selected_id == kBackgroundId) {
    return SelectionResult{}; // No selection
  }
  
  // Process the selection based on ID
  SelectionResult result = ProcessSingleSelection(selected_id, screen_x, screen_y);
  
  // Apply filter if provided
  if (options.filter && !IsEmpty(result)) {
    if (!options.filter(result)) {
      return SelectionResult{}; // Filtered out
    }
  }
  
  // Update selection state
  UpdateSelectionState(result, options.add_to_selection);
  
  return result;
}

SelectionResult SelectionManager::QuerySelection(float screen_x, float screen_y, const SelectionOptions& options) {
  // Same logic as Select() but without modifying selection state
  
  // Validate input coordinates
  if (screen_x < 0 || screen_y < 0) {
    return SelectionResult{}; // Invalid coordinates
  }
  
  // We can only select if ID buffer exists and matches scene buffer size
  // The reason is that selection is done in screen space, and the mouse coordinates 
  // are relative to the main framebuffer, not the ID buffer. But for performance,
  // ID buffer might be smaller than the main framebuffer. This can cause mismatch
  // between mouse coordinates and ID buffer pixel coordinates. Currently we assume
  // ID buffer matches main framebuffer size.
  //
  // TODO: If ID buffer != main buffer size, need coordinate transformation or scaling.
  // For now we'll check size match and fail if they don't match, because the logic
  // gets much more complex when accounting for different resolutions. We can assert
  // but ID buffer matches the main framebuffer size. We need to ensure both use same size.
  
  if (!id_frame_buffer_ || !scene_manager_->frame_buffer_) {
    return SelectionResult{}; // Can't select without buffers
  }
  
  float id_buffer_width = id_frame_buffer_->GetWidth();
  float id_buffer_height = id_frame_buffer_->GetHeight();
  float main_buffer_width = scene_manager_->frame_buffer_->GetWidth();
  float main_buffer_height = scene_manager_->frame_buffer_->GetHeight();
  
  // Screen coordinates should be relative to the same space as the rendered content
  // If ID buffer matches main buffer, use direct mapping
  int pixel_x = static_cast<int>(std::round(screen_x));
  int pixel_y = static_cast<int>(std::round(screen_y));

  // Read pixel ID (with radius tolerance if specified)
  uint32_t selected_id = 0;
  if (options.radius <= 0) {
    selected_id = ReadPixelId(pixel_x, pixel_y);
  } else {
    // TODO: Implement radius-based selection
    // For now, just use center pixel
    selected_id = ReadPixelId(pixel_x, pixel_y);
  }

  if (selected_id == kBackgroundId) {
    return SelectionResult{}; // No selection
  }
  
  // Process the selection based on ID
  SelectionResult result = ProcessSingleSelection(selected_id, screen_x, screen_y);
  
  // Apply filter if provided
  if (options.filter && !IsEmpty(result)) {
    if (!options.filter(result)) {
      return SelectionResult{}; // Filtered out
    }
  }
  
  // DO NOT call UpdateSelectionState - this is query only
  return result;
}

SelectionResult SelectionManager::SelectPoint(float screen_x, float screen_y, int radius) {
  SelectionOptions options;
  options.radius = radius;
  options.mode = SelectionMode::kPoints;
  return Select(screen_x, screen_y, options);
}

SelectionResult SelectionManager::SelectObject(float screen_x, float screen_y) {
  SelectionOptions options;
  options.mode = SelectionMode::kObjects;
  return Select(screen_x, screen_y, options);
}

bool SelectionManager::AddToSelection(float screen_x, float screen_y, const SelectionOptions& options) {
  SelectionOptions modified_options = options;
  modified_options.add_to_selection = true;
  
  SelectionResult result = Select(screen_x, screen_y, modified_options);
  return !IsEmpty(result);
}

bool SelectionManager::ToggleSelection(float screen_x, float screen_y, const SelectionOptions& options) {
  // First, find what would be selected at this location
  SelectionResult candidate = Select(screen_x, screen_y, options);
  if (IsEmpty(candidate)) {
    return false;
  }
  
  // Check if it's already in the multi-selection
  multi_selection_.Toggle(candidate);
  current_selection_ = candidate;
  
  NotifySelectionChanged();
  return true;
}

size_t SelectionManager::SelectRegion(const SelectionRectangle& rectangle, const SelectionOptions& options) {
  // TODO: Implement region selection by reading ID buffer pixels within rectangle
  // For now, return 0 (not implemented)
  return 0;
}

void SelectionManager::ClearSelection() {
  current_selection_ = SelectionResult{};
  multi_selection_.Clear();
  ClearVisualFeedback();
  NotifySelectionChanged();
}

bool SelectionManager::HasSelection() const {
  return !IsEmpty(current_selection_) || !multi_selection_.Empty();
}

void SelectionManager::RegisterObject(const std::string& name, OpenGlObject* object) {
  if (!object) return;
  
  registered_objects_[name] = object;
  
  // Assign unique ID for all objects (including point clouds)
  if (object_to_id_.find(name) == object_to_id_.end()) {
    object_to_id_[name] = next_object_id_;
    next_object_id_ += 1;
    
    // Also track point clouds separately for point selection
    PointCloud* point_cloud = dynamic_cast<PointCloud*>(object);
    if (point_cloud) {
      registered_point_clouds_[name] = point_cloud;
      
      // Assign explicit global index range for this point cloud
      uint32_t point_count = point_cloud->GetPointCount();
      if (next_global_index_ + point_count < kMaxPointId) {
        PointCloudRange range;
        range.start_index = next_global_index_;
        range.end_index = next_global_index_ + point_count;
        range.point_cloud = point_cloud;
        
        point_cloud_ranges_.push_back(range);
        
        // Set the ID base on the point cloud so it uses the correct range
        point_cloud->SetObjectIdBase(next_global_index_);
        
        next_global_index_ += point_count;
        
      } else {
        std::cerr << "[SelectionManager] Warning: Point cloud '" << name 
                  << "' exceeds available ID space" << std::endl;
      }
    }
    
    // Prevent overflow
    if (next_object_id_ > kMaxObjectId) {
      next_object_id_ = kObjectIdBase;
    }
  }
}

void SelectionManager::UnregisterObject(const std::string& name) {
  registered_objects_.erase(name);
  
  // Remove point cloud and its range if it exists
  auto pc_it = registered_point_clouds_.find(name);
  if (pc_it != registered_point_clouds_.end()) {
    PointCloud* point_cloud = pc_it->second;
    registered_point_clouds_.erase(pc_it);
    
    // Remove from ranges (note: this is O(n), could optimize with map if needed)
    point_cloud_ranges_.erase(
      std::remove_if(point_cloud_ranges_.begin(), point_cloud_ranges_.end(),
                    [point_cloud](const PointCloudRange& range) {
                      return range.point_cloud == point_cloud;
                    }),
      point_cloud_ranges_.end()
    );
  }
  
  // Keep the ID mapping in case the object is re-added
  // object_to_id_.erase(name);
}

// === Private Implementation ===

uint32_t SelectionManager::EncodeObjectId(const std::string& object_name) {
  auto it = object_to_id_.find(object_name);
  if (it != object_to_id_.end()) {
    return it->second;
  }
  return kBackgroundId;
}

uint32_t SelectionManager::EncodePointId(size_t point_index) {
  return kPointIdBase + static_cast<uint32_t>(point_index);
}

glm::vec3 SelectionManager::IdToColor(uint32_t id) {
  return glm::vec3(
    static_cast<float>((id >> 0) & 0xFF) / 255.0f,
    static_cast<float>((id >> 8) & 0xFF) / 255.0f,
    static_cast<float>((id >> 16) & 0xFF) / 255.0f
  );
}

uint32_t SelectionManager::ColorToId(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint32_t>(r) << 0) | 
         (static_cast<uint32_t>(g) << 8) | 
         (static_cast<uint32_t>(b) << 16);
}

SelectionResult SelectionManager::ProcessSingleSelection(uint32_t id, float screen_x, float screen_y) {
  if (id >= kObjectIdBase && id <= kMaxObjectId) {
    return FindObjectById(id, screen_x, screen_y);
  } else if (id >= kPointIdBase && id <= kMaxPointId) {
    return FindPointById(id, screen_x, screen_y);
  }
  
  return SelectionResult{}; // No valid selection
}

SelectionResult SelectionManager::FindObjectById(uint32_t object_id, float screen_x, float screen_y) {
  // Find object name by ID
  for (const auto& [name, id] : object_to_id_) {
    if (id == object_id) {
      auto obj_it = registered_objects_.find(name);
      if (obj_it != registered_objects_.end()) {
        ObjectSelection obj_selection;
        obj_selection.object_name = name;
        obj_selection.object = obj_it->second;
        obj_selection.screen_position = glm::vec2(screen_x, screen_y);
        // TODO: Calculate actual world position from object bounds or click ray
        obj_selection.world_position = glm::vec3(0.0f);
        
        return obj_selection;
      }
      break;
    }
  }
  
  return SelectionResult{};
}

SelectionResult SelectionManager::FindPointById(uint32_t point_id, float screen_x, float screen_y) {
  // Simple approach: point_id is just the global index across all point clouds
  if (point_id < kPointIdBase || point_id >= kObjectIdBase) {
    return SelectionResult{};
  }
  
  // Find which cloud this point_id belongs to using explicit ranges
  for (const auto& range : point_cloud_ranges_) {
    if (point_id >= range.start_index && point_id < range.end_index) {
      // Found the right cloud!
      size_t local_index = point_id - range.start_index;
      
      // Find cloud name
      std::string cloud_name;
      for (const auto& [name, pc] : registered_point_clouds_) {
        if (pc == range.point_cloud) {
          cloud_name = name;
          break;
        }
      }
      
      
      PointSelection selection;
      selection.cloud_name = cloud_name;
      selection.point_cloud = range.point_cloud;
      selection.point_index = local_index;
      selection.screen_position = glm::vec2(screen_x, screen_y);
      
      // Get the actual point position
      const auto& points = range.point_cloud->GetPoints();
      if (local_index < points.size()) {
        selection.world_position = points[local_index];
      } else {
        selection.world_position = glm::vec3(0.0f);
      }
      
      return selection;
    }
  }
  
  // Point ID doesn't match any registered point cloud
  return SelectionResult{};
}

void SelectionManager::UpdateSelectionState(const SelectionResult& new_selection, bool add_to_multi) {
  if (IsEmpty(new_selection)) {
    return;
  }
  
  
  current_selection_ = new_selection;
  
  if (add_to_multi) {
    multi_selection_.Add(new_selection);
  } else {
    // Replace multi-selection with single selection
    multi_selection_.Clear();
    multi_selection_.Add(new_selection);
  }
  
  ApplySelectionFeedback();
  NotifySelectionChanged();
}

void SelectionManager::NotifySelectionChanged() {
  if (callback_) {
    callback_(current_selection_, multi_selection_);
  }
}

void SelectionManager::RenderIdBuffer() {
  if (!scene_manager_->frame_buffer_) return;  // Need main framebuffer to get dimensions
  float width = scene_manager_->frame_buffer_->GetWidth();
  float height = scene_manager_->frame_buffer_->GetHeight();

  // CRITICAL FIX: Recalculate projection and view matrices using same logic as main render
  // This ensures perfect synchronization between main render and ID buffer
  float aspect_ratio = width / height;
  glm::mat4 projection = scene_manager_->camera_->GetProjectionMatrix(aspect_ratio, scene_manager_->z_near_, scene_manager_->z_far_);
  glm::mat4 view = scene_manager_->camera_->GetViewMatrix();
  
  // Create or resize ID framebuffer to match main framebuffer size
  // IMPORTANT: Use 0 samples (no multisampling) for ID buffer to ensure exact pixel values
  if (id_frame_buffer_ == nullptr) {
    id_frame_buffer_ = std::make_unique<FrameBuffer>(width, height, 0);  // No multisampling for ID picking
  } else if (id_frame_buffer_->GetWidth() != width || 
             id_frame_buffer_->GetHeight() != height) {
    id_frame_buffer_->Resize(width, height);
  }
  
  // Render to ID framebuffer
  id_frame_buffer_->Bind();
  id_frame_buffer_->Clear(0.0f, 0.0f, 0.0f, 0.0f);  // Black background = no point (ID 0)
  glClear(GL_DEPTH_BUFFER_BIT);  // Ensure depth buffer is cleared
  
  // Ensure proper OpenGL state for ID buffer rendering
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDisable(GL_BLEND); // No blending for ID buffer
  
  // CRITICAL: Set viewport to match ID framebuffer dimensions exactly
  glViewport(0, 0, static_cast<int>(width), static_cast<int>(height));
  
  // Apply coordinate system transformation if enabled
  glm::mat4 transform = scene_manager_->use_coord_transform_ ? scene_manager_->coord_transform_ : glm::mat4(1.0f);
  
  // Render point clouds in ID mode first
  for (auto& [name, obj] : scene_manager_->drawable_objects_) {
    PointCloud* point_cloud = dynamic_cast<PointCloud*>(obj.get());
    if (point_cloud) {
      // Get the object ID for this point cloud
      auto id_it = object_to_id_.find(name);
      if (id_it != object_to_id_.end()) {
        uint32_t object_id = id_it->second;
        
        // Temporarily switch to ID buffer rendering mode
        PointMode original_mode = point_cloud->GetRenderMode();
        point_cloud->SetRenderMode(PointMode::kIdBuffer);
        
        // Find the explicit range for this point cloud
        uint32_t id_base = kPointIdBase;
        for (const auto& range : point_cloud_ranges_) {
          if (range.point_cloud == point_cloud) {
            id_base = range.start_index;  // start_index is already the absolute ID
            break;
          }
        }
        
        point_cloud->SetObjectIdBase(id_base);
        
        // Render the point cloud with ID encoding
        point_cloud->OnDraw(projection, view, transform);
        
        // Restore original rendering mode
        point_cloud->SetRenderMode(original_mode);
      }
    }
  }
  
  // Render objects with unique ID colors for selection  
  for (const auto& [name, id] : object_to_id_) {
    auto obj_it = registered_objects_.find(name);
    if (obj_it == registered_objects_.end()) continue;
    
    OpenGlObject* object = obj_it->second;
    if (!object->SupportsSelection()) continue;
    
    // Convert ID to RGB color (24-bit)
    glm::vec3 id_color = IdToColor(id);
    
    // Use the new ID rendering interface to render objects with solid ID colors
    if (object->SupportsIdRendering()) {
      object->SetIdRenderMode(true);
      object->SetIdColor(id_color);
      object->OnDraw(projection, view, transform);
      object->SetIdRenderMode(false); // Restore normal rendering mode
    }
  }
  
  // Restore OpenGL state
  glEnable(GL_BLEND); // Re-enable blending for normal rendering
  
  id_frame_buffer_->Unbind();
}

uint32_t SelectionManager::ReadPixelId(int x, int y) {
  if (!id_frame_buffer_) {
    return kBackgroundId;
  }
  
  // Flip Y coordinate (OpenGL bottom-left vs screen top-left)
  int gl_y = static_cast<int>(id_frame_buffer_->GetHeight()) - y - 1;
  
  // Bind the ID framebuffer for reading
  id_frame_buffer_->Bind();
  
  // Read pixel RGB values
  uint8_t pixel[3];
  glReadPixels(x, gl_y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);
  
  id_frame_buffer_->Unbind();
  
  // Decode ID from RGB values (works for both points and objects)
  return ColorToId(pixel[0], pixel[1], pixel[2]);
}

void SelectionManager::ApplySelectionFeedback() {
  // TODO: Apply visual feedback for selected items
  // This should set highlighting on selected objects
}

void SelectionManager::ClearVisualFeedback() {
  // TODO: Clear visual feedback for all items
  // This should remove highlighting from all objects
}

// === Utility Functions ===

std::string GetSelectionName(const SelectionResult& result) {
  return std::visit(overloaded {
    [](const PointSelection& ps) { return ps.cloud_name + "_point_" + std::to_string(ps.point_index); },
    [](const ObjectSelection& os) { return os.object_name; },
    [](std::monostate) { return std::string(""); }
  }, result);
}

glm::vec3 GetSelectionWorldPosition(const SelectionResult& result) {
  return std::visit(overloaded {
    [](const PointSelection& ps) { return ps.world_position; },
    [](const ObjectSelection& os) { return os.world_position; },
    [](std::monostate) { return glm::vec3(0.0f); }
  }, result);
}

glm::vec2 GetSelectionScreenPosition(const SelectionResult& result) {
  return std::visit(overloaded {
    [](const PointSelection& ps) { return ps.screen_position; },
    [](const ObjectSelection& os) { return os.screen_position; },
    [](std::monostate) { return glm::vec2(0.0f); }
  }, result);
}

} // namespace quickviz