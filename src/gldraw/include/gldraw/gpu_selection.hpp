/**
 * @file gpu_selection.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-28
 * @brief GPU ID-buffer selection system for interactive object picking
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_GPU_SELECTION_HPP
#define QUICKVIZ_GPU_SELECTION_HPP

#include <string>
#include <unordered_map>
#include <functional>
#include <memory>

#include <glm/glm.hpp>
#include "gldraw/interface/opengl_object.hpp"

namespace quickviz {

// Forward declarations
class GlSceneManager;
class PointCloud;

/**
 * @brief Selection result for GPU-based picking
 */
struct GPUSelectionResult {
  enum class Type {
    kNone,
    kObject,      // Geometric object (sphere, box, mesh, etc.)
    kPoint        // Individual point in point cloud
  };

  Type type = Type::kNone;
  
  // Common data
  std::string name;                     // Object or point cloud name
  glm::vec2 screen_position{0.0f};     // Screen coordinates where selection occurred
  glm::vec3 world_position{0.0f};      // 3D world position of selection
  
  // Object-specific data
  OpenGlObject* object = nullptr;       // Pointer to selected object
  
  // Point-specific data  
  PointCloud* point_cloud = nullptr;   // Pointer to point cloud containing the point
  size_t point_index = SIZE_MAX;       // Index of selected point within the cloud
  
  // Utility methods
  bool IsValid() const { return type != Type::kNone; }
  bool IsObject() const { return type == Type::kObject; }
  bool IsPoint() const { return type == Type::kPoint; }
  
  // Factory methods
  static GPUSelectionResult None() {
    return GPUSelectionResult{};
  }
  
  static GPUSelectionResult Object(const std::string& obj_name, 
                                  OpenGlObject* obj, 
                                  const glm::vec3& world_pos,
                                  const glm::vec2& screen_pos) {
    GPUSelectionResult result;
    result.type = Type::kObject;
    result.name = obj_name;
    result.object = obj;
    result.world_position = world_pos;
    result.screen_position = screen_pos;
    return result;
  }
  
  static GPUSelectionResult Point(const std::string& cloud_name,
                                 PointCloud* cloud,
                                 size_t point_idx,
                                 const glm::vec3& point_pos,
                                 const glm::vec2& screen_pos) {
    GPUSelectionResult result;
    result.type = Type::kPoint;
    result.name = cloud_name;
    result.point_cloud = cloud;
    result.point_index = point_idx;
    result.world_position = point_pos;
    result.screen_position = screen_pos;
    return result;
  }
};

/**
 * @brief Selection priority when multiple selectable items are at same pixel
 */
enum class GPUSelectionMode {
  kObjects,     // Only select objects, ignore points
  kPoints,      // Only select points, ignore objects  
  kHybrid,      // Points have priority over objects
  kClosest      // Closest to camera wins (using GPU depth buffer)
};

/**
 * @brief GPU-based selection system
 * 
 * This system uses GPU ID-buffer rendering for all selection operations.
 * It renders the entire scene with unique colors representing object/point IDs,
 * then reads the pixel at the mouse position to determine what was selected.
 * 
 * ID Encoding:
 * - 0x000000: Background (no selection)
 * - 0x000001-0x7FFFFF: Point indices (up to 8.4M points)
 * - 0x800000-0xFFFFFF: Object IDs (up to 8.4M objects)
 */
class GPUSelection {
 public:
  explicit GPUSelection(GlSceneManager* scene_manager);
  ~GPUSelection() = default;

  // Disable copy/move for simplicity
  GPUSelection(const GPUSelection&) = delete;
  GPUSelection& operator=(const GPUSelection&) = delete;
  
  /**
   * @brief Main selection method - single entry point for all GPU-based selection
   * @param pixel_x Screen X coordinate in pixels
   * @param pixel_y Screen Y coordinate in pixels (OpenGL coordinates, origin bottom-left)
   * @param radius Selection radius in pixels (for tolerance)
   * @return Selection result
   */
  GPUSelectionResult SelectAt(int pixel_x, int pixel_y, int radius = 0);
  
  /**
   * @brief Select at screen coordinates with automatic pixel conversion
   * @param screen_x Screen X coordinate (window coordinates)
   * @param screen_y Screen Y coordinate (window coordinates) 
   * @param screen_width Window width
   * @param screen_height Window height
   * @param radius Selection radius in pixels
   * @return Selection result
   */
  GPUSelectionResult SelectAtScreen(float screen_x, float screen_y,
                                   float screen_width, float screen_height,
                                   int radius = 0);
  
  /**
   * @brief Clear current selection and visual feedback
   */
  void ClearSelection();
  
  /**
   * @brief Get current selection
   */
  const GPUSelectionResult& GetCurrentSelection() const { return current_selection_; }
  
  // Configuration
  void SetMode(GPUSelectionMode mode) { mode_ = mode; }
  GPUSelectionMode GetMode() const { return mode_; }
  
  // Callback for selection changes
  using SelectionCallback = std::function<void(const GPUSelectionResult&)>;
  void SetSelectionCallback(SelectionCallback callback) { callback_ = callback; }
  
  // Object registration (for ID mapping)
  void RegisterObject(const std::string& name, OpenGlObject* object);
  void UnregisterObject(const std::string& name);
  
  // Public access for GlSceneManager to render objects with ID colors
  const std::unordered_map<std::string, OpenGlObject*>& GetRegisteredObjects() const { return registered_objects_; }
  
 private:
  // ID encoding/decoding
  static constexpr uint32_t kBackgroundId = 0x000000;
  static constexpr uint32_t kPointIdBase = 0x000001;
  static constexpr uint32_t kObjectIdBase = 0x800000;
  static constexpr uint32_t kMaxPointId = 0x7FFFFF;
  static constexpr uint32_t kMaxObjectId = 0xFFFFFF;
  
  uint32_t EncodeObjectId(const std::string& object_name);
  uint32_t EncodePointId(size_t point_index);
  glm::vec3 IdToColor(uint32_t id);
  uint32_t ColorToId(const glm::vec3& color);
  uint32_t ColorToId(uint8_t r, uint8_t g, uint8_t b);
  
  // Selection processing
  GPUSelectionResult ProcessSelection(uint32_t id, int pixel_x, int pixel_y);
  GPUSelectionResult FindObjectById(uint32_t object_id, int pixel_x, int pixel_y);
  GPUSelectionResult FindPointById(uint32_t point_id, int pixel_x, int pixel_y);
  
  // ID buffer rendering
  void RenderIdBuffer();
  uint32_t ReadPixelId(int x, int y);
  
  // Visual feedback
  void ApplySelection(const GPUSelectionResult& result);
  void ClearVisualFeedback();
  
  // Internal state
  GlSceneManager* scene_manager_;
  GPUSelectionResult current_selection_;
  GPUSelectionMode mode_ = GPUSelectionMode::kHybrid;
  SelectionCallback callback_;
  
  // Object registration
  std::unordered_map<std::string, OpenGlObject*> registered_objects_;
  std::unordered_map<std::string, uint32_t> object_id_map_;
  uint32_t next_object_id_ = kObjectIdBase;
  
  // Visual feedback tracking
  std::string highlighted_object_name_;
  std::string highlighted_point_cloud_name_;
  size_t highlighted_point_index_ = SIZE_MAX;
};

}  // namespace quickviz

#endif  // QUICKVIZ_GPU_SELECTION_HPP