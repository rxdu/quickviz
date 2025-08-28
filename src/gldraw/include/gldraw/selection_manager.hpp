/**
 * @file selection_manager.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-28
 * @brief Interactive selection system for 3D scenes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SELECTION_MANAGER_HPP
#define QUICKVIZ_SELECTION_MANAGER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <memory>
#include <variant>

#include <glm/glm.hpp>
#include "gldraw/interface/opengl_object.hpp"

namespace quickviz {

// Forward declarations
class GlSceneManager;
class PointCloud;
class FrameBuffer;

/**
 * @brief Selection result for a single point in a point cloud
 */
struct PointSelection {
  std::string cloud_name;                  // Name of the point cloud
  PointCloud* point_cloud = nullptr;      // Pointer to the point cloud
  size_t point_index = SIZE_MAX;          // Index within the point cloud
  glm::vec3 world_position{0.0f};         // 3D world coordinates
  glm::vec2 screen_position{0.0f};        // Screen coordinates where clicked
  
  // Equality operator for std::variant operations
  bool operator==(const PointSelection& other) const {
    return cloud_name == other.cloud_name && 
           point_cloud == other.point_cloud && 
           point_index == other.point_index;
  }
};

/**
 * @brief Selection result for a 3D object/primitive
 */
struct ObjectSelection {
  std::string object_name;                 // Name of the object
  OpenGlObject* object = nullptr;         // Pointer to the object
  glm::vec3 world_position{0.0f};         // 3D world coordinates (approximate)
  glm::vec2 screen_position{0.0f};        // Screen coordinates where clicked
  
  // Equality operator for std::variant operations
  bool operator==(const ObjectSelection& other) const {
    return object_name == other.object_name && 
           object == other.object;
  }
};

/**
 * @brief Type-safe selection result using std::variant
 * Can represent no selection, point selection, or object selection
 */
using SelectionResult = std::variant<std::monostate, PointSelection, ObjectSelection>;

/**
 * @brief Collection of multiple selections for multi-selection operations
 */
class MultiSelection {
public:
  MultiSelection() = default;
  
  // Access methods
  size_t Count() const { return selections_.size(); }
  bool Empty() const { return selections_.empty(); }
  const std::vector<SelectionResult>& GetSelections() const { return selections_; }
  
  // Modification methods
  void Add(const SelectionResult& selection);
  void Remove(const SelectionResult& selection);
  void Toggle(const SelectionResult& selection);
  void Clear() { selections_.clear(); }
  
  // Analysis methods
  glm::vec3 GetCentroid() const;
  std::pair<glm::vec3, glm::vec3> GetBounds() const;
  
  // Type-specific access
  std::vector<PointSelection> GetPoints() const;
  std::vector<ObjectSelection> GetObjects() const;
  
private:
  std::vector<SelectionResult> selections_;
};

/**
 * @brief Selection mode for handling overlapping selectable items
 */
enum class SelectionMode {
  kObjects,     // Only select objects, ignore points
  kPoints,      // Only select points, ignore objects  
  kHybrid,      // Points have priority over objects
  kClosest      // Closest to camera wins (using depth buffer)
};

/**
 * @brief Selection options for customizing selection behavior
 */
struct SelectionOptions {
  int radius = 3;                          // Selection tolerance in pixels
  SelectionMode mode = SelectionMode::kHybrid;
  std::string target_object = "";          // Empty = select any, otherwise only this object
  bool add_to_selection = false;           // Add to multi-selection instead of replacing
  
  // Selection filter - return true to allow selection of this candidate
  using FilterFunction = std::function<bool(const SelectionResult&)>;
  FilterFunction filter = nullptr;
};

/**
 * @brief Rectangle region for area selection
 */
struct SelectionRectangle {
  glm::vec2 min;  // Bottom-left corner
  glm::vec2 max;  // Top-right corner
  
  SelectionRectangle(const glm::vec2& corner1, const glm::vec2& corner2) {
    min = glm::min(corner1, corner2);
    max = glm::max(corner1, corner2);
  }
  
  bool Contains(const glm::vec2& point) const {
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
  }
  
  float Width() const { return max.x - min.x; }
  float Height() const { return max.y - min.y; }
};

/**
 * @brief Main selection system for interactive 3D scene selection
 * 
 * This system handles both individual point selection within point clouds
 * and whole object selection for 3D primitives. It uses GPU ID-buffer
 * rendering internally for pixel-perfect accuracy.
 * 
 * Key features:
 * - Single-click selection with tolerance radius
 * - Multi-selection with Ctrl+Click semantics
 * - Rectangle/area selection
 * - Customizable selection filters
 * - Type-safe result handling with std::variant
 */
class SelectionManager {
public:
  explicit SelectionManager(GlSceneManager* scene_manager);
  ~SelectionManager() = default;

  // Disable copy/move for simplicity
  SelectionManager(const SelectionManager&) = delete;
  SelectionManager& operator=(const SelectionManager&) = delete;
  
  // === Primary Selection Interface ===
  
  /**
   * @brief Main selection method - single entry point for all selection operations
   * @param screen_x Screen X coordinate (window coordinates)
   * @param screen_y Screen Y coordinate (window coordinates)
   * @param options Selection options (radius, mode, filters, etc.)
   * @return Selection result (empty if nothing selected)
   */
  SelectionResult Select(float screen_x, float screen_y, const SelectionOptions& options = {});
  
  /**
   * @brief Convenience method for simple point selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate  
   * @param radius Selection radius in pixels
   * @return Selection result
   */
  SelectionResult SelectPoint(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Convenience method for simple object selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @return Selection result
   */
  SelectionResult SelectObject(float screen_x, float screen_y);
  
  // === Multi-Selection Interface ===
  
  /**
   * @brief Add to current multi-selection (Ctrl+Click semantics)
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param options Selection options
   * @return true if something was selected and added
   */
  bool AddToSelection(float screen_x, float screen_y, const SelectionOptions& options = {});
  
  /**
   * @brief Toggle selection state (Ctrl+Click on already selected item)
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param options Selection options
   * @return true if something was found and toggled
   */
  bool ToggleSelection(float screen_x, float screen_y, const SelectionOptions& options = {});
  
  /**
   * @brief Select all items within a rectangular region
   * @param rectangle Selection rectangle in screen coordinates
   * @param options Selection options (applied to each candidate)
   * @return Number of items selected
   */
  size_t SelectRegion(const SelectionRectangle& rectangle, const SelectionOptions& options = {});
  
  /**
   * @brief Clear all selections
   */
  void ClearSelection();
  
  // === Selection State Access ===
  
  /**
   * @brief Get current single selection (most recent)
   * @return Selection result (empty if no selection)
   */
  const SelectionResult& GetCurrentSelection() const { return current_selection_; }
  
  /**
   * @brief Get current multi-selection
   * @return Multi-selection object containing all selected items
   */
  const MultiSelection& GetMultiSelection() const { return multi_selection_; }
  
  /**
   * @brief Check if anything is currently selected
   */
  bool HasSelection() const;
  
  /**
   * @brief Get total number of selected items
   */
  size_t GetSelectionCount() const { return multi_selection_.Count(); }
  
  // === Configuration ===
  
  /**
   * @brief Set default selection mode
   * @param mode Selection mode for handling overlaps
   */
  void SetMode(SelectionMode mode) { default_mode_ = mode; }
  
  /**
   * @brief Get current default selection mode
   */
  SelectionMode GetMode() const { return default_mode_; }
  
  // === Callbacks ===
  
  /**
   * @brief Callback type for selection changes
   * @param result Current single selection (empty if none)
   * @param multi_selection Current multi-selection state
   */
  using SelectionCallback = std::function<void(const SelectionResult&, const MultiSelection&)>;
  
  /**
   * @brief Set callback for selection changes
   * @param callback Function to call when selection changes
   */
  void SetSelectionCallback(SelectionCallback callback) { callback_ = callback; }
  
  // === Object Registration (for ID mapping) ===
  
  /**
   * @brief Register an object for selection (called automatically by GlSceneManager)
   * @param name Object name
   * @param object Object pointer
   */
  void RegisterObject(const std::string& name, OpenGlObject* object);
  
  /**
   * @brief Unregister an object (called automatically by GlSceneManager)
   * @param name Object name
   */
  void UnregisterObject(const std::string& name);
  
  // === Internal Access (for GlSceneManager) ===
  
  /**
   * @brief Get registered objects for ID buffer rendering
   * @return Map of object names to pointers
   */
  const std::unordered_map<std::string, OpenGlObject*>& GetRegisteredObjects() const { 
    return registered_objects_; 
  }

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
  uint32_t ColorToId(uint8_t r, uint8_t g, uint8_t b);
  
  // Selection processing
  SelectionResult ProcessSingleSelection(uint32_t id, float screen_x, float screen_y);
  SelectionResult FindObjectById(uint32_t object_id, float screen_x, float screen_y);
  SelectionResult FindPointById(uint32_t point_id, float screen_x, float screen_y);
  
  // Multi-selection helpers
  void ApplyFilter(std::vector<SelectionResult>& candidates, const SelectionOptions& options);
  void UpdateSelectionState(const SelectionResult& new_selection, bool add_to_multi);
  void NotifySelectionChanged();
  
  // ID buffer rendering and reading
  void RenderIdBuffer();
  uint32_t ReadPixelId(int x, int y);
  std::vector<uint32_t> ReadRegionIds(const SelectionRectangle& rect);
  
  // Visual feedback
  void ApplySelectionFeedback();
  void ClearVisualFeedback();
  
  // Internal state
  GlSceneManager* scene_manager_;
  SelectionResult current_selection_;
  MultiSelection multi_selection_;
  SelectionMode default_mode_ = SelectionMode::kHybrid;
  SelectionCallback callback_;
  
  // Object registration
  std::unordered_map<std::string, OpenGlObject*> registered_objects_;
  std::unordered_map<std::string, uint32_t> object_to_id_;
  uint32_t next_object_id_ = kObjectIdBase;
  
  // Point cloud registration for point selection
  std::unordered_map<std::string, PointCloud*> registered_point_clouds_;
  
  // ID framebuffer for GPU-based selection
  std::unique_ptr<FrameBuffer> id_frame_buffer_;
};

// === Utility Functions ===

/**
 * @brief Helper for std::visit pattern matching on SelectionResult
 */
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

/**
 * @brief Check if a selection result is empty (no selection)
 */
inline bool IsEmpty(const SelectionResult& result) {
  return std::holds_alternative<std::monostate>(result);
}

/**
 * @brief Get the name of the selected item (works for both points and objects)
 */
std::string GetSelectionName(const SelectionResult& result);

/**
 * @brief Get the world position of the selected item
 */
glm::vec3 GetSelectionWorldPosition(const SelectionResult& result);

/**
 * @brief Get the screen position of the selected item
 */
glm::vec2 GetSelectionScreenPosition(const SelectionResult& result);

} // namespace quickviz

#endif // QUICKVIZ_SELECTION_MANAGER_HPP