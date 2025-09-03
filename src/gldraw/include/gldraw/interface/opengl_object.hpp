/*
 * @file opengl_drawable.hpp
 * @date 11/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_OPENGL_DRAWABLE_HPP
#define QUICKVIZ_OPENGL_DRAWABLE_HPP

#include <glm/glm.hpp>
#include <stdexcept>
#include <limits>

// Forward declarations for selection system
namespace quickviz {
enum class SelectionPriority {
  kBackground = 0,
  kObject = 1,     // Regular 3D objects (spheres, cubes, etc.)
  kPoint = 2       // Individual points in point clouds
};
}

namespace quickviz {
class OpenGlObject {
 public:
  virtual ~OpenGlObject() = default;

  // Disable copy construction and assignment
  OpenGlObject(const OpenGlObject&) = delete;
  OpenGlObject& operator=(const OpenGlObject&) = delete;

  // Enable move construction and assignment
  OpenGlObject(OpenGlObject&&) = default;
  OpenGlObject& operator=(OpenGlObject&&) = default;

  /****** public methods ******/
  
  /**
   * @brief Allocate GPU resources for rendering
   * @throws std::runtime_error if allocation fails
   */
  virtual void AllocateGpuResources() = 0;
  
  /**
   * @brief Release GPU resources
   * @note This method should not throw exceptions
   */
  virtual void ReleaseGpuResources() noexcept = 0;
  
  /**
   * @brief Draw the OpenGL object
   * 
   * @param projection The projection matrix
   * @param view The view matrix
   * @param coord_transform The coordinate system transformation matrix (from standard to OpenGL)
   * @throws std::runtime_error if rendering fails
   */
  virtual void OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                     const glm::mat4& coord_transform = glm::mat4(1.0f)) = 0;
                     
  /**
   * @brief Check if GPU resources are allocated
   * @return true if resources are allocated, false otherwise
   */
  virtual bool IsGpuResourcesAllocated() const noexcept = 0;
  
  /**
   * @brief Set whether this object is highlighted (selected)
   * @param highlighted Whether to highlight the object
   * @note Default implementation does nothing - subclasses can override to provide visual feedback
   */
  virtual void SetHighlighted(bool highlighted) { (void)highlighted; }
  
  /**
   * @brief Check if this object supports selection/picking
   * @return true if object can be selected, false otherwise
   * @note Default is false - subclasses that support selection should override
   */
  virtual bool SupportsSelection() const { return false; }
  
  /**
   * @brief Get bounding box for this object (for selection support)
   * @return {min_bounds, max_bounds} in world space, or {{0,0,0}, {0,0,0}} if not available
   * @note Default returns zero bounds - subclasses can override for selection support
   */
  virtual std::pair<glm::vec3, glm::vec3> GetBoundingBox() const { 
    return {glm::vec3(0.0f), glm::vec3(0.0f)}; 
  }
  
  // === GPU ID-Buffer Selection System ===
  
  /**
   * @brief Check if this object supports point-level picking
   * @return true if individual points/vertices can be picked, false otherwise
   * @note Default is false - point clouds and meshes with vertex picking should override
   */
  virtual bool SupportsPointPicking() const { return false; }
  
  /**
   * @brief Pick a specific point/vertex within this object
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate  
   * @param screen_width Viewport width
   * @param screen_height Viewport height
   * @param projection Projection matrix
   * @param view View matrix
   * @param coord_transform Coordinate transformation matrix
   * @return Point index within object, or SIZE_MAX if no point found
   * @note Default returns SIZE_MAX - objects supporting point picking should override
   */
  virtual size_t PickPointAt(float screen_x, float screen_y,
                            float screen_width, float screen_height,
                            const glm::mat4& projection,
                            const glm::mat4& view,
                            const glm::mat4& coord_transform = glm::mat4(1.0f)) const {
    return SIZE_MAX;
  }
  
  /**
   * @brief Get 3D position of a point within this object
   * @param point_index Index of point within object
   * @return 3D position in world space, or zero vector if invalid index
   * @note Default returns zero - objects with points should override
   */
  virtual glm::vec3 GetPointPosition(size_t point_index) const {
    return glm::vec3(0.0f);
  }
  
  /**
   * @brief Get selection priority for this object type
   * @return Priority level (lower values = higher priority)
   * @note Default is object-level priority
   */
  virtual SelectionPriority GetSelectionPriority() const { 
    return SelectionPriority::kObject; 
  }
  
  // === GPU ID-Buffer Selection Support ===
  
  /**
   * @brief Enable/disable ID rendering mode for GPU-based selection
   * @param enabled Whether to render with solid ID color instead of normal rendering
   * @note When enabled, object should render with solid color, no textures/lighting
   * @note Default implementation does nothing - subclasses should override to support ID rendering
   */
  virtual void SetIdRenderMode(bool enabled) { (void)enabled; }
  
  /**
   * @brief Set the ID color for this object in ID rendering mode
   * @param color RGB color representing the unique ID (values 0-1)
   * @note This color encodes the object's selection ID for GPU picking
   * @note Default implementation does nothing - subclasses should override to support ID rendering
   */
  virtual void SetIdColor(const glm::vec3& color) { (void)color; }
  
  /**
   * @brief Check if this object supports ID rendering mode
   * @return true if object can render with ID colors for GPU selection
   * @note Default is false - subclasses supporting GPU selection should override
   */
  virtual bool SupportsIdRendering() const { return false; }

  // === Transform and Visibility State Management ===
  
  /**
   * @brief Set object transform matrix (position, rotation, scale)
   * @param transform 4x4 transformation matrix in local-to-world space
   * @note This is the primary interface for state management integration
   */
  virtual void SetTransform(const glm::mat4& transform) { 
    transform_ = transform; 
    MarkTransformDirty();
  }
  
  /**
   * @brief Get current object transform matrix
   * @return Current transformation matrix
   */
  virtual glm::mat4 GetTransform() const { 
    return transform_; 
  }
  
  /**
   * @brief Set object visibility state
   * @param visible True to show object, false to hide
   * @note Hidden objects should skip rendering entirely
   */
  virtual void SetVisible(bool visible) { 
    visible_ = visible; 
  }
  
  /**
   * @brief Check if object is visible
   * @return True if object should be rendered
   */
  virtual bool IsVisible() const { 
    return visible_; 
  }
  
  /**
   * @brief Get object bounding box in world space
   * @return Pair of {min_corner, max_corner} in world coordinates
   * @note Default implementation transforms local bounds by current transform
   */
  virtual std::pair<glm::vec3, glm::vec3> GetWorldBounds() const {
    auto local_bounds = GetBoundingBox();
    if (local_bounds.first == glm::vec3(0.0f) && local_bounds.second == glm::vec3(0.0f)) {
      return local_bounds; // No bounds available
    }
    
    // Transform bounding box corners by current transform
    glm::vec3 corners[8] = {
      {local_bounds.first.x, local_bounds.first.y, local_bounds.first.z},
      {local_bounds.second.x, local_bounds.first.y, local_bounds.first.z},
      {local_bounds.first.x, local_bounds.second.y, local_bounds.first.z},
      {local_bounds.second.x, local_bounds.second.y, local_bounds.first.z},
      {local_bounds.first.x, local_bounds.first.y, local_bounds.second.z},
      {local_bounds.second.x, local_bounds.first.y, local_bounds.second.z},
      {local_bounds.first.x, local_bounds.second.y, local_bounds.second.z},
      {local_bounds.second.x, local_bounds.second.y, local_bounds.second.z}
    };
    
    glm::vec3 min_world(std::numeric_limits<float>::max());
    glm::vec3 max_world(std::numeric_limits<float>::lowest());
    
    for (const auto& corner : corners) {
      glm::vec4 world_corner = transform_ * glm::vec4(corner, 1.0f);
      glm::vec3 world_pos = glm::vec3(world_corner);
      
      min_world = glm::min(min_world, world_pos);
      max_world = glm::max(max_world, world_pos);
    }
    
    return {min_world, max_world};
  }

 protected:
  OpenGlObject() = default;
  
  /**
   * @brief Mark transform as dirty for lazy update patterns
   * 
   * Called automatically when transform changes. Subclasses can override
   * to implement efficient update strategies (e.g., only recalculate
   * world-space data when actually needed for rendering).
   */
  virtual void MarkTransformDirty() {
    // Default implementation does nothing
    // Subclasses can override for lazy evaluation
  }

private:
  // Core state that all objects should have
  glm::mat4 transform_ = glm::mat4(1.0f);  ///< Local-to-world transform
  bool visible_ = true;                     ///< Visibility state
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_DRAWABLE_HPP
