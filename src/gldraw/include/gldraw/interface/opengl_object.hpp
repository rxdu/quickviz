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

 protected:
  OpenGlObject() = default;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_DRAWABLE_HPP
