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
   * @brief Get bounding box for this object (for ray intersection tests)
   * @return {min_bounds, max_bounds} in world space, or {{0,0,0}, {0,0,0}} if not available
   * @note Default returns zero bounds - subclasses can override for selection support
   */
  virtual std::pair<glm::vec3, glm::vec3> GetBoundingBox() const { 
    return {glm::vec3(0.0f), glm::vec3(0.0f)}; 
  }

 protected:
  OpenGlObject() = default;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_DRAWABLE_HPP
