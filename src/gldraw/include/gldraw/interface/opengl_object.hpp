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

 protected:
  OpenGlObject() = default;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_DRAWABLE_HPP
