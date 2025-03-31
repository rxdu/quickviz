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

namespace quickviz {
class OpenGlObject {
 public:
  virtual ~OpenGlObject() = default;

  /****** public methods ******/
  virtual void AllocateGpuResources() = 0;
  virtual void ReleaseGpuResources() = 0;
  
  /**
   * @brief Draw the OpenGL object
   * 
   * @param projection The projection matrix
   * @param view The view matrix
   * @param coord_transform The coordinate system transformation matrix (from standard to OpenGL)
   */
  virtual void OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                     const glm::mat4& coord_transform = glm::mat4(1.0f)) = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_DRAWABLE_HPP
