/*
 * @file render_interface.hpp
 * @date 8/27/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_RENDER_INTERFACE_HPP
#define QUICKVIZ_RENDER_INTERFACE_HPP

#include <string>

#include <glm/glm.hpp>

namespace quickviz {

// Forward declarations
enum class VirtualObjectType : int;

// Ray-casting removed - using GPU ID-buffer selection exclusively

/**
 * @brief Virtual object data for backend operations
 *
 * Simplified data structure that matches what virtual objects actually need
 * to communicate to the rendering backend.
 */
struct VirtualObjectData {
  // Transform and visibility
  glm::mat4 transform = glm::mat4(1.0f);
  bool visible = true;

  // Visual state
  glm::vec3 color = glm::vec3(1.0f);
  float alpha = 1.0f;
  bool highlighted = false;  // For selection/hover feedback

  // Type-specific parameters (for primitive objects)
  struct {
    float radius = 1.0f;               // For spheres
    glm::vec3 size = glm::vec3(1.0f);  // For boxes
    float height = 1.0f;               // For cylinders
    int tessellation = 16;             // Quality setting
  } geometry;
};

/**
 * @brief Abstract interface for rendering backends
 *
 * RenderInterface provides a clean interface for virtual scene rendering.
 * The design is specifically tailored to wrap GlSceneManager efficiently
 * while allowing future backends.
 */
class RenderInterface {
 public:
  virtual ~RenderInterface() = default;

  // Object lifecycle (matches GlSceneManager patterns)
  virtual void CreateObject(const std::string& id, VirtualObjectType type,
                            const VirtualObjectData& initial_data) = 0;
  virtual void UpdateObject(const std::string& id,
                            const VirtualObjectData& data) = 0;
  virtual void RemoveObject(const std::string& id) = 0;
  virtual void ClearAllObjects() = 0;

  // Rendering pipeline (matches GlSceneManager RenderToFramebuffer pattern)
  virtual void RenderToFramebuffer(float width, float height) = 0;
  virtual uint32_t GetFramebufferTexture() const = 0;

  // Interaction support (GPU ID-buffer selection only)
  virtual std::string PickObjectAt(float screen_x, float screen_y) = 0;

  // Viewport and camera (delegate to underlying system)
  virtual void SetBackgroundColor(float r, float g, float b, float a) = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RENDER_INTERFACE_HPP
