/*
 * @file surface_renderable.hpp
 * @date 2025-01-22
 * @brief Renderable object for surface/mesh visualization
 *
 * This class converts SurfaceData into a renderable triangle mesh object.
 * It encapsulates the OpenGL mesh rendering logic and provides a clean
 * interface for surface visualization.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SURFACE_RENDERABLE_HPP
#define VISUALIZATION_SURFACE_RENDERABLE_HPP

#include "visualization/contracts/surface_data.hpp"
#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/renderable/mesh.hpp"
#include <memory>
#include <string>

namespace quickviz {
namespace visualization {

/**
 * @brief Renderable object for surface/mesh visualization
 * 
 * This class converts SurfaceData into an OpenGL renderable triangle mesh
 * by creating a TriangleMesh object with appropriate materials and settings.
 */
class SurfaceRenderable : public OpenGlObject {
public:
  /**
   * @brief Create a SurfaceRenderable from external data
   * @param surface_data External surface specification
   * @return Unique pointer to renderable surface object
   */
  static std::unique_ptr<SurfaceRenderable> FromData(
      const SurfaceData& surface_data);

  /**
   * @brief Constructor (use FromData for creation)
   * @param surface_data Surface specification
   * @param mesh Underlying mesh object
   */
  SurfaceRenderable(const SurfaceData& surface_data,
                   std::unique_ptr<Mesh> mesh);

  ~SurfaceRenderable() = default;

  // OpenGlObject interface - delegated to triangle mesh
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override;

  // Surface management
  void UpdateSurface(const SurfaceData& new_surface_data);
  void SetVisibility(bool visible);
  void SetTransparency(float transparency);
  void SetWireframeMode(bool wireframe);
  
  // Information access
  const SurfaceData& GetSurfaceData() const { return surface_data_; }
  const std::string& GetSurfaceName() const { return surface_data_.surface_name; }
  size_t GetVertexCount() const { return surface_data_.GetVertexCount(); }
  size_t GetTriangleCount() const { return surface_data_.GetTriangleCount(); }
  bool IsVisible() const { return is_visible_; }

private:
  SurfaceData surface_data_;
  std::unique_ptr<Mesh> mesh_;
  bool is_visible_ = true;

  void ConfigureMesh();
  static std::unique_ptr<Mesh> CreateMesh(const SurfaceData& data);
};

} // namespace visualization
} // namespace quickviz

#endif // VISUALIZATION_SURFACE_RENDERABLE_HPP