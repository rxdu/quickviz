/*
 * @file surface_renderable.cpp
 * @date 2025-01-22
 * @brief Implementation of SurfaceRenderable class
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "visualization/renderables/surface_renderable.hpp"

namespace quickviz {
namespace visualization {

std::unique_ptr<SurfaceRenderable> SurfaceRenderable::FromData(
    const SurfaceData& surface_data) {
  
  if (surface_data.IsEmpty()) {
    return nullptr;
  }
  
  auto mesh = CreateMesh(surface_data);
  if (!mesh) {
    return nullptr;
  }
  
  return std::make_unique<SurfaceRenderable>(surface_data, std::move(mesh));
}

SurfaceRenderable::SurfaceRenderable(const SurfaceData& surface_data,
                                   std::unique_ptr<Mesh> mesh)
    : surface_data_(surface_data),
      mesh_(std::move(mesh)) {
  ConfigureMesh();
}

void SurfaceRenderable::AllocateGpuResources() {
  if (mesh_ && !IsGpuResourcesAllocated()) {
    mesh_->AllocateGpuResources();
  }
}

void SurfaceRenderable::ReleaseGpuResources() noexcept {
  if (mesh_ && IsGpuResourcesAllocated()) {
    mesh_->ReleaseGpuResources();
  }
}

void SurfaceRenderable::OnDraw(const glm::mat4& projection, 
                              const glm::mat4& view,
                              const glm::mat4& coord_transform) {
  if (mesh_ && IsGpuResourcesAllocated() && is_visible_) {
    mesh_->OnDraw(projection, view, coord_transform);
  }
}

bool SurfaceRenderable::IsGpuResourcesAllocated() const noexcept {
  return mesh_ && mesh_->IsGpuResourcesAllocated();
}

void SurfaceRenderable::UpdateSurface(const SurfaceData& new_surface_data) {
  surface_data_ = new_surface_data;
  
  // Recreate mesh with new data
  bool was_allocated = IsGpuResourcesAllocated();
  if (was_allocated) {
    mesh_->ReleaseGpuResources();
  }
  
  mesh_ = CreateMesh(surface_data_);
  if (mesh_) {
    ConfigureMesh();
    if (was_allocated) {
      mesh_->AllocateGpuResources();
    }
  }
}

void SurfaceRenderable::SetVisibility(bool visible) {
  is_visible_ = visible;
}

void SurfaceRenderable::SetTransparency(float transparency) {
  surface_data_.transparency = glm::clamp(transparency, 0.0f, 1.0f);
  if (mesh_) {
    mesh_->SetTransparency(surface_data_.transparency);
  }
}

void SurfaceRenderable::SetWireframeMode(bool wireframe) {
  surface_data_.show_wireframe = wireframe;
  if (mesh_) {
    mesh_->SetWireframeMode(wireframe);
    if (wireframe) {
      mesh_->SetWireframeColor(surface_data_.wireframe_color);
    }
  }
}

void SurfaceRenderable::ConfigureMesh() {
  if (!mesh_) return;
  
  // Set material properties
  mesh_->SetColor(surface_data_.color);
  mesh_->SetTransparency(surface_data_.transparency);
  
  // Configure wireframe if enabled
  if (surface_data_.show_wireframe) {
    mesh_->SetWireframeMode(true);
    mesh_->SetWireframeColor(surface_data_.wireframe_color);
  }
  
  // Configure normal visualization if enabled
  if (surface_data_.show_normals) {
    mesh_->SetShowNormals(true, surface_data_.normal_scale);
    mesh_->SetNormalColor(surface_data_.normal_color);
  }
}

std::unique_ptr<Mesh> SurfaceRenderable::CreateMesh(
    const SurfaceData& data) {
  
  if (data.IsEmpty() || data.triangle_indices.size() % 3 != 0) {
    return nullptr;
  }
  
  auto mesh = std::make_unique<Mesh>();
  
  // Set vertex data
  mesh->SetVertices(data.vertices);
  mesh->SetIndices(data.triangle_indices);
  
  // Set normals if available
  if (data.HasNormals()) {
    mesh->SetNormals(data.normals);
  }
  // Note: Mesh will auto-generate normals during AllocateGpuResources if needed
  
  return mesh;
}

} // namespace visualization
} // namespace quickviz