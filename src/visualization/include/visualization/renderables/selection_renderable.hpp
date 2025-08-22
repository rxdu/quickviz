/*
 * @file selection_renderable.hpp
 * @date 2025-01-22
 * @brief Renderable object for point selection visualization
 *
 * This class converts SelectionData into a renderable object that highlights
 * points in an existing PointCloud. It encapsulates the visualization logic
 * and provides a clean interface for selection rendering.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SELECTION_RENDERABLE_HPP
#define VISUALIZATION_SELECTION_RENDERABLE_HPP

#include "visualization/contracts/selection_data.hpp"
#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include <memory>
#include <string>

namespace quickviz {
namespace visualization {

/**
 * @brief Renderable object for point selection visualization
 * 
 * This class converts SelectionData into an OpenGL renderable object by
 * creating highlights on an existing PointCloud. It manages the layer
 * lifecycle and provides a clean interface between external data and rendering.
 */
class SelectionRenderable : public OpenGlObject {
public:
  /**
   * @brief Create a SelectionRenderable from external data
   * @param selection_data External selection specification
   * @param target_cloud Point cloud to highlight (must remain valid)
   * @return Unique pointer to renderable selection object
   */
  static std::unique_ptr<SelectionRenderable> FromData(
      const SelectionData& selection_data,
      PointCloud& target_cloud);

  /**
   * @brief Constructor (use FromData for creation)
   * @param selection_data Selection specification
   * @param target_cloud Target point cloud reference
   * @param layer_name Unique layer name for this selection
   */
  SelectionRenderable(const SelectionData& selection_data,
                     PointCloud& target_cloud,
                     const std::string& layer_name);

  ~SelectionRenderable();

  // OpenGlObject interface - delegated to target point cloud
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override;

  // Selection management
  void UpdateSelection(const SelectionData& new_selection_data);
  void ClearSelection();
  
  // Information access
  const SelectionData& GetSelectionData() const { return selection_data_; }
  const std::string& GetLayerName() const { return layer_name_; }
  size_t GetSelectedCount() const { return selection_data_.point_indices.size(); }

private:
  SelectionData selection_data_;
  PointCloud& target_cloud_;
  std::string layer_name_;
  bool is_applied_ = false;

  void ApplyHighlight();
  void RemoveHighlight();
  static std::string GenerateLayerName(const std::string& base_name);
};

} // namespace visualization
} // namespace quickviz

#endif // VISUALIZATION_SELECTION_RENDERABLE_HPP