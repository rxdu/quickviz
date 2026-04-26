/**
 * @file tf_frame_tree.hpp
 * @brief Renderable tree of named coordinate frames
 *
 * Visualizes a hierarchy of coordinate frames analogous to ROS tf2: each
 * frame has a name and a transform expressed in its parent's frame. World
 * transforms are computed by walking parent chains.
 *
 * Each frame renders as a small RGB axis triplet (X=red, Y=green,
 * Z=blue). Parent → child connection lines are drawn in gray when
 * enabled, giving a quick visual of tree topology.
 *
 * Threading: `SetFrame`, `RemoveFrame`, and `Clear` mutate internal
 * state and rebuild the GPU vertex buffer; they must be called on the
 * render thread. `OnDraw` is called by the scene manager.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_TF_FRAME_TREE_HPP
#define QUICKVIZ_TF_FRAME_TREE_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "scene/interface/opengl_object.hpp"
#include "../shader_program.hpp"

namespace quickviz {

class TfFrameTree : public OpenGlObject {
 public:
  explicit TfFrameTree(float axis_length = 0.3f);
  ~TfFrameTree();

  // === Frame management ===

  /**
   * @brief Add or update a frame.
   *
   * @param name              Unique frame identifier.
   * @param parent            Parent frame name, or empty string for a
   *                          root frame (rooted in world).
   * @param transform_in_parent  4x4 transform of `name` in `parent`'s
   *                             coordinates (or world if no parent).
   *
   * If `name` already exists, its parent and transform are updated;
   * children are re-rooted automatically because they reference by
   * parent name.
   */
  void SetFrame(const std::string& name, const std::string& parent,
                const glm::mat4& transform_in_parent);

  /**
   * @brief Remove a frame and all its descendants.
   * @return Number of frames removed.
   */
  std::size_t RemoveFrame(const std::string& name);

  /// Remove all frames.
  void Clear();

  /// Number of frames currently in the tree.
  std::size_t GetFrameCount() const { return frames_.size(); }

  /// Returns true if the frame exists in the tree.
  bool HasFrame(const std::string& name) const;

  /**
   * @brief World-space transform for a named frame.
   *
   * Walks parents until a root is reached. Returns identity if the
   * frame doesn't exist or has a broken parent chain.
   */
  glm::mat4 GetWorldTransform(const std::string& name) const;

  // === Appearance ===

  /// Length (world units) of each axis line. Default: 0.3.
  void SetAxisLength(float length);

  /// Whether to draw lines connecting parent origins to child origins.
  void SetShowConnections(bool show) { show_connections_ = show; dirty_ = true; }
  bool GetShowConnections() const { return show_connections_; }

  /// Color of parent→child connection lines. Default: medium gray.
  void SetConnectionColor(const glm::vec3& color) {
    connection_color_ = color;
    dirty_ = true;
  }

  // === OpenGlObject interface ===

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override {
    return vao_ != 0;
  }

 private:
  struct Frame {
    std::string parent;             // empty == world root
    glm::mat4 local{1.0f};          // transform in parent's frame
  };

  struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
  };

  // Recompute the vertex buffer from the current tree state. Called
  // lazily before drawing if dirty_ is set.
  void RebuildVertices();

  std::unordered_map<std::string, Frame> frames_;
  float axis_length_ = 0.3f;
  bool show_connections_ = true;
  glm::vec3 connection_color_ = glm::vec3(0.5f, 0.5f, 0.5f);

  // GL resources
  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  ShaderProgram shader_;

  // CPU-side vertex cache (rebuilt when tree changes).
  std::vector<Vertex> vertices_;
  bool dirty_ = true;
  std::size_t buffer_capacity_ = 0;
};

}  // namespace quickviz

#endif  // QUICKVIZ_TF_FRAME_TREE_HPP
