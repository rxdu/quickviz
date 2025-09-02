/*
 * scene_manager.hpp
 *
 * Created on 3/6/25 9:09 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef GL_SCENE_MANAGER_HPP
#define GL_SCENE_MANAGER_HPP

#include <memory>
#include <string>
#include <map>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "gldraw/interface/opengl_object.hpp"

#include "gldraw/frame_buffer.hpp"
#include "gldraw/camera.hpp"
#include "gldraw/camera_controller.hpp"
#include "gldraw/coordinate_transformer.hpp"
#include "gldraw/selection_manager.hpp"

// Forward declarations
namespace quickviz {
class PointCloud;
class SelectionManager;
struct SelectionOptions;
class MultiSelection;
enum class SelectionMode;
}  // namespace quickviz

namespace quickviz {
class SceneManager {
  friend class SelectionManager;  // Allow SelectionManager to access private
                                  // members
 public:
  enum class Mode { k2D, k3D };

  // GPU ID-buffer selection system (no ray casting needed)

  using PreDrawCallback = std::function<void()>;

  SceneManager(const std::string& name, Mode mode = Mode::k3D);
  ~SceneManager();

  // do not allow copy
  SceneManager(const SceneManager&) = delete;
  SceneManager& operator=(const SceneManager&) = delete;

  // public methods
  Mode GetMode() const { return mode_; }

  void SetBackgroundColor(float r, float g, float b, float a);
  void SetClippingPlanes(float z_near, float z_far);

  void AddOpenGLObject(const std::string& name,
                       std::unique_ptr<OpenGlObject> object);
  void RemoveOpenGLObject(const std::string& name);
  OpenGlObject* GetOpenGLObject(const std::string& name);
  void ClearOpenGLObjects();

  /**
   * @brief Set a callback to be called before drawing the scene
   *
   * This callback will be called in the main thread before any OpenGL objects
   * are drawn. It can be used to update scene data in a thread-safe manner.
   *
   * @param callback The callback function to be called
   */
  void SetPreDrawCallback(PreDrawCallback callback) {
    pre_draw_callback_ = std::move(callback);
  }

  /**
   * @brief Enable or disable coordinate system transformation
   *
   * When enabled, the scene will use the standard coordinate system (Z-up)
   * and transform it to OpenGL's coordinate system (Y-up) for rendering.
   *
   * @param enable Whether to enable the transformation
   */
  void EnableCoordinateSystemTransformation(bool enable) {
    use_coord_transform_ = enable;
  }

  /**
   * @brief Check if coordinate system transformation is enabled
   *
   * @return true if enabled, false otherwise
   */
  bool IsCoordinateSystemTransformationEnabled() const {
    return use_coord_transform_;
  }

  /**
   * @brief Render scene to framebuffer at specified dimensions
   * @param width Framebuffer width
   * @param height Framebuffer height
   */
  void RenderToFramebuffer(float width, float height);

  /**
   * @brief Get the framebuffer texture ID for ImGui rendering
   * @return OpenGL texture ID
   */
  uint32_t GetFramebufferTexture() const;

  /**
   * @brief Get camera controller for input handling
   * @return Pointer to camera controller
   */
  CameraController* GetCameraController() const {
    return camera_controller_.get();
  }

  // Camera access for selection tools
  Camera* GetCamera() const { return camera_.get(); }
  const glm::mat4& GetProjectionMatrix() const { return projection_; }
  const glm::mat4& GetViewMatrix() const { return view_; }
  const glm::mat4& GetCoordinateTransform() const { return coord_transform_; }

  // === Interactive Selection System ===

  /**
   * @brief Get the selection system for interactive selection operations
   * @return Reference to selection manager
   */
  SelectionManager& GetSelection() { return *selection_manager_; }
  const SelectionManager& GetSelection() const { return *selection_manager_; }

  /**
   * @brief Main selection method - select at screen coordinates
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param options Selection options (radius, mode, filters)
   * @return Selection result
   */
  SelectionResult Select(float screen_x, float screen_y,
                         const SelectionOptions& options = {});

  /**
   * @brief Multi-selection support - add to current selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param options Selection options
   * @return true if something was selected and added
   */
  bool AddToSelection(float screen_x, float screen_y,
                      const SelectionOptions& options = {});

  /**
   * @brief Get current multi-selection state
   * @return Multi-selection object with all selected items
   */
  const MultiSelection& GetMultiSelection() const;

  /**
   * @brief Enable or disable selection functionality
   * @param enabled If false, selection operations will return empty results
   *                and no ID buffer rendering will occur
   */
  void SetSelectionEnabled(bool enabled) { selection_enabled_ = enabled; }

  /**
   * @brief Check if selection functionality is enabled
   * @return true if selection is enabled, false otherwise
   */
  bool IsSelectionEnabled() const { return selection_enabled_; }

 protected:
  void UpdateView(const glm::mat4& projection, const glm::mat4& view);

  glm::vec4 background_color_ = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

  std::string name_;
  Mode mode_ = Mode::k3D;

  // Main on-screen framebuffer
  std::unique_ptr<FrameBuffer> frame_buffer_;

  // Use std::map instead of unordered_map to ensure consistent iteration order
  // This is critical for GPU selection ID assignment consistency
  std::map<std::string, std::unique_ptr<OpenGlObject>> drawable_objects_;

  // Camera and view/projection matrices
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<CameraController> camera_controller_;
  glm::mat4 projection_ = glm::mat4(1.0f);
  glm::mat4 view_ = glm::mat4(1.0f);
  float z_near_ = 0.1f;
  float z_far_ = 1000.0f;

  // Coordinate system transformation
  bool use_coord_transform_ = true;
  glm::mat4 coord_transform_ = glm::mat4(1.0f);

  // Pre-draw callback
  PreDrawCallback pre_draw_callback_;

  // Interactive selection system
  std::unique_ptr<SelectionManager> selection_manager_;
  bool selection_enabled_ = true;
};
}  // namespace quickviz

#endif  // GL_SCENE_MANAGER_HPP
