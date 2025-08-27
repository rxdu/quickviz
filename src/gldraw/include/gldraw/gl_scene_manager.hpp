/*
 * gl_scene_manager.hpp
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
#include <unordered_map>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "gldraw/interface/opengl_object.hpp"

#include "gldraw/frame_buffer.hpp"
#include "gldraw/camera.hpp"
#include "gldraw/camera_controller.hpp"
#include "gldraw/coordinate_system_transformer.hpp"

// Forward declarations
namespace quickviz {
class PointCloud;
}

namespace quickviz {
class GlSceneManager {
 public:
  enum class Mode { k2D, k3D };

  // Mouse ray casting
  struct MouseRay {
    glm::vec3 origin;
    glm::vec3 direction;
    bool valid = false;
  };

  using PreDrawCallback = std::function<void()>;

  GlSceneManager(const std::string& name, Mode mode = Mode::k3D);
  ~GlSceneManager();

  // do not allow copy
  GlSceneManager(const GlSceneManager&) = delete;
  GlSceneManager& operator=(const GlSceneManager&) = delete;

  // public methods
  Mode GetMode() const { return mode_; }

  void SetShowRenderingInfo(bool show);
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
  CameraController* GetCameraController() const { return camera_controller_.get(); }
  
  // Camera access for selection tools
  Camera* GetCamera() const { return camera_.get(); }
  const glm::mat4& GetProjectionMatrix() const { return projection_; }
  const glm::mat4& GetViewMatrix() const { return view_; }
  const glm::mat4& GetCoordinateTransform() const { return coord_transform_; }

  MouseRay GetMouseRayInWorldSpace(float mouse_x, float mouse_y, 
                                   float window_width, float window_height) const;

  // GPU ID-buffer picking support
  size_t PickPointAtPixel(int x, int y, const std::string& point_cloud_name = "");
  size_t PickPointAtPixelWithRadius(int x, int y, int radius = 2, const std::string& point_cloud_name = "");

  // === Point Selection API ===
  
  /**
   * @brief Set the active point cloud for selection operations
   * @param point_cloud Point cloud to enable selection on (must be already added to scene)
   */
  void SetActivePointCloud(PointCloud* point_cloud);
  
  /**
   * @brief Get the currently active point cloud
   * @return Pointer to active point cloud, or nullptr if none
   */
  PointCloud* GetActivePointCloud() const { return active_point_cloud_; }
  
  /**
   * @brief Select a single point (replace current selection)
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate  
   * @param radius Picking radius in pixels
   * @return true if a point was selected
   */
  bool SelectPointAt(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Add a point to current selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param radius Picking radius in pixels
   * @return true if a point was selected and added
   */
  bool AddPointAt(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Toggle point selection state
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param radius Picking radius in pixels
   * @return true if a point was found and toggled
   */
  bool TogglePointAt(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Clear all selected points
   */
  void ClearPointSelection();

  // === Selection Data Access ===
  
  /**
   * @brief Get indices of selected points
   * @return Vector of point indices in the active point cloud
   */
  const std::vector<size_t>& GetSelectedPointIndices() const { return selected_point_indices_; }
  
  /**
   * @brief Get number of selected points
   */
  size_t GetSelectedPointCount() const { return selected_point_indices_.size(); }

  /**
   * @brief Get 3D positions of selected points
   * @return Vector of 3D positions suitable for external processing
   */
  std::vector<glm::vec3> GetSelectedPoints() const;
  
  /**
   * @brief Get colors of selected points (if available)
   * @return Vector of colors, or empty if point cloud has no color data
   */
  std::vector<glm::vec3> GetSelectedPointColors() const;
  
  /**
   * @brief Get centroid of selected points
   * @return Centroid position, or zero vector if no selection
   */
  glm::vec3 GetSelectionCentroid() const;
  
  /**
   * @brief Get bounding box of selected points  
   * @return {min_bounds, max_bounds} or {{0,0,0}, {0,0,0}} if no selection
   */
  std::pair<glm::vec3, glm::vec3> GetSelectionBounds() const;

  // === Selection Visualization ===
  
  /**
   * @brief Configure selection visualization
   * @param color Highlight color (default: yellow)
   * @param size_multiplier Point size multiplier (default: 1.5x)
   * @param layer_name Layer name for highlights (default: "selection")
   */
  void SetSelectionVisualization(const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f),
                                float size_multiplier = 1.5f,
                                const std::string& layer_name = "selection");
  
  /**
   * @brief Enable/disable selection visualization
   * @param enabled Whether to show selection highlights
   */
  void SetSelectionVisualizationEnabled(bool enabled);

  // === Selection Callbacks ===
  
  /**
   * @brief Callback type for selection changes
   * @param indices Currently selected point indices
   */
  using PointSelectionCallback = std::function<void(const std::vector<size_t>&)>;
  
  /**
   * @brief Set callback for selection changes
   * @param callback Function to call when selection changes
   */
  void SetPointSelectionCallback(PointSelectionCallback callback) {
    point_selection_callback_ = callback;
  }
  
  // === Object Selection API ===
  
  /**
   * @brief Select an object at screen coordinates
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @return Name of selected object, or empty string if none
   */
  std::string SelectObjectAt(float screen_x, float screen_y);
  
  /**
   * @brief Get currently selected object name
   * @return Name of selected object, or empty string if none
   */
  const std::string& GetSelectedObjectName() const { return selected_object_name_; }
  
  /**
   * @brief Clear object selection
   */
  void ClearObjectSelection();
  
  /**
   * @brief Highlight an object (visual feedback)
   * @param name Object name
   * @param highlighted Whether to highlight
   */
  void SetObjectHighlight(const std::string& name, bool highlighted);
  
  /**
   * @brief Callback type for object selection changes
   * @param name Name of selected object (empty if none)
   */
  using ObjectSelectionCallback = std::function<void(const std::string&)>;
  
  /**
   * @brief Set callback for object selection changes
   * @param callback Function to call when object selection changes
   */
  void SetObjectSelectionCallback(ObjectSelectionCallback callback) {
    object_selection_callback_ = callback;
  }
  
 private:
  void RenderIdBuffer();
  size_t ReadPixelId(int x, int y);
  
  // Point selection helper methods
  void UpdateSelectionVisualization();
  void NotifySelectionChanged();
  bool IsPointSelected(size_t point_index) const;
  void RemoveFromSelection(size_t point_index);
  void AddToSelection(size_t point_index);

 protected:
  void UpdateView(const glm::mat4& projection, const glm::mat4& view);

  glm::vec4 background_color_ = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

  std::string name_;
  Mode mode_ = Mode::k3D;
  std::unique_ptr<FrameBuffer> frame_buffer_;
  std::unique_ptr<FrameBuffer> id_frame_buffer_;  // Off-screen buffer for ID picking
  glm::mat4 projection_ = glm::mat4(1.0f);
  glm::mat4 view_ = glm::mat4(1.0f);
  std::unordered_map<std::string, std::unique_ptr<OpenGlObject>>
      drawable_objects_;

  std::unique_ptr<Camera> camera_;
  std::unique_ptr<CameraController> camera_controller_;
  bool show_rendering_info_ = true;

  // Coordinate system transformation
  bool use_coord_transform_ = true;
  glm::mat4 coord_transform_ = glm::mat4(1.0f);
  float z_near_ = 0.1f;
  float z_far_ = 1000.0f;

  // Pre-draw callback
  PreDrawCallback pre_draw_callback_;
  
  // Point selection state
  PointCloud* active_point_cloud_ = nullptr;
  std::vector<size_t> selected_point_indices_;
  PointSelectionCallback point_selection_callback_;
  
  // Selection visualization settings
  glm::vec3 selection_color_ = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow
  float selection_size_multiplier_ = 1.5f;
  std::string selection_layer_name_ = "selection";
  bool selection_visualization_enabled_ = true;
  
  // Object selection state
  std::string selected_object_name_;
  std::unordered_map<std::string, bool> object_highlights_;
  ObjectSelectionCallback object_selection_callback_;
};
}  // namespace quickviz

#endif  // GL_SCENE_MANAGER_HPP
