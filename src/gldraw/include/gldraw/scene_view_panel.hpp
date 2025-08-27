/*
 * scene_view_panel.hpp
 *
 * Created on August 27, 2025
 * Description: ImGui integration panel for GlSceneManager
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef SCENE_VIEW_PANEL_HPP
#define SCENE_VIEW_PANEL_HPP

#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "imview/panel.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/camera.hpp"
#include "gldraw/camera_controller.hpp"

// Forward declaration
namespace quickviz {
class PointCloud;
}

namespace quickviz {

/**
 * @brief ImGui panel wrapper for GlSceneManager
 * 
 * Separates UI integration from rendering backend by wrapping GlSceneManager
 * in an ImGui Panel. Handles ImGui window management and input processing
 * while delegating rendering to the scene manager.
 */
class SceneViewPanel : public Panel {
 public:
  /**
   * @brief Constructor
   * @param name Panel name for ImGui window
   * @param mode 2D or 3D rendering mode
   */
  SceneViewPanel(const std::string& name, 
                 GlSceneManager::Mode mode = GlSceneManager::Mode::k3D);
  
  virtual ~SceneViewPanel() = default;

  // Panel interface
  void Draw() override;
  
  /**
   * @brief Render content without Begin/End calls (for use within existing ImGui context)
   */
  void RenderInsideWindow();

  /**
   * @brief Get the underlying scene manager
   * @return Pointer to GlSceneManager for object management
   */
  GlSceneManager* GetSceneManager() const { return scene_manager_.get(); }

  /**
   * @brief Set whether to show rendering info overlay
   * @param show True to display FPS and frame time
   */
  void SetShowRenderingInfo(bool show);

  /**
   * @brief Set background color for the 3D view
   * @param r Red component (0-1)
   * @param g Green component (0-1)  
   * @param b Blue component (0-1)
   * @param a Alpha component (0-1)
   */
  void SetBackgroundColor(float r, float g, float b, float a);

  // Delegate common GlSceneManager methods
  GlSceneManager::Mode GetMode() const;
  void SetClippingPlanes(float z_near, float z_far);
  
  void AddOpenGLObject(const std::string& name, std::unique_ptr<OpenGlObject> object);
  void RemoveOpenGLObject(const std::string& name);
  OpenGlObject* GetOpenGLObject(const std::string& name);
  void ClearOpenGLObjects();
  
  void SetPreDrawCallback(GlSceneManager::PreDrawCallback callback);
  void EnableCoordinateSystemTransformation(bool enable);
  bool IsCoordinateSystemTransformationEnabled() const;
  
  // Camera access
  CameraController* GetCameraController() const;
  Camera* GetCamera() const;
  const glm::mat4& GetProjectionMatrix() const;
  const glm::mat4& GetViewMatrix() const;
  const glm::mat4& GetCoordinateTransform() const;
  GlSceneManager::MouseRay GetMouseRayInWorldSpace(float mouse_x, float mouse_y, 
                                                   float window_width, float window_height) const;
  
  // GPU ID-buffer picking support
  size_t PickPointAtPixel(int x, int y, const std::string& point_cloud_name = "");
  size_t PickPointAtPixelWithRadius(int x, int y, int radius = 2, const std::string& point_cloud_name = "");
  
  // Point cloud selection
  void SetActivePointCloud(PointCloud* point_cloud);
  PointCloud* GetActivePointCloud() const;
  
  // Point selection operations
  bool SelectPointAt(float screen_x, float screen_y, int radius = 3);
  bool AddPointAt(float screen_x, float screen_y, int radius = 3);
  bool TogglePointAt(float screen_x, float screen_y, int radius = 3);
  void ClearPointSelection();
  const std::vector<size_t>& GetSelectedPointIndices() const;
  size_t GetSelectedPointCount() const;
  glm::vec3 GetSelectionCentroid() const;
  std::pair<glm::vec3, glm::vec3> GetSelectionBounds() const;
  
  // Selection visualization
  void SetSelectionVisualization(const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f),
                                float size_multiplier = 1.5f,
                                const std::string& layer_name = "selection");
  void SetSelectionVisualizationEnabled(bool enabled);
  void SetPointSelectionCallback(GlSceneManager::PointSelectionCallback callback);
  
  // Object selection
  void SelectObjectAt(float screen_x, float screen_y);
  const std::string& GetSelectedObjectName() const;
  void ClearObjectSelection();
  void SetObjectHighlight(const std::string& name, bool highlighted);
  void SetObjectSelectionCallback(GlSceneManager::ObjectSelectionCallback callback);
  
  // Panel method delegation (for backward compatibility with tests that used GlSceneManager)
  void SetAutoLayout(bool auto_layout) { Panel::SetAutoLayout(auto_layout); }
  void SetNoTitleBar(bool no_title_bar) { Panel::SetNoTitleBar(no_title_bar); }
  void SetFlexGrow(float flex_grow) { Panel::SetFlexGrow(flex_grow); }
  void SetFlexShrink(float flex_shrink) { Panel::SetFlexShrink(flex_shrink); }

 protected:
  /**
   * @brief Handle ImGui input events and forward to scene manager
   */
  void HandleInput();
  
  /**
   * @brief Render FPS overlay if enabled
   */
  void RenderInfoOverlay();
  
 private:
  std::unique_ptr<GlSceneManager> scene_manager_;
  
  // UI state
  bool show_rendering_info_ = false;
};

} // namespace quickviz

#endif // SCENE_VIEW_PANEL_HPP