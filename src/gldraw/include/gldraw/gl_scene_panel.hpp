/*
 * gl_scene_panel.hpp
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
#include "gldraw/selection_manager.hpp"
#include "core/event/input_event.hpp"
#include "core/event/event_dispatcher.hpp"
#include "core/event/input_mapping.hpp"
#include "gldraw/input/scene_input_handler.hpp"

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
class GlScenePanel : public Panel {
 public:
  /**
   * @brief Constructor
   * @param name Panel name for ImGui window
   * @param mode 2D or 3D rendering mode
   */
  GlScenePanel(const std::string& name,
               GlSceneManager::Mode mode = GlSceneManager::Mode::k3D);

  virtual ~GlScenePanel() = default;

  // Panel interface
  void Draw() override;

  /**
   * @brief Render content without Begin/End calls (for use within existing
   * ImGui context)
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

  void AddOpenGLObject(const std::string& name,
                       std::unique_ptr<OpenGlObject> object);
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

  // === Selection System ===
  /**
   * @brief Enable or disable selection functionality
   * @param enabled If false, selection operations will return empty results
   *                and no ID buffer rendering will occur
   */
  void SetSelectionEnabled(bool enabled);

  /**
   * @brief Check if selection functionality is enabled
   * @return true if selection is enabled, false otherwise
   */
  bool IsSelectionEnabled() const;

  /**
   * @brief Get access to the selection system
   * @return Reference to selection manager for advanced operations
   */
  SelectionManager& GetSelection();
  const SelectionManager& GetSelection() const;

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
   * @brief Multi-selection - add to current selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param options Selection options
   * @return true if something was selected and added
   */
  bool AddToSelection(float screen_x, float screen_y,
                      const SelectionOptions& options = {});

  /**
   * @brief Get current multi-selection
   * @return Multi-selection with all selected items
   */
  const MultiSelection& GetMultiSelection() const;

  /**
   * @brief Clear all selections
   */
  void ClearSelection();

  // === Enhanced Input System ===
  /**
   * @brief Get the input dispatcher for registering custom handlers
   * @return Reference to the input dispatcher
   */
  EventDispatcher& GetInputDispatcher() { return input_dispatcher_; }
  const EventDispatcher& GetInputDispatcher() const { return input_dispatcher_; }

  /**
   * @brief Get the input mapping for configuring key/mouse bindings
   * @return Reference to the input mapping
   */
  InputMapping& GetInputMapping() { return input_mapping_; }
  const InputMapping& GetInputMapping() const { return input_mapping_; }

  /**
   * @brief Enable/disable the enhanced input system
   * @param enabled If true, uses the new input system; false for legacy
   */
  void SetEnhancedInputEnabled(bool enabled) { use_enhanced_input_ = enabled; }
  bool IsEnhancedInputEnabled() const { return use_enhanced_input_; }

 protected:
  // Override Panel input methods for 3D scene interaction
  bool OnInputEvent(const InputEvent& event) override;
  void OnMouseClick(const glm::vec2& position, int button) override;

  /**
   * @brief Handle ImGui input events and forward to scene manager (legacy)
   */
  void HandleInput();

  /**
   * @brief Enhanced input handling using the new input system (legacy)
   */
  void HandleInputEnhanced();

  /**
   * @brief Render FPS overlay if enabled
   * @param content_size Size of the content area
   * @param image_pos Position where the image was rendered
   */
  void RenderInfoOverlay(const ImVec2& content_size, const ImVec2& image_pos);

 private:
  // Helper methods for input conversion
  std::shared_ptr<InputEvent> CreateInputEvent(InputEventType type, int button_or_key = -1);
  ModifierKeys GetCurrentModifiers();

 private:
  std::unique_ptr<GlSceneManager> scene_manager_;

  // UI state
  bool show_rendering_info_ = true;
  
  // Enhanced input system (legacy)
  EventDispatcher input_dispatcher_;
  InputMapping input_mapping_;
  bool use_enhanced_input_ = false;  // Default to legacy for compatibility

  // New imview-based input system
  std::shared_ptr<SceneInputHandler> scene_input_handler_;
};

}  // namespace quickviz

#endif  // SCENE_VIEW_PANEL_HPP