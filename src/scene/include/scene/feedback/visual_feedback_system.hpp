/*
 * @file visual_feedback_system.hpp
 * @date Sept 2, 2025
 * @brief Unified visual feedback system for all object types
 *
 * Provides a coordination layer that manages visual feedback across different
 * object types (point clouds, meshes, primitives) while preserving the
 * performance optimizations of specialized systems like LayerManager.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VISUAL_FEEDBACK_SYSTEM_HPP
#define QUICKVIZ_VISUAL_FEEDBACK_SYSTEM_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

// Forward declarations
namespace quickviz {
class SceneManager;
class SelectionManager;
class PointCloud;
class MultiSelection;
class PointCloudFeedbackHandler;
class ObjectFeedbackHandler;
}

namespace quickviz {

/**
 * @brief Types of visual feedback for different interaction states
 */
enum class FeedbackType {
  kHover,           // Mouse hover highlighting
  kSelection,       // Selected object highlighting  
  kPreSelection,    // Preview before selection confirmation
  kManipulation,    // During drag/transform operations
  kError,           // Invalid operations/constraints
  kSuccess,         // Operation confirmation
  kProgress,        // Long-running operations
};

/**
 * @brief Visual styles for feedback rendering
 */
enum class FeedbackStyle {
  kOutline,         // Object outline/silhouette
  kSurfaceOverlay,  // Semi-transparent surface tint
  kWireframe,       // Wireframe overlay
  kPointHighlight,  // Enhanced point rendering (point clouds only)
  kBoundingBox,     // Bounding box visualization
  kGlow,            // Glow/halo effect
  kPulse,           // Animated pulsing
};

/**
 * @brief Configuration for visual feedback appearance and behavior
 */
struct FeedbackTheme {
  // Colors for different feedback types
  glm::vec4 hover_color = glm::vec4(0.7f, 0.7f, 1.0f, 0.8f);      // Light blue
  glm::vec4 selection_color = glm::vec4(1.0f, 0.6f, 0.0f, 1.0f);  // Orange
  glm::vec4 pre_selection_color = glm::vec4(0.5f, 0.8f, 0.5f, 0.6f); // Light green
  glm::vec4 error_color = glm::vec4(1.0f, 0.2f, 0.2f, 0.8f);      // Red
  glm::vec4 success_color = glm::vec4(0.2f, 1.0f, 0.2f, 0.8f);    // Green
  glm::vec4 manipulation_color = glm::vec4(1.0f, 1.0f, 0.0f, 0.9f); // Yellow
  
  // Point cloud specific settings
  float point_size_multiplier_hover = 1.2f;
  float point_size_multiplier_selection = 1.5f;
  float point_size_multiplier_error = 1.3f;
  
  // Animation parameters
  float pulse_frequency = 2.0f;
  float fade_duration = 0.3f;
  float outline_width = 2.0f;
  
  // Intensity modifiers
  float hover_intensity = 0.7f;
  float selection_intensity = 1.0f;
  float error_intensity = 0.9f;
  
  // Predefined themes
  static FeedbackTheme Default();
  static FeedbackTheme HighContrast();
  static FeedbackTheme Subtle();
  static FeedbackTheme CADStyle();
};

/**
 * @brief Feedback callback for notifying external systems of feedback changes
 */
using FeedbackCallback = std::function<void(const std::string& object_name, FeedbackType type, bool active)>;

/**
 * @brief Main coordination class for visual feedback across all object types
 * 
 * This class provides a unified API for visual feedback while internally
 * delegating to specialized handlers that preserve object-specific optimizations.
 * 
 * Key design principles:
 * - Object-type agnostic API for developers
 * - Preserves existing LayerManager performance for point clouds
 * - Separate overlay system for meshes/primitives
 * - Automatic lifecycle management (hover/selection state tracking)
 * - Animation and theming support
 */
class VisualFeedbackSystem {
public:
  /**
   * @brief Constructor
   * @param scene_manager Pointer to scene manager for object registry access
   */
  explicit VisualFeedbackSystem(SceneManager* scene_manager);
  
  /**
   * @brief Destructor
   */
  ~VisualFeedbackSystem();

  // === Core Feedback API ===
  
  /**
   * @brief Show feedback for an object
   * @param object_name Name of the object in the scene
   * @param type Type of feedback to show
   * @param style Optional style override (uses theme default if not specified)
   */
  void ShowFeedback(const std::string& object_name, FeedbackType type, 
                    FeedbackStyle style = FeedbackStyle::kOutline);
  
  /**
   * @brief Remove specific feedback from an object
   * @param object_name Name of the object
   * @param type Type of feedback to remove
   */
  void RemoveFeedback(const std::string& object_name, FeedbackType type);
  
  /**
   * @brief Clear all feedback of a specific type from all objects
   * @param type Type of feedback to clear
   */
  void ClearFeedback(FeedbackType type);
  
  /**
   * @brief Clear all feedback from all objects
   */
  void ClearAllFeedback();
  
  // === Integration API ===
  
  /**
   * @brief Handle selection change events from SelectionManager
   * @param selection Current selection state
   */
  void OnSelectionChanged(const MultiSelection& selection);
  
  /**
   * @brief Handle object hover events from input system
   * @param object_name Name of object being hovered (empty string for no hover)
   */
  void OnObjectHovered(const std::string& object_name);
  
  /**
   * @brief Handle object unhover events
   */
  void OnObjectUnhovered();
  
  // === Animation and Updates ===
  
  /**
   * @brief Update animation states and render feedback
   * Called once per frame by GlScenePanel
   * @param delta_time Time since last frame in seconds
   */
  void Update(float delta_time);
  
  /**
   * @brief Render non-point-cloud feedback overlays
   * Point cloud feedback is handled by existing LayerManager rendering
   * @param projection Projection matrix
   * @param view View matrix
   */
  void RenderOverlays(const glm::mat4& projection, const glm::mat4& view);
  
  // === Configuration ===
  
  /**
   * @brief Set the visual feedback theme
   * @param theme Theme configuration
   */
  void SetTheme(const FeedbackTheme& theme);
  
  /**
   * @brief Get current theme
   * @return Current theme configuration
   */
  const FeedbackTheme& GetTheme() const { return theme_; }
  
  /**
   * @brief Enable or disable the entire feedback system
   * @param enabled True to enable feedback
   */
  void SetEnabled(bool enabled) { enabled_ = enabled; }
  
  /**
   * @brief Check if feedback system is enabled
   * @return True if enabled
   */
  bool IsEnabled() const { return enabled_; }
  
  /**
   * @brief Register callback for feedback state changes
   * @param callback Function to call when feedback state changes
   */
  void SetFeedbackCallback(FeedbackCallback callback) { feedback_callback_ = std::move(callback); }
  
  // === State Queries ===
  
  /**
   * @brief Check if object has specific type of feedback active
   * @param object_name Name of the object
   * @param type Type of feedback to check
   * @return True if object has this feedback type active
   */
  bool HasFeedback(const std::string& object_name, FeedbackType type) const;
  
  /**
   * @brief Get all objects with active feedback of a specific type
   * @param type Type of feedback to query
   * @return Vector of object names with this feedback type
   */
  std::vector<std::string> GetObjectsWithFeedback(FeedbackType type) const;
  
  /**
   * @brief Get count of active feedback instances
   * @return Total number of active feedback instances across all objects
   */
  size_t GetActiveFeedbackCount() const;

private:
  // === Internal Data ===
  
  SceneManager* scene_manager_;
  FeedbackTheme theme_;
  bool enabled_;
  
  // Specialized handlers for different object types
  std::unique_ptr<PointCloudFeedbackHandler> point_cloud_handler_;
  std::unique_ptr<ObjectFeedbackHandler> object_handler_;
  
  // State tracking
  std::string currently_hovered_object_;
  std::unordered_map<std::string, std::vector<FeedbackType>> active_feedback_;
  
  // Callbacks
  FeedbackCallback feedback_callback_;
  
  // === Internal Methods ===
  
  /**
   * @brief Determine object type and delegate to appropriate handler
   * @param object_name Name of the object
   * @param type Feedback type
   * @param style Feedback style
   * @param show True to show, false to hide
   */
  void DelegateFeedback(const std::string& object_name, FeedbackType type, 
                       FeedbackStyle style, bool show);
  
  /**
   * @brief Update internal state tracking
   * @param object_name Name of the object
   * @param type Feedback type
   * @param active True if feedback is now active
   */
  void UpdateFeedbackState(const std::string& object_name, FeedbackType type, bool active);
  
  /**
   * @brief Get object from scene manager with error handling
   * @param object_name Name of the object
   * @return Pointer to object or nullptr if not found
   */
  class OpenGlObject* GetObject(const std::string& object_name) const;
  
  /**
   * @brief Check if object is a point cloud
   * @param object Pointer to object
   * @return True if object is a PointCloud
   */
  bool IsPointCloud(class OpenGlObject* object) const;
  
  /**
   * @brief Extract point indices for point cloud feedback from selection state
   * @param object_name Name of the point cloud object
   * @return Vector of point indices to highlight
   */
  std::vector<size_t> GetPointIndicesForFeedback(const std::string& object_name) const;
};

} // namespace quickviz

#endif // QUICKVIZ_VISUAL_FEEDBACK_SYSTEM_HPP