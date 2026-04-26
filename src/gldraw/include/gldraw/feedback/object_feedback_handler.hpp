/*
 * @file object_feedback_handler.hpp  
 * @date Sept 2, 2025
 * @brief Specialized feedback handler for non-point-cloud objects
 *
 * This handler provides visual feedback for meshes, geometric primitives,
 * and other OpenGL objects through overlay rendering techniques like
 * outlines, surface overlays, and wireframe rendering.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_OBJECT_FEEDBACK_HANDLER_HPP
#define QUICKVIZ_OBJECT_FEEDBACK_HANDLER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <glm/glm.hpp>
#include <GL/gl.h>

#include "gldraw/feedback/visual_feedback_system.hpp"

// Forward declarations
namespace quickviz {
class SceneManager;
class OpenGlObject;
}

namespace quickviz {

/**
 * @brief Specialized feedback handler for non-point-cloud objects
 * 
 * Handles visual feedback for meshes, geometric primitives, and other
 * OpenGL objects using overlay rendering techniques. This complements
 * the point cloud feedback system by providing whole-object highlighting.
 */
class ObjectFeedbackHandler {
public:
  /**
   * @brief Constructor  
   * @param scene_manager Pointer to scene manager for object access
   */
  explicit ObjectFeedbackHandler(SceneManager* scene_manager);
  
  /**
   * @brief Destructor
   */
  ~ObjectFeedbackHandler();

  /**
   * @brief Show feedback for an object
   * @param object_name Name of the object in the scene
   * @param type Type of feedback to show
   * @param style Style of feedback rendering
   * @param theme Theme configuration for colors and appearance
   */
  void ShowObjectFeedback(const std::string& object_name, 
                         FeedbackType type,
                         FeedbackStyle style,
                         const FeedbackTheme& theme);
  
  /**
   * @brief Remove feedback from an object
   * @param object_name Name of the object
   * @param type Type of feedback to remove
   */
  void RemoveObjectFeedback(const std::string& object_name, FeedbackType type);
  
  /**
   * @brief Clear all feedback of a specific type from all objects
   * @param type Type of feedback to clear  
   */
  void ClearFeedback(FeedbackType type);
  
  /**
   * @brief Clear all feedback from all objects
   */
  void ClearAllFeedback();
  
  /**
   * @brief Update animation states
   * @param delta_time Time since last frame in seconds
   */
  void Update(float delta_time);
  
  /**
   * @brief Render all active feedback overlays
   * @param projection Projection matrix
   * @param view View matrix
   */
  void RenderFeedback(const glm::mat4& projection, const glm::mat4& view);
  
  /**
   * @brief Check if object has specific feedback active
   * @param object_name Name of the object
   * @param type Type of feedback to check
   * @return True if feedback is active
   */
  bool HasFeedback(const std::string& object_name, FeedbackType type) const;
  
  /**
   * @brief Get all objects with active feedback of a specific type
   * @param type Type of feedback to query
   * @return Vector of object names
   */
  std::vector<std::string> GetObjectsWithFeedback(FeedbackType type) const;

private:
  /**
   * @brief State information for active object feedback
   */
  struct ObjectFeedbackState {
    std::string object_name;
    FeedbackType type;
    FeedbackStyle style;
    glm::vec4 color;
    float intensity;
    bool animated;
    float animation_time;
    float animation_speed;
    
    // Animation parameters
    float pulse_phase;        // For pulse animation
    float fade_progress;      // For fade in/out animation
    
    ObjectFeedbackState() : intensity(1.0f), animated(false), animation_time(0.0f), 
                           animation_speed(1.0f), pulse_phase(0.0f), fade_progress(1.0f) {}
  };
  
  SceneManager* scene_manager_;
  std::vector<ObjectFeedbackState> active_feedback_;
  
  // OpenGL resources for rendering
  GLuint outline_shader_program_;
  GLuint overlay_shader_program_;
  GLuint wireframe_shader_program_;
  
  bool shaders_initialized_;
  
  /**
   * @brief Initialize OpenGL shaders for feedback rendering
   */
  void InitializeShaders();
  
  /**
   * @brief Clean up OpenGL resources
   */
  void CleanupShaders();
  
  /**
   * @brief Get color for feedback type from theme
   * @param type Type of feedback
   * @param theme Theme configuration
   * @return Color for this feedback type
   */
  glm::vec4 GetColorForFeedbackType(FeedbackType type, const FeedbackTheme& theme) const;
  
  /**
   * @brief Update animation state for a feedback instance
   * @param state Reference to feedback state to update
   * @param delta_time Time since last frame
   */
  void UpdateAnimation(ObjectFeedbackState& state, float delta_time);
  
  /**
   * @brief Calculate current intensity based on animation state
   * @param state Feedback state with animation information
   * @return Current intensity multiplier (0.0 to 1.0)
   */
  float CalculateAnimatedIntensity(const ObjectFeedbackState& state) const;
  
  // Rendering methods for different styles
  void RenderOutline(const ObjectFeedbackState& state, OpenGlObject* object,
                    const glm::mat4& projection, const glm::mat4& view);
  void RenderSurfaceOverlay(const ObjectFeedbackState& state, OpenGlObject* object,
                           const glm::mat4& projection, const glm::mat4& view);
  void RenderWireframe(const ObjectFeedbackState& state, OpenGlObject* object,
                      const glm::mat4& projection, const glm::mat4& view);
  void RenderBoundingBox(const ObjectFeedbackState& state, OpenGlObject* object,
                        const glm::mat4& projection, const glm::mat4& view);
  
  /**
   * @brief Find active feedback state for an object and type
   * @param object_name Name of the object
   * @param type Type of feedback
   * @return Iterator to feedback state, or end() if not found
   */
  std::vector<ObjectFeedbackState>::iterator FindFeedbackState(const std::string& object_name, 
                                                              FeedbackType type);
  
  /**
   * @brief Find active feedback state for an object and type (const version)
   */
  std::vector<ObjectFeedbackState>::const_iterator FindFeedbackState(const std::string& object_name, 
                                                                    FeedbackType type) const;
};

} // namespace quickviz

#endif // QUICKVIZ_OBJECT_FEEDBACK_HANDLER_HPP