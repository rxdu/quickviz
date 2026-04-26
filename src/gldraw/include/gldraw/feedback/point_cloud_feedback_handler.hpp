/*
 * @file point_cloud_feedback_handler.hpp
 * @date Sept 2, 2025
 * @brief Specialized feedback handler for point clouds using LayerManager
 *
 * This handler leverages the existing LayerManager system to provide
 * visual feedback for point clouds while preserving the 60-100x performance
 * optimizations of the index-based rendering approach.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POINT_CLOUD_FEEDBACK_HANDLER_HPP
#define QUICKVIZ_POINT_CLOUD_FEEDBACK_HANDLER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <glm/glm.hpp>

#include "gldraw/feedback/visual_feedback_system.hpp"
#include "gldraw/renderable/details/point_layer_manager.hpp"

// Forward declarations
namespace quickviz {
class PointCloud;
}

namespace quickviz {

/**
 * @brief Specialized feedback handler for point clouds
 * 
 * Uses the existing LayerManager system to provide efficient visual feedback
 * for point clouds. This preserves all existing performance optimizations
 * while providing a clean interface for the VisualFeedbackSystem.
 */
class PointCloudFeedbackHandler {
public:
  /**
   * @brief Constructor
   */
  PointCloudFeedbackHandler();
  
  /**
   * @brief Destructor
   */
  ~PointCloudFeedbackHandler() = default;

  /**
   * @brief Show feedback for specific points in a point cloud
   * @param point_cloud Pointer to the point cloud object
   * @param point_indices Indices of points to highlight
   * @param type Type of feedback to show
   * @param theme Theme configuration for colors and styles
   */
  void ShowPointFeedback(PointCloud* point_cloud, 
                        const std::vector<size_t>& point_indices,
                        FeedbackType type,
                        const FeedbackTheme& theme);
  
  /**
   * @brief Remove feedback from a point cloud
   * @param point_cloud Pointer to the point cloud object
   * @param type Type of feedback to remove
   */
  void RemovePointFeedback(PointCloud* point_cloud, FeedbackType type);
  
  /**
   * @brief Clear all feedback of a specific type from all point clouds
   * @param type Type of feedback to clear
   */
  void ClearFeedback(FeedbackType type);
  
  /**
   * @brief Clear all feedback from all point clouds
   */
  void ClearAllFeedback();
  
  /**
   * @brief Update animations (if any point cloud feedback supports animation)
   * @param delta_time Time since last frame in seconds
   */
  void Update(float delta_time);
  
  /**
   * @brief Check if point cloud has specific feedback active
   * @param point_cloud Pointer to the point cloud object  
   * @param type Type of feedback to check
   * @return True if feedback is active
   */
  bool HasFeedback(PointCloud* point_cloud, FeedbackType type) const;

private:
  /**
   * @brief Configuration for feedback layers
   */
  struct FeedbackLayerSpec {
    std::string layer_name_prefix;     // e.g., "__feedback_hover_"
    int priority;                      // Layer priority
    PointLayer::HighlightMode highlight_mode;
    
    // Default layer properties (can be overridden by theme)
    glm::vec3 default_color;
    float default_size_multiplier;
  };
  
  // Predefined layer specifications for each feedback type
  static const std::unordered_map<FeedbackType, FeedbackLayerSpec> FEEDBACK_SPECS;
  
  /**
   * @brief Generate unique layer name for a specific point cloud and feedback type
   * @param point_cloud Pointer to the point cloud
   * @param type Type of feedback
   * @return Unique layer name
   */
  std::string GenerateLayerName(PointCloud* point_cloud, FeedbackType type) const;
  
  /**
   * @brief Get or create feedback layer for a point cloud
   * @param point_cloud Pointer to the point cloud
   * @param type Type of feedback
   * @param theme Theme for styling the layer
   * @return Shared pointer to the feedback layer
   */
  std::shared_ptr<PointLayer> GetOrCreateFeedbackLayer(PointCloud* point_cloud,
                                                       FeedbackType type, 
                                                       const FeedbackTheme& theme);
  
  /**
   * @brief Apply theme settings to a point layer
   * @param layer Pointer to the layer
   * @param type Type of feedback (determines which theme colors to use)
   * @param theme Theme configuration
   */
  void ApplyThemeToLayer(std::shared_ptr<PointLayer> layer, 
                        FeedbackType type, 
                        const FeedbackTheme& theme);
  
  // State tracking for active feedback layers
  struct ActiveFeedback {
    PointCloud* point_cloud;
    FeedbackType type;
    std::string layer_name;
  };
  
  std::vector<ActiveFeedback> active_feedback_;
};

} // namespace quickviz

#endif // QUICKVIZ_POINT_CLOUD_FEEDBACK_HANDLER_HPP