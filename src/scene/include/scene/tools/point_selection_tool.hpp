/**
 * @file point_selection_tool.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-02
 * @brief Point selection tool for interactive point cloud editing
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POINT_SELECTION_TOOL_HPP
#define QUICKVIZ_POINT_SELECTION_TOOL_HPP

#include <functional>
#include <string>

#include <glm/glm.hpp>
#include "scene/tools/interaction_tool.hpp"
#include "scene/selection_manager.hpp"

namespace quickviz {

/**
 * @brief Interactive point selection tool for point clouds
 * 
 * Provides a complete user interaction experience for selecting points
 * in point clouds. Builds on top of SelectionManager infrastructure
 * to provide:
 * - Click-to-select points with visual feedback
 * - Multi-selection with Ctrl+Click
 * - Selection radius adjustment
 * - Target point cloud filtering
 * - Hover feedback and selection preview
 * 
 * Usage:
 * ```cpp
 * auto tool = std::make_shared<PointSelectionTool>("point_select", scene_manager);
 * tool->SetSelectionCallback([](const SelectionResult& result) {
 *     if (auto point_sel = std::get_if<PointSelection>(&result)) {
 *         std::cout << "Selected point " << point_sel->point_index << std::endl;
 *     }
 * });
 * tool_manager.RegisterTool(tool);
 * tool_manager.ActivateTool("point_select");
 * ```
 */
class PointSelectionTool : public InteractionTool {
public:
    /**
     * @brief Selection behavior mode
     */
    enum class SelectionMode {
        kSingle,        // Replace current selection
        kAdd,          // Add to current selection (Ctrl+Click behavior)
        kToggle,       // Toggle selection state (Ctrl+Click on selected)
        kSubtract      // Remove from current selection (Alt+Click)
    };

    /**
     * @brief Visual feedback options
     */
    struct VisualFeedback {
        bool show_hover_highlight = true;    // Highlight point under cursor
        bool show_selection_radius = false;  // Draw selection radius circle
        bool show_selection_count = true;    // Show selection count overlay
        glm::vec3 hover_color = glm::vec3(1.0f, 1.0f, 0.0f);     // Yellow hover
        glm::vec3 selection_color = glm::vec3(1.0f, 0.5f, 0.0f);  // Orange selection
        float hover_size_multiplier = 1.3f;  // Hover point size multiplier
        float selection_size_multiplier = 1.2f; // Selected point size multiplier
    };

    /**
     * @brief Point selection callback function
     * Called whenever a point is selected/deselected
     */
    using SelectionCallback = std::function<void(const SelectionResult&, const MultiSelection&)>;
    
    /**
     * @brief Hover feedback callback function  
     * Called when hovering over different points
     */
    using HoverCallback = std::function<void(const SelectionResult&)>;

    PointSelectionTool(const std::string& name, SceneManager* scene_manager);
    virtual ~PointSelectionTool() = default;

    // === InteractionTool Interface ===
    CursorType GetCursorType() const override { return CursorType::kCrosshair; }
    void OnRender(const glm::mat4& projection, const glm::mat4& view) override;

    // === Configuration ===
    
    /**
     * @brief Set default selection mode (can be overridden by modifiers)
     * @param mode Selection mode to use by default
     */
    void SetSelectionMode(SelectionMode mode) { default_mode_ = mode; }
    SelectionMode GetSelectionMode() const { return default_mode_; }
    
    /**
     * @brief Set selection radius in pixels
     * @param radius Selection tolerance in screen pixels
     */
    void SetSelectionRadius(int radius) { selection_radius_ = radius; }
    int GetSelectionRadius() const { return selection_radius_; }
    
    /**
     * @brief Filter selection to specific point cloud (empty = select from any)
     * @param name Point cloud name, or empty string for any point cloud
     */
    void SetTargetPointCloud(const std::string& name) { target_point_cloud_ = name; }
    const std::string& GetTargetPointCloud() const { return target_point_cloud_; }
    
    /**
     * @brief Configure visual feedback options
     * @param feedback Visual feedback configuration
     */
    void SetVisualFeedback(const VisualFeedback& feedback) { visual_feedback_ = feedback; }
    const VisualFeedback& GetVisualFeedback() const { return visual_feedback_; }

    // === Callbacks ===
    
    /**
     * @brief Set callback for selection changes
     * @param callback Function called when selection changes
     */
    void SetSelectionCallback(SelectionCallback callback) { selection_callback_ = callback; }
    
    /**
     * @brief Set callback for hover feedback
     * @param callback Function called when hovering over points
     */
    void SetHoverCallback(HoverCallback callback) { hover_callback_ = callback; }

    // === Selection State Access ===
    
    /**
     * @brief Get current point selection
     * @return Current selection result
     */
    const SelectionResult& GetCurrentSelection() const;
    
    /**
     * @brief Get current multi-selection
     * @return Multi-selection with all selected points
     */
    const MultiSelection& GetMultiSelection() const;
    
    /**
     * @brief Get selection count
     * @return Number of currently selected points
     */
    size_t GetSelectionCount() const;
    
    /**
     * @brief Clear all selections
     */
    void ClearSelection();

    // === Manual Selection API ===
    
    /**
     * @brief Programmatically select point at screen coordinates
     * @param screen_x Screen X coordinate
     * @param screen_y Screen Y coordinate  
     * @param mode Selection mode (uses default if not specified)
     * @return true if point was found and selected
     */
    bool SelectPointAt(float screen_x, float screen_y, SelectionMode mode = SelectionMode::kSingle);
    
    /**
     * @brief Programmatically select specific point by index
     * @param point_cloud_name Name of point cloud containing point
     * @param point_index Index within point cloud
     * @param mode Selection mode
     * @return true if point was found and selected
     */
    bool SelectPointByIndex(const std::string& point_cloud_name, size_t point_index, SelectionMode mode = SelectionMode::kSingle);

protected:
    // === InteractionTool Overrides ===
    void DoActivate() override;
    void DoDeactivate() override;
    bool OnMouseEvent(const InputEvent& event) override;
    bool OnKeyboardEvent(const InputEvent& event) override;

private:
    // === Internal Event Handling ===
    bool HandleMouseClick(const InputEvent& event);
    bool HandleMouseMove(const InputEvent& event);
    void UpdateHoverFeedback(float screen_x, float screen_y);
    
    // === Selection Logic ===
    SelectionMode DetermineSelectionMode(const InputEvent& event) const;
    void PerformSelection(float screen_x, float screen_y, SelectionMode mode);
    void UpdateVisualFeedback();
    void UpdateHoverLayer();
    void ClearSelectionLayers();
    void NotifySelectionChanged(const SelectionResult& result);
    void NotifyHoverChanged(const SelectionResult& result);

    // === Visual Feedback Rendering ===
    void RenderHoverFeedback(const glm::mat4& projection, const glm::mat4& view);
    void RenderSelectionRadius(const glm::mat4& projection, const glm::mat4& view);
    void RenderSelectionCount(const glm::mat4& projection, const glm::mat4& view);

    // === Configuration ===
    SelectionMode default_mode_ = SelectionMode::kSingle;
    int selection_radius_ = 3;  // pixels
    std::string target_point_cloud_;  // empty = any point cloud
    VisualFeedback visual_feedback_;

    // === Callbacks ===
    SelectionCallback selection_callback_;
    HoverCallback hover_callback_;

    // === Internal State ===
    SelectionResult current_hover_;  // Point currently being hovered
    glm::vec2 last_mouse_pos_;      // Last mouse position for hover tracking
    bool mouse_moved_since_click_ = false;  // Prevent accidental selection on drag
    
    // === Visual Feedback State ===
    struct HoverState {
        bool active = false;
        std::string point_cloud_name;
        size_t point_index = SIZE_MAX;
        glm::vec3 world_position;
        glm::vec2 screen_position;
    } hover_state_;
};

/**
 * @brief Factory for creating common point selection tool configurations
 */
class PointSelectionToolFactory {
public:
    /**
     * @brief Create standard point selection tool
     * @param scene_manager Scene to operate on
     * @param name Tool name (default: "point_select")
     */
    static std::shared_ptr<PointSelectionTool> CreateStandard(
        SceneManager* scene_manager, 
        const std::string& name = "point_select");
    
    /**
     * @brief Create point selection tool for specific point cloud
     * @param scene_manager Scene to operate on
     * @param point_cloud_name Target point cloud name
     * @param name Tool name
     */
    static std::shared_ptr<PointSelectionTool> CreateForPointCloud(
        SceneManager* scene_manager,
        const std::string& point_cloud_name,
        const std::string& name = "point_select");
    
    /**
     * @brief Create point selection tool with custom visual feedback
     * @param scene_manager Scene to operate on
     * @param feedback Visual feedback configuration
     * @param name Tool name (default: "point_select")
     */
    static std::shared_ptr<PointSelectionTool> CreateWithVisualFeedback(
        SceneManager* scene_manager,
        const PointSelectionTool::VisualFeedback& feedback,
        const std::string& name = "point_select");
};

} // namespace quickviz

#endif // QUICKVIZ_POINT_SELECTION_TOOL_HPP