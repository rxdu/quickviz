/**
 * @file point_selection_tool.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-02
 * @brief Implementation of point selection tool
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/tools/point_selection_tool.hpp"
#include "scene/scene_manager.hpp"
#include "scene/renderable/point_cloud.hpp"

#include <iostream>
#include <algorithm>
#include <cmath>

namespace quickviz {

// === PointSelectionTool Implementation ===

PointSelectionTool::PointSelectionTool(const std::string& name, SceneManager* scene_manager)
    : InteractionTool(name, scene_manager) {
    
    // Set tool display properties
    SetDisplayName("Point Selection");
    SetDescription("Click to select points in point clouds. Hold Ctrl for multi-select.");
    
    // Initialize visual feedback with defaults
    visual_feedback_ = VisualFeedback{};
    
    // Initialize hover state
    hover_state_ = HoverState{};
    
    // Tool created - no debug output needed in production
}

void PointSelectionTool::DoActivate() {
    // Set up tool-specific state when activated
    ClearSelection();
    hover_state_.active = false;
    mouse_moved_since_click_ = false;
    
    // Tool activated
}

void PointSelectionTool::DoDeactivate() {
    // Clean up hover feedback when deactivated
    hover_state_.active = false;
    current_hover_ = std::monostate{};
    
    // Clear hover layer
    UpdateHoverLayer();
    
    // Tool deactivated
}

bool PointSelectionTool::OnMouseEvent(const InputEvent& event) {
    switch (event.GetType()) {
        case InputEventType::kMousePress:
        case InputEventType::kMouseRelease:
            return HandleMouseClick(event);
            
        case InputEventType::kMouseMove:
            return HandleMouseMove(event);
            
        default:
            return false;
    }
}

bool PointSelectionTool::OnKeyboardEvent(const InputEvent& event) {
    // Handle keyboard shortcuts for selection modes
    if (event.GetType() == InputEventType::kKeyPress) {
        int key = event.GetKey();
        
        // Map common keys (you may need to adjust based on actual key enum)
        if (key == 256) {  // Escape key (adjust as needed)
            // Clear selection on Escape
            ClearSelection();
            return true;
        }
        
        if (key == 65) {  // 'A' key (adjust as needed)
            // Select all points with Ctrl+A
            ModifierKeys ctrl_mod;
            ctrl_mod.ctrl = true;
            if (HasModifiers(event, ctrl_mod)) {
                // TODO: Implement select all functionality
                // For now, just consume the event without action
                return true;
            }
        }
    }
    
    return false;
}

bool PointSelectionTool::HandleMouseClick(const InputEvent& event) {
    if (event.GetType() != InputEventType::kMousePress) {
        return false;  // Only handle button press, not release
    }
    
    // Only handle left mouse button for selection (button code 0)
    if (event.GetMouseButton() != 0) {  // Left mouse button
        return false;
    }
    
    // Reset mouse movement tracking on each click
    auto screen_pos = event.GetScreenPosition();
    last_mouse_pos_ = screen_pos;
    mouse_moved_since_click_ = false;
    
    // Determine selection mode based on modifiers
    SelectionMode mode = DetermineSelectionMode(event);
    
    // Perform the selection
    float screen_x = screen_pos.x;
    float screen_y = screen_pos.y;
    
    PerformSelection(screen_x, screen_y, mode);
    
    return true;  // Consume the event
}

bool PointSelectionTool::HandleMouseMove(const InputEvent& event) {
    glm::vec2 current_pos = event.GetScreenPosition();
    
    // Track mouse movement for drag detection
    float movement = glm::length(current_pos - last_mouse_pos_);
    if (movement > 5.0f) {  // 5 pixel threshold
        mouse_moved_since_click_ = true;
    }
    
    last_mouse_pos_ = current_pos;
    
    // Update hover feedback if enabled
    if (visual_feedback_.show_hover_highlight) {
        UpdateHoverFeedback(current_pos.x, current_pos.y);
    }
    
    return false;  // Don't consume mouse move events
}

PointSelectionTool::SelectionMode PointSelectionTool::DetermineSelectionMode(const InputEvent& event) const {
    const auto& modifiers = event.GetModifiers();
    
    // Check modifier keys to determine selection mode
    if (modifiers.ctrl) {
        return SelectionMode::kToggle;  // Ctrl+Click = toggle selection
    } else if (modifiers.alt) {
        return SelectionMode::kSubtract;  // Alt+Click = remove from selection
    } else if (modifiers.shift) {
        return SelectionMode::kAdd;  // Shift+Click = add to selection
    }
    
    return default_mode_;  // Use default mode
}

void PointSelectionTool::PerformSelection(float screen_x, float screen_y, SelectionMode mode) {
    
    
    // Create selection options based on tool configuration
    SelectionOptions options;
    options.radius = selection_radius_;
    options.mode = quickviz::SelectionMode::kPoints;  // Force point-only selection
    options.target_object = target_point_cloud_;  // Filter to target point cloud if set
    
    SelectionResult result;
    
    // Use the appropriate SelectionManager method based on mode
    switch (mode) {
        case SelectionMode::kSingle:
            // Single selection: replace current selection
            options.add_to_selection = false;
            result = scene_manager_->Select(screen_x, screen_y, options);
            break;
            
        case SelectionMode::kAdd: {
            // Add to selection 
            bool added = scene_manager_->AddToSelection(screen_x, screen_y, options);
            
            // Don't call Select again - AddToSelection already updated the selection
            // For callback notification, just use an empty result since AddToSelection already handled it
            result = std::monostate{};
            break;
        }
            
        case SelectionMode::kToggle:
            // Toggle selection state
            scene_manager_->GetSelection().ToggleSelection(screen_x, screen_y, options);
            // Don't call Select again - ToggleSelection already updated the selection  
            // For callback notification, just use an empty result since ToggleSelection already handled it
            result = std::monostate{};
            break;
            
        case SelectionMode::kSubtract:
            // Find what would be selected and remove it
            options.add_to_selection = false;
            result = scene_manager_->Select(screen_x, screen_y, options);
            if (!IsEmpty(result)) {
                auto& multi_selection = scene_manager_->GetSelection().GetMultiSelection();
                const_cast<MultiSelection&>(multi_selection).Remove(result);
            }
            break;
    }
    
    // Update visual feedback
    UpdateVisualFeedback();
    
    // Notify callback - but only for modes that don't handle their own notification
    if (mode == SelectionMode::kSingle || mode == SelectionMode::kSubtract) {
        NotifySelectionChanged(result);
    }
    // Add and Toggle modes handle their own notifications via the SelectionManager
    
    // Selection completed - callback notification will handle any required logging
}

void PointSelectionTool::UpdateHoverFeedback(float screen_x, float screen_y) {
    // Create selection options for hover detection
    SelectionOptions options;
    options.radius = selection_radius_;
    options.mode = quickviz::SelectionMode::kPoints;
    options.target_object = target_point_cloud_;
    
    // Check what's under the cursor (without modifying selection state)
    SelectionResult hover_result = scene_manager_->GetSelection().QuerySelection(screen_x, screen_y, options);
    
    
    // Update hover state
    bool hover_changed = false;
    if (auto point_sel = std::get_if<PointSelection>(&hover_result)) {
        if (!hover_state_.active || 
            hover_state_.point_cloud_name != point_sel->cloud_name ||
            hover_state_.point_index != point_sel->point_index) {
            
            hover_state_.active = true;
            hover_state_.point_cloud_name = point_sel->cloud_name;
            hover_state_.point_index = point_sel->point_index;
            hover_state_.world_position = point_sel->world_position;
            hover_state_.screen_position = point_sel->screen_position;
            hover_changed = true;
        }
    } else {
        if (hover_state_.active) {
            hover_state_.active = false;
            hover_changed = true;
        }
    }
    
    // Update current hover result
    if (hover_changed) {
        current_hover_ = hover_result;
        NotifyHoverChanged(hover_result);
        
        // Update hover layer immediately when hover changes
        UpdateHoverLayer();
    }
}

void PointSelectionTool::UpdateHoverLayer() {
    // Apply hover highlighting using point cloud layers
    auto* point_cloud = dynamic_cast<PointCloud*>(scene_manager_->GetOpenGLObject("point_cloud"));
    if (!point_cloud) {
        return;
    }
    
    // Get or create hover layer
    auto hover_layer = point_cloud->GetLayer("tool_hover");
    bool newly_created = false;
    if (!hover_layer) {
        hover_layer = point_cloud->CreateLayer("tool_hover", 400);  // Higher priority than selection (300)
        newly_created = true;
    }
    
    if (hover_state_.active) {
        // Show hover for the current point
        
        // For newly created layers, we need to set initial properties
        if (newly_created) {
            hover_layer->SetColor(visual_feedback_.hover_color);
            hover_layer->SetPointSizeMultiplier(visual_feedback_.hover_size_multiplier);
            hover_layer->SetHighlightMode(quickviz::PointLayer::HighlightMode::kColorAndSize);
        }
        
        // Update the point being hovered
        std::vector<size_t> hover_points = {hover_state_.point_index};
        hover_layer->SetPoints(hover_points);
        hover_layer->SetVisible(true);
    } else {
        // Hide hover layer when not hovering
        hover_layer->ClearPoints();
        hover_layer->SetVisible(false);
    }
}

void PointSelectionTool::ClearSelectionLayers() {
    // Clear selection layers from known point clouds
    // For now, we'll clear the main point cloud that we know about
    auto* point_cloud = dynamic_cast<PointCloud*>(scene_manager_->GetOpenGLObject("point_cloud"));
    if (point_cloud) {
        auto selection_layer = point_cloud->GetLayer("tool_selection");
        if (selection_layer) {
            selection_layer->ClearPoints();
            selection_layer->SetVisible(false);
        }
    }
}

void PointSelectionTool::UpdateVisualFeedback() {
    // Apply visual feedback to selected points using point cloud layers
    auto& multi_selection = GetMultiSelection();
    auto point_selections = multi_selection.GetPoints();
    
    // Clear existing selection layers first
    ClearSelectionLayers();
    
    // Group selections by point cloud
    std::map<std::string, std::vector<size_t>> selections_by_cloud;
    for (const auto& point_sel : point_selections) {
        selections_by_cloud[point_sel.cloud_name].push_back(point_sel.point_index);
    }
    
    // Apply highlighting to each point cloud
    for (const auto& [cloud_name, point_indices] : selections_by_cloud) {
        auto* point_cloud = dynamic_cast<PointCloud*>(scene_manager_->GetOpenGLObject(cloud_name));
        if (point_cloud) {
            // Create or update selection layer
            auto selection_layer = point_cloud->GetLayer("tool_selection");
            if (!selection_layer) {
                selection_layer = point_cloud->CreateLayer("tool_selection", 300);  // Higher priority than hover
            }
            
            selection_layer->SetPoints(point_indices);
            selection_layer->SetColor(visual_feedback_.selection_color);
            selection_layer->SetPointSizeMultiplier(visual_feedback_.selection_size_multiplier);
            selection_layer->SetHighlightMode(quickviz::PointLayer::HighlightMode::kColorAndSize);
            selection_layer->SetVisible(true);
        }
    }
}

void PointSelectionTool::NotifySelectionChanged(const SelectionResult& result) {
    if (selection_callback_) {
        selection_callback_(result, GetMultiSelection());
    }
}

void PointSelectionTool::NotifyHoverChanged(const SelectionResult& result) {
    if (hover_callback_) {
        hover_callback_(result);
    }
}

// === Visual Feedback Rendering ===

void PointSelectionTool::OnRender(const glm::mat4& projection, const glm::mat4& view) {
    if (GetState() == State::kInactive) {
        return;
    }
    
    // Render hover feedback
    if (visual_feedback_.show_hover_highlight && hover_state_.active) {
        RenderHoverFeedback(projection, view);
    }
    
    // Render selection radius indicator
    if (visual_feedback_.show_selection_radius) {
        RenderSelectionRadius(projection, view);
    }
    
    // Render selection count overlay
    if (visual_feedback_.show_selection_count && GetSelectionCount() > 0) {
        RenderSelectionCount(projection, view);
    }
}

void PointSelectionTool::RenderHoverFeedback(const glm::mat4& projection, const glm::mat4& view) {
    // Apply hover highlighting using point cloud layers
    auto* point_cloud = dynamic_cast<PointCloud*>(scene_manager_->GetOpenGLObject("point_cloud"));
    if (!point_cloud) {
        return;
    }
    
    // Get or create hover layer
    auto hover_layer = point_cloud->GetLayer("tool_hover");
    if (!hover_layer) {
        hover_layer = point_cloud->CreateLayer("tool_hover", 400);  // Higher priority than selection (300)
    }
    
    if (hover_state_.active) {
        // Show hover for the current point
        hover_layer->ClearPoints();  // Clear any previous hover points
        hover_layer->SetPoints({hover_state_.point_index});
        hover_layer->SetColor(visual_feedback_.hover_color);
        hover_layer->SetPointSizeMultiplier(visual_feedback_.hover_size_multiplier);
        hover_layer->SetHighlightMode(quickviz::PointLayer::HighlightMode::kColorAndSize);
        hover_layer->SetVisible(true);
    } else {
        // Hide hover layer when not hovering
        hover_layer->ClearPoints();
        hover_layer->SetVisible(false);
    }
}

void PointSelectionTool::RenderSelectionRadius(const glm::mat4& projection, const glm::mat4& view) {
    // TODO: Implement selection radius circle rendering
    // This would draw a circle at the current mouse position showing the selection tolerance
    // Implementation would use immediate mode rendering similar to hover feedback
}

void PointSelectionTool::RenderSelectionCount(const glm::mat4& projection, const glm::mat4& view) {
    // TODO: Implement selection count overlay rendering  
    // This would draw text showing "X points selected" in the corner
    // Could be implemented using ImGui overlay or OpenGL text rendering
}

// === Public API Implementation ===

const SelectionResult& PointSelectionTool::GetCurrentSelection() const {
    return scene_manager_->GetSelection().GetCurrentSelection();
}

const MultiSelection& PointSelectionTool::GetMultiSelection() const {
    return scene_manager_->GetSelection().GetMultiSelection();
}

size_t PointSelectionTool::GetSelectionCount() const {
    return scene_manager_->GetSelection().GetSelectionCount();
}

void PointSelectionTool::ClearSelection() {
    scene_manager_->GetSelection().ClearSelection();
    UpdateVisualFeedback();  // This now includes clearing old layers
    
    // Notify callback
    SelectionResult empty_result = std::monostate{};
    NotifySelectionChanged(empty_result);
}

bool PointSelectionTool::SelectPointAt(float screen_x, float screen_y, SelectionMode mode) {
    PerformSelection(screen_x, screen_y, mode);
    return !IsEmpty(GetCurrentSelection());
}

bool PointSelectionTool::SelectPointByIndex(const std::string& point_cloud_name, size_t point_index, SelectionMode mode) {
    // TODO: Implement direct point selection by index
    // This would need to work with the SelectionManager to create a PointSelection result
    // Could be useful for programmatic selection or selection restoration
    return false;
}

// === Factory Implementation ===

std::shared_ptr<PointSelectionTool> PointSelectionToolFactory::CreateStandard(
    SceneManager* scene_manager, const std::string& name) {
    
    auto tool = std::make_shared<PointSelectionTool>(name, scene_manager);
    
    // Configure with standard settings
    tool->SetSelectionMode(PointSelectionTool::SelectionMode::kSingle);
    tool->SetSelectionRadius(3);
    
    return tool;
}

std::shared_ptr<PointSelectionTool> PointSelectionToolFactory::CreateForPointCloud(
    SceneManager* scene_manager,
    const std::string& point_cloud_name,
    const std::string& name) {
    
    auto tool = CreateStandard(scene_manager, name);
    tool->SetTargetPointCloud(point_cloud_name);
    tool->SetDisplayName("Point Selection (" + point_cloud_name + ")");
    
    return tool;
}

std::shared_ptr<PointSelectionTool> PointSelectionToolFactory::CreateWithVisualFeedback(
    SceneManager* scene_manager,
    const PointSelectionTool::VisualFeedback& feedback,
    const std::string& name) {
    
    auto tool = CreateStandard(scene_manager, name);
    tool->SetVisualFeedback(feedback);
    
    return tool;
}

} // namespace quickviz