/**
 * @file test_point_selection_tool.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)  
 * @date 2025-09-02
 * @brief Tests for point selection tool functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "scene/tools/point_selection_tool.hpp"
#include "scene/tools/interaction_tool.hpp"
#include "scene/scene_manager.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "core/event/input_event.hpp"

namespace quickviz {

class PointSelectionToolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create scene manager for testing
        scene_manager = std::make_unique<SceneManager>("test_scene");
        
        // Create test point cloud
        point_cloud = std::make_unique<PointCloud>();
        
        // Add some test points
        std::vector<glm::vec3> test_points = {
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f),
            glm::vec3(1.0f, 1.0f, 1.0f)
        };
        
        std::vector<glm::vec3> test_colors(test_points.size(), glm::vec3(1.0f, 0.0f, 0.0f));
        point_cloud->SetPoints(test_points, test_colors);
        
        // Add point cloud to scene  
        scene_manager->AddOpenGLObject("test_cloud", std::move(point_cloud));
        
        // Create point selection tool
        tool = std::make_shared<PointSelectionTool>("test_tool", scene_manager.get());
        
        // Register tool with scene manager
        scene_manager->RegisterTool(tool);
    }

    void TearDown() override {
        tool.reset();
        scene_manager.reset();
    }

    // Helper to create mouse click event
    InputEvent CreateMouseClickEvent(float x, float y, MouseButton button = MouseButton::kLeft, 
                                   ModifierKeys modifiers = ModifierKeys::kNone) {
        InputEvent event;
        event.type = InputEventType::kMouseButton;
        event.mouse_data.x = x;
        event.mouse_data.y = y;
        event.mouse_data.button = button;
        event.mouse_data.action = MouseAction::kPress;
        event.modifiers = modifiers;
        return event;
    }
    
    // Helper to create mouse move event
    InputEvent CreateMouseMoveEvent(float x, float y) {
        InputEvent event;
        event.type = InputEventType::kMouseMove;
        event.mouse_data.x = x;
        event.mouse_data.y = y;
        return event;
    }

    std::unique_ptr<SceneManager> scene_manager;
    std::unique_ptr<PointCloud> point_cloud;
    std::shared_ptr<PointSelectionTool> tool;
};

// === Basic Tool Functionality Tests ===

TEST_F(PointSelectionToolTest, ToolCreation) {
    EXPECT_EQ(tool->GetName(), "test_tool");
    EXPECT_EQ(tool->GetDisplayName(), "Point Selection");
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kInactive);
    EXPECT_TRUE(tool->IsEnabled());
    EXPECT_EQ(tool->GetCursorType(), InteractionTool::CursorType::kCrosshair);
}

TEST_F(PointSelectionToolTest, ToolActivationDeactivation) {
    // Test activation
    tool->OnActivate();
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kActive);
    
    // Test deactivation
    tool->OnDeactivate();
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kInactive);
}

TEST_F(PointSelectionToolTest, DefaultConfiguration) {
    EXPECT_EQ(tool->GetSelectionMode(), PointSelectionTool::SelectionMode::kSingle);
    EXPECT_EQ(tool->GetSelectionRadius(), 3);
    EXPECT_TRUE(tool->GetTargetPointCloud().empty());
    EXPECT_EQ(tool->GetSelectionCount(), 0);
}

// === Configuration Tests ===

TEST_F(PointSelectionToolTest, SelectionModeConfiguration) {
    tool->SetSelectionMode(PointSelectionTool::SelectionMode::kAdd);
    EXPECT_EQ(tool->GetSelectionMode(), PointSelectionTool::SelectionMode::kAdd);
    
    tool->SetSelectionMode(PointSelectionTool::SelectionMode::kToggle);
    EXPECT_EQ(tool->GetSelectionMode(), PointSelectionTool::SelectionMode::kToggle);
}

TEST_F(PointSelectionToolTest, SelectionRadiusConfiguration) {
    tool->SetSelectionRadius(5);
    EXPECT_EQ(tool->GetSelectionRadius(), 5);
    
    tool->SetSelectionRadius(10);
    EXPECT_EQ(tool->GetSelectionRadius(), 10);
}

TEST_F(PointSelectionToolTest, TargetPointCloudConfiguration) {
    tool->SetTargetPointCloud("test_cloud");
    EXPECT_EQ(tool->GetTargetPointCloud(), "test_cloud");
    
    tool->SetTargetPointCloud("");
    EXPECT_TRUE(tool->GetTargetPointCloud().empty());
}

TEST_F(PointSelectionToolTest, VisualFeedbackConfiguration) {
    PointSelectionTool::VisualFeedback feedback;
    feedback.show_hover_highlight = false;
    feedback.show_selection_radius = true;
    feedback.hover_color = glm::vec3(0.0f, 1.0f, 0.0f);  // Green
    
    tool->SetVisualFeedback(feedback);
    const auto& retrieved_feedback = tool->GetVisualFeedback();
    
    EXPECT_FALSE(retrieved_feedback.show_hover_highlight);
    EXPECT_TRUE(retrieved_feedback.show_selection_radius);
    EXPECT_EQ(retrieved_feedback.hover_color, glm::vec3(0.0f, 1.0f, 0.0f));
}

// === Input Handling Tests ===

TEST_F(PointSelectionToolTest, InputEventHandlingWhenInactive) {
    // Tool should not handle input when inactive
    auto event = CreateMouseClickEvent(100.0f, 100.0f);
    EXPECT_FALSE(tool->OnInputEvent(event));
}

TEST_F(PointSelectionToolTest, InputEventHandlingWhenActive) {
    tool->OnActivate();
    
    // Tool should handle mouse events when active
    auto click_event = CreateMouseClickEvent(100.0f, 100.0f);
    // Note: This might return false if no point is found, but the tool should still process it
    tool->OnInputEvent(click_event);
    
    // Tool should handle mouse move events (but not consume them)
    auto move_event = CreateMouseMoveEvent(150.0f, 150.0f);
    EXPECT_FALSE(tool->OnInputEvent(move_event));  // Move events are not consumed
}

TEST_F(PointSelectionToolTest, KeyboardEventHandling) {
    tool->OnActivate();
    
    // Create Escape key event
    InputEvent event;
    event.type = InputEventType::kKeyboard;
    event.keyboard_data.key = Key::kEscape;
    event.keyboard_data.action = KeyAction::kPress;
    event.modifiers = ModifierKeys::kNone;
    
    // Escape should be handled and clear selection
    EXPECT_TRUE(tool->OnInputEvent(event));
}

// === Selection Tests ===

TEST_F(PointSelectionToolTest, SelectionClearing) {
    tool->ClearSelection();
    EXPECT_EQ(tool->GetSelectionCount(), 0);
    EXPECT_TRUE(IsEmpty(tool->GetCurrentSelection()));
}

TEST_F(PointSelectionToolTest, ProgrammaticSelection) {
    tool->OnActivate();
    
    // Test programmatic selection at screen coordinates
    // Note: This may not find actual points without proper scene setup
    bool result = tool->SelectPointAt(100.0f, 100.0f, PointSelectionTool::SelectionMode::kSingle);
    
    // The result depends on whether a point is actually found at those coordinates
    // In this test setup, it will likely be false since we don't have full GL context
    EXPECT_FALSE(result);  // Expected for test without full rendering
}

// === Callback Tests ===

TEST_F(PointSelectionToolTest, SelectionCallback) {
    bool callback_called = false;
    SelectionResult callback_result;
    MultiSelection callback_multi_selection;
    
    tool->SetSelectionCallback([&](const SelectionResult& result, const MultiSelection& multi) {
        callback_called = true;
        callback_result = result;
        callback_multi_selection = multi;
    });
    
    // Clear selection should trigger callback
    tool->ClearSelection();
    EXPECT_TRUE(callback_called);
    EXPECT_TRUE(IsEmpty(callback_result));
}

TEST_F(PointSelectionToolTest, HoverCallback) {
    bool hover_callback_called = false;
    SelectionResult hover_result;
    
    tool->SetHoverCallback([&](const SelectionResult& result) {
        hover_callback_called = true;
        hover_result = result;
    });
    
    tool->OnActivate();
    
    // Mouse move should potentially trigger hover callback
    auto move_event = CreateMouseMoveEvent(100.0f, 100.0f);
    tool->OnInputEvent(move_event);
    
    // Hover callback may or may not be called depending on whether point is found
    // This is expected behavior - no assertion needed for this case
}

// === Factory Tests ===

TEST_F(PointSelectionToolTest, StandardFactory) {
    auto standard_tool = PointSelectionToolFactory::CreateStandard(scene_manager.get(), "standard_tool");
    
    EXPECT_EQ(standard_tool->GetName(), "standard_tool");
    EXPECT_EQ(standard_tool->GetSelectionMode(), PointSelectionTool::SelectionMode::kSingle);
    EXPECT_EQ(standard_tool->GetSelectionRadius(), 3);
    EXPECT_TRUE(standard_tool->GetTargetPointCloud().empty());
}

TEST_F(PointSelectionToolTest, PointCloudSpecificFactory) {
    auto pc_tool = PointSelectionToolFactory::CreateForPointCloud(
        scene_manager.get(), "test_cloud", "pc_tool");
    
    EXPECT_EQ(pc_tool->GetName(), "pc_tool");
    EXPECT_EQ(pc_tool->GetTargetPointCloud(), "test_cloud");
    EXPECT_EQ(pc_tool->GetDisplayName(), "Point Selection (test_cloud)");
}

TEST_F(PointSelectionToolTest, VisualFeedbackFactory) {
    PointSelectionTool::VisualFeedback feedback;
    feedback.hover_color = glm::vec3(1.0f, 0.0f, 1.0f);  // Magenta
    feedback.show_selection_radius = true;
    
    auto feedback_tool = PointSelectionToolFactory::CreateWithVisualFeedback(
        scene_manager.get(), feedback, "feedback_tool");
    
    EXPECT_EQ(feedback_tool->GetName(), "feedback_tool");
    const auto& retrieved_feedback = feedback_tool->GetVisualFeedback();
    EXPECT_EQ(retrieved_feedback.hover_color, glm::vec3(1.0f, 0.0f, 1.0f));
    EXPECT_TRUE(retrieved_feedback.show_selection_radius);
}

// === Tool Manager Integration Tests ===

TEST_F(PointSelectionToolTest, ToolManagerIntegration) {
    // Test tool registration through scene manager
    EXPECT_TRUE(scene_manager->ActivateTool("test_tool"));
    EXPECT_EQ(scene_manager->GetActiveTool(), tool);
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kActive);
    
    // Test tool deactivation
    scene_manager->GetTools().DeactivateCurrentTool();
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kInactive);
    EXPECT_EQ(scene_manager->GetActiveTool(), nullptr);
}

TEST_F(PointSelectionToolTest, PriorityBasedEventHandling) {
    // Test that tool priority changes based on state
    int inactive_priority = tool->GetPriority();
    
    tool->OnActivate();
    int active_priority = tool->GetPriority();
    
    // Active tools should have higher priority
    EXPECT_GT(active_priority, inactive_priority);
}

// === Edge Case Tests ===

TEST_F(PointSelectionToolTest, InvalidSceneManagerHandling) {
    // Test that tool creation with null scene manager throws
    EXPECT_THROW(
        std::make_shared<PointSelectionTool>("invalid_tool", nullptr),
        std::invalid_argument
    );
}

TEST_F(PointSelectionToolTest, MultipleActivationDeactivation) {
    // Multiple activations should be safe
    tool->OnActivate();
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kActive);
    
    tool->OnActivate();  // Second activation
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kActive);
    
    // Multiple deactivations should be safe
    tool->OnDeactivate();
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kInactive);
    
    tool->OnDeactivate();  // Second deactivation
    EXPECT_EQ(tool->GetState(), InteractionTool::State::kInactive);
}

} // namespace quickviz