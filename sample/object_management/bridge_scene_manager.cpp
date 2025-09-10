/*
 * @file bridge_scene_manager.cpp
 * @date Sep 10, 2025
 * @brief Implementation of Bridge Scene Manager
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "bridge_scene_manager.hpp"
#include "bridge_control_panel.hpp"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace quickviz {

BridgeSceneManager::BridgeSceneManager(const std::string& name, SceneManager::Mode mode)
    : GlScenePanel(name, mode) {
    
    // Create a shared scene manager for the bridge
    // We'll use the existing one through a shared pointer wrapper
    auto shared_scene_manager = std::shared_ptr<SceneManager>(GetSceneManager(), [](SceneManager*){
        // Custom deleter that does nothing since GlScenePanel owns it
    });
    
    // Create the bridge with our scene manager
    bridge_ = std::make_unique<SceneManagerBridge>(shared_scene_manager);
    
    std::cout << "BridgeSceneManager created with bridge integration" << std::endl;
}

void BridgeSceneManager::Draw() {
    // Update demo animations if enabled
    UpdateDemoAnimations();
    
    // Call parent draw (renders the 3D scene)
    GlScenePanel::Draw();
    
    // Draw status overlay
    DrawStatusOverlay();
}

void BridgeSceneManager::UpdateDemoAnimations() {
    // Check if auto rotate is enabled from control panel
    bool should_rotate = control_panel_ ? control_panel_->IsAutoRotateDemoEnabled() : false;
    if (!should_rotate) return;
    
    // Rotate some objects for demonstration
    demo_rotation_angle_ += 0.01f;
    if (demo_rotation_angle_ > 2.0f * M_PI) {
        demo_rotation_angle_ -= 2.0f * M_PI;
    }
    
    // Only animate in Direct mode to show performance
    if (bridge_->GetOperationMode() == OperationMode::kDirect) {
        // Rotate spheres around their axes
        auto sphere1_id = bridge_->GetObjectId("sphere1");
        auto sphere2_id = bridge_->GetObjectId("sphere2");
        
        if (sphere1_id != kInvalidObjectId) {
            glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), demo_rotation_angle_, glm::vec3(0, 0, 1));
            glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(-2, -2, 2));
            bridge_->SetTransform(sphere1_id, translation * rotation);
        }
        
        if (sphere2_id != kInvalidObjectId) {
            glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), -demo_rotation_angle_, glm::vec3(1, 1, 0));
            glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(2, -2, 2));
            bridge_->SetTransform(sphere2_id, translation * rotation);
        }
    }
}

void BridgeSceneManager::DrawStatusOverlay() {
    // Update status information periodically to avoid per-frame string operations
    status_refresh_timer_ += ImGui::GetIO().DeltaTime;
    if (status_refresh_timer_ >= kStatusRefreshInterval) {
        status_refresh_timer_ = 0.0f;
        
        // Update mode text
        switch (bridge_->GetOperationMode()) {
            case OperationMode::kDirect:
                cached_mode_text_ = "Direct Mode (Real-time)";
                cached_mode_color_ = IM_COL32(0, 255, 0, 255);
                break;
            case OperationMode::kImmediate:
                cached_mode_text_ = "Immediate Mode";
                cached_mode_color_ = IM_COL32(255, 255, 0, 255);
                break;
            case OperationMode::kRecorded:
                cached_mode_text_ = "Recorded Mode (Undo/Redo)";
                cached_mode_color_ = IM_COL32(255, 100, 100, 255);
                break;
        }
        
        // Update undo/redo text
        if (bridge_->SupportsUndo()) {
            auto stats = bridge_->GetCommandStatistics();
            
            char undo_buffer[64];
            snprintf(undo_buffer, sizeof(undo_buffer), "Undo: %s | Redo: %s", 
                    bridge_->CanUndo() ? "Available" : "None",
                    bridge_->CanRedo() ? "Available" : "None");
            cached_undo_text_ = undo_buffer;
            
            char stats_buffer[64];
            snprintf(stats_buffer, sizeof(stats_buffer), "Commands: %zu", stats.total_commands_executed);
            cached_stats_text_ = stats_buffer;
        }
    }
    
    // Show current operation mode in the 3D view
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();
    
    // Status background
    ImVec2 status_pos = ImVec2(window_pos.x + 10, window_pos.y + 30);
    ImVec2 status_size = ImVec2(200, 60);
    
    draw_list->AddRectFilled(
        status_pos, 
        ImVec2(status_pos.x + status_size.x, status_pos.y + status_size.y),
        IM_COL32(0, 0, 0, 128)
    );
    
    // Draw cached mode text
    draw_list->AddText(
        ImVec2(status_pos.x + 5, status_pos.y + 5),
        cached_mode_color_,
        cached_mode_text_.c_str()
    );
    
    // Draw cached undo/redo information
    if (bridge_->SupportsUndo()) {
        draw_list->AddText(
            ImVec2(status_pos.x + 5, status_pos.y + 25),
            IM_COL32(200, 200, 200, 255),
            cached_undo_text_.c_str()
        );
        
        draw_list->AddText(
            ImVec2(status_pos.x + 5, status_pos.y + 40),
            IM_COL32(150, 150, 150, 255),
            cached_stats_text_.c_str()
        );
    }
    
    // Demo rotation indicator
    bool should_show = control_panel_ ? control_panel_->IsAutoRotateDemoEnabled() : false;
    if (should_show) {
        draw_list->AddText(
            ImVec2(window_pos.x + window_size.x - 120, window_pos.y + 30),
            IM_COL32(100, 255, 100, 255),
            "Demo Animation ON"
        );
    }
}

} // namespace quickviz