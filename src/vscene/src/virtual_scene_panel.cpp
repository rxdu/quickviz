/*
 * @file virtual_scene_panel.cpp
 * @date August 27, 2025
 * @brief Implementation of VirtualScenePanel for ImGui integration
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/virtual_scene_panel.hpp"
#include "vscene/event_system.hpp"
#include "imgui.h"
#include <chrono>
#include <iostream>

namespace quickviz {

VirtualScenePanel::VirtualScenePanel(const std::string& name)
    : Panel(name), virtual_scene_(std::make_unique<VirtualScene>()) {
}

VirtualScenePanel::~VirtualScenePanel() = default;

void VirtualScenePanel::Draw() {
    Begin();
    RenderInsideWindow();
    End();
}

void VirtualScenePanel::RenderInsideWindow() {
    // Update virtual scene
    virtual_scene_->Update(0.016f); // Assume 60 FPS for now
    
    // Handle input events first
    HandleInput();
    
    // Get the available content region for rendering
    ImVec2 content_region = ImGui::GetContentRegionAvail();
    
    // Render the virtual scene if we have a backend
    if (auto* backend = virtual_scene_->GetRenderBackend()) {
        // Render to the content region size
        backend->RenderToFramebuffer(content_region.x, content_region.y);
        
        // Get the rendered texture and display it
        // Note: This assumes the backend provides a way to get the rendered texture
        // In real implementation, this would integrate with the backend's texture system
        
        // Create an invisible button that covers the entire content area for input handling
        ImGui::InvisibleButton("scene_area", content_region);
        
        // Check if the scene area is hovered/clicked for input processing
        if (ImGui::IsItemHovered()) {
            ProcessMouseHover();
        }
        
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            ProcessMouseClick(0);
        } else if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
            ProcessMouseClick(1);
        } else if (ImGui::IsItemClicked(ImGuiMouseButton_Middle)) {
            ProcessMouseClick(2);
        }
        
        // Handle dragging
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            ProcessMouseDrag();
        }
        
        // Update interaction state
        UpdateInteractionState();
        
        // Draw debug overlay if enabled
        if (config_.show_debug_info) {
            RenderDebugOverlay();
        }
    } else {
        // No backend - show placeholder
        ImGui::Text("No render backend configured");
        ImGui::Text("Content region: %.1f x %.1f", content_region.x, content_region.y);
    }
}

void VirtualScenePanel::SetRenderBackend(std::unique_ptr<RenderInterface> backend) {
    virtual_scene_->SetRenderBackend(std::move(backend));
}

void VirtualScenePanel::HandleInput() {
    // Update mouse position
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 window_pos = GetWindowPos();
    ImVec2 content_min = ImGui::GetWindowContentRegionMin();
    
    // Convert to local coordinates relative to content area
    glm::vec2 local_mouse = glm::vec2(
        mouse_pos.x - window_pos.x - content_min.x,
        mouse_pos.y - window_pos.y - content_min.y
    );
    
    input_state_.last_mouse_pos = local_mouse;
}

void VirtualScenePanel::RenderDebugOverlay() {
    // Get window draw list for overlay rendering
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 window_pos = GetWindowPos();
    ImVec2 window_size = GetWindowSize();
    
    // Draw selection count
    std::string debug_text = "Selected: " + std::to_string(virtual_scene_->GetSelectedCount());
    debug_text += " / " + std::to_string(virtual_scene_->GetObjectIds().size());
    
    // Position in top-left corner of content area
    ImVec2 text_pos = ImVec2(window_pos.x + 10, window_pos.y + 30);
    draw_list->AddText(text_pos, IM_COL32(255, 255, 0, 255), debug_text.c_str());
    
    // Draw mouse position
    std::string mouse_text = "Mouse: (" + 
        std::to_string((int)input_state_.last_mouse_pos.x) + ", " +
        std::to_string((int)input_state_.last_mouse_pos.y) + ")";
    ImVec2 mouse_text_pos = ImVec2(text_pos.x, text_pos.y + 20);
    draw_list->AddText(mouse_text_pos, IM_COL32(255, 255, 0, 255), mouse_text.c_str());
}

void VirtualScenePanel::UpdateInteractionState() {
    // Update visual feedback for selected objects
    auto selected_ids = virtual_scene_->GetSelectedIds();
    for (const auto& id : selected_ids) {
        if (auto* object = virtual_scene_->GetObject(id)) {
            // Visual feedback is handled by the virtual object's selection state
            // which should be reflected in the backend rendering
        }
    }
}

glm::vec2 VirtualScenePanel::GetLocalMousePosition() const {
    return input_state_.last_mouse_pos;
}

bool VirtualScenePanel::IsMouseInContentArea() const {
    ImVec2 content_region = ImGui::GetContentRegionAvail();
    glm::vec2 mouse_pos = GetLocalMousePosition();
    
    return mouse_pos.x >= 0 && mouse_pos.y >= 0 &&
           mouse_pos.x < content_region.x && mouse_pos.y < content_region.y;
}

void VirtualScenePanel::ProcessMouseClick(int button) {
    if (!IsMouseInContentArea()) return;
    
    glm::vec2 mouse_pos = GetLocalMousePosition();
    
    // Use the virtual scene's picking system
    VirtualObject* clicked_object = virtual_scene_->Pick(mouse_pos.x, mouse_pos.y);
    
    if (clicked_object) {
        // Dispatch object click event
        DispatchClickEvent(clicked_object, button);
        
        // Handle selection logic
        bool ctrl_pressed = ImGui::GetIO().KeyCtrl;
        bool shift_pressed = ImGui::GetIO().KeyShift;
        
        if (ctrl_pressed && config_.enable_multi_selection) {
            // Toggle selection
            virtual_scene_->ToggleSelection(clicked_object->GetId());
        } else if (shift_pressed && config_.enable_multi_selection) {
            // Add to selection
            virtual_scene_->AddToSelection(clicked_object->GetId());
        } else {
            // Single selection (clear others first)
            virtual_scene_->ClearSelection();
            virtual_scene_->SetSelected(clicked_object->GetId(), true);
        }
        
        // Call object's click callback if it exists
        if (clicked_object->OnClick) {
            glm::vec3 world_pos(0.0f); // TODO: Convert screen to world coordinates
            clicked_object->OnClick(clicked_object, mouse_pos, world_pos);
        }
    } else {
        // Background click
        glm::vec3 world_pos(0.0f); // TODO: Convert screen to world coordinates
        
        // Dispatch background click event
        auto event = EventBuilder::BackgroundClicked(mouse_pos, world_pos, button);
        event.ctrl_pressed = ImGui::GetIO().KeyCtrl;
        event.shift_pressed = ImGui::GetIO().KeyShift;
        event.alt_pressed = ImGui::GetIO().KeyAlt;
        virtual_scene_->GetEventDispatcher()->Dispatch(event);
        
        // Clear selection unless Ctrl is held
        if (!ImGui::GetIO().KeyCtrl) {
            virtual_scene_->ClearSelection();
        }
    }
}

void VirtualScenePanel::ProcessMouseDrag() {
    if (!IsMouseInContentArea()) return;
    
    if (!input_state_.dragging) {
        // Start dragging
        input_state_.dragging = true;
        glm::vec2 mouse_pos = GetLocalMousePosition();
        input_state_.drag_object = virtual_scene_->Pick(mouse_pos.x, mouse_pos.y);
        if (input_state_.drag_object) {
            input_state_.drag_start_world_pos = glm::vec3(0.0f); // TODO: Convert screen to world
        }
    } else if (input_state_.drag_object) {
        // Continue dragging
        ImVec2 mouse_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        glm::vec3 world_delta(mouse_delta.x, -mouse_delta.y, 0.0f); // Flip Y for 3D coordinates
        
        // Dispatch drag event
        DispatchDragEvent(input_state_.drag_object);
        
        // Call object's drag callback if it exists
        if (input_state_.drag_object->OnDrag) {
            input_state_.drag_object->OnDrag(input_state_.drag_object, world_delta);
        }
    }
    
    // Reset drag state when mouse is released
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        input_state_.dragging = false;
        input_state_.drag_object = nullptr;
    }
}

void VirtualScenePanel::ProcessMouseHover() {
    if (!IsMouseInContentArea()) return;
    
    glm::vec2 mouse_pos = GetLocalMousePosition();
    VirtualObject* hovered_object = virtual_scene_->Pick(mouse_pos.x, mouse_pos.y);
    
    // Process hover state changes (handled by VirtualScene internally)
    virtual_scene_->ProcessHover(mouse_pos.x, mouse_pos.y);
}

void VirtualScenePanel::DispatchClickEvent(VirtualObject* object, int button) {
    if (!object) return;
    
    glm::vec2 mouse_pos = GetLocalMousePosition();
    glm::vec3 world_pos(0.0f); // TODO: Convert screen to world coordinates
    
    auto event = EventBuilder::ObjectClicked(object->GetId(), mouse_pos, world_pos, button);
    event.ctrl_pressed = ImGui::GetIO().KeyCtrl;
    event.shift_pressed = ImGui::GetIO().KeyShift;
    event.alt_pressed = ImGui::GetIO().KeyAlt;
    
    virtual_scene_->GetEventDispatcher()->Dispatch(event);
}

void VirtualScenePanel::DispatchDragEvent(VirtualObject* object) {
    if (!object) return;
    
    glm::vec2 mouse_pos = GetLocalMousePosition();
    ImVec2 mouse_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
    glm::vec3 world_delta(mouse_delta.x, -mouse_delta.y, 0.0f);
    
    auto event = EventBuilder::ObjectDragged(object->GetId(), mouse_pos, world_delta);
    virtual_scene_->GetEventDispatcher()->Dispatch(event);
}

void VirtualScenePanel::DispatchHoverEvent(VirtualObject* object, bool entering) {
    if (!object) return;
    
    VirtualEventType event_type = entering ? VirtualEventType::ObjectHoverEnter : VirtualEventType::ObjectHoverExit;
    
    VirtualEvent event;
    event.type = event_type;
    event.object_id = object->GetId();
    event.screen_pos = GetLocalMousePosition();
    event.timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    
    virtual_scene_->GetEventDispatcher()->Dispatch(event);
}

} // namespace quickviz