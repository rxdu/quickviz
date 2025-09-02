/**
 * @file interaction_tool.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-02
 * @brief Implementation of base interaction tool interface
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/tools/interaction_tool.hpp"
#include "gldraw/scene_manager.hpp"

#include <algorithm>
#include <iostream>

namespace quickviz {

// === InteractionTool Implementation ===

InteractionTool::InteractionTool(const std::string& name, SceneManager* scene_manager)
    : name_(name)
    , display_name_(name)
    , scene_manager_(scene_manager) {
    if (!scene_manager_) {
        throw std::invalid_argument("InteractionTool: scene_manager cannot be null");
    }
}

// GetPriority is already implemented in the base class InputEventHandler

bool InteractionTool::OnInputEvent(const InputEvent& event) {
    // Only handle input when enabled and not inactive
    if (!enabled_ || state_ == State::kInactive) {
        return false;
    }
    
    // Dispatch to specific event handlers
    if (event.IsMouseEvent()) {
        return OnMouseEvent(event);
    } else if (event.IsKeyboardEvent()) {
        return OnKeyboardEvent(event);
    }
    
    return false;
}

void InteractionTool::OnActivate() {
    if (state_ != State::kInactive) {
        return; // Already active or working
    }
    
    SetState(State::kActive);
    DoActivate();
    
    std::cout << "Tool '" << display_name_ << "' activated" << std::endl;
}

void InteractionTool::OnDeactivate() {
    if (state_ == State::kInactive) {
        return; // Already inactive
    }
    
    State old_state = state_;
    SetState(State::kInactive);
    DoDeactivate();
    
    std::cout << "Tool '" << display_name_ << "' deactivated (was " 
              << static_cast<int>(old_state) << ")" << std::endl;
}

void InteractionTool::SetState(State new_state) {
    if (state_ == new_state) {
        return;
    }
    
    State old_state = state_;
    state_ = new_state;
    
    // State transition validation
    switch (new_state) {
        case State::kInactive:
            // Can always go inactive
            break;
            
        case State::kActive:
            // Can only activate from inactive or hover
            if (old_state != State::kInactive && old_state != State::kHover) {
                std::cerr << "Warning: Invalid state transition from " 
                          << static_cast<int>(old_state) << " to Active" << std::endl;
            }
            break;
            
        case State::kHover:
            // Can hover from inactive or active
            if (old_state != State::kInactive && old_state != State::kActive) {
                std::cerr << "Warning: Invalid state transition from " 
                          << static_cast<int>(old_state) << " to Hover" << std::endl;
            }
            break;
            
        case State::kWorking:
            // Can only work from active state
            if (old_state != State::kActive) {
                std::cerr << "Warning: Invalid state transition from " 
                          << static_cast<int>(old_state) << " to Working" << std::endl;
            }
            break;
    }
}

glm::vec2 InteractionTool::ScreenToNDC(const glm::vec2& screen_pos, const glm::vec2& viewport_size) const {
    // Convert screen coordinates to normalized device coordinates (-1 to +1)
    glm::vec2 ndc;
    ndc.x = (2.0f * screen_pos.x) / viewport_size.x - 1.0f;
    ndc.y = 1.0f - (2.0f * screen_pos.y) / viewport_size.y; // Y is flipped in screen coords
    return ndc;
}

glm::vec2 InteractionTool::GetMousePosition(const InputEvent& event) const {
    if (event.IsMouseEvent()) {
        return event.GetScreenPosition();
    }
    return glm::vec2(0.0f, 0.0f);
}

bool InteractionTool::HasModifiers(const InputEvent& event, ModifierKeys modifiers) const {
    const auto& event_modifiers = event.GetModifiers();
    return (modifiers.ctrl ? event_modifiers.ctrl : true) &&
           (modifiers.shift ? event_modifiers.shift : true) &&
           (modifiers.alt ? event_modifiers.alt : true) &&
           (modifiers.super ? event_modifiers.super : true);
}

// === ToolManager Implementation ===

ToolManager::ToolManager(SceneManager* scene_manager) 
    : scene_manager_(scene_manager) {
    if (!scene_manager_) {
        throw std::invalid_argument("ToolManager: scene_manager cannot be null");
    }
}

void ToolManager::RegisterTool(std::shared_ptr<InteractionTool> tool) {
    if (!tool) {
        std::cerr << "Warning: Attempted to register null tool" << std::endl;
        return;
    }
    
    // Check if tool with same name already exists
    auto existing = GetTool(tool->GetName());
    if (existing) {
        std::cerr << "Warning: Tool '" << tool->GetName() << "' already registered, replacing" << std::endl;
        UnregisterTool(tool->GetName());
    }
    
    tools_.push_back(tool);
    std::cout << "Registered tool: " << tool->GetDisplayName() << std::endl;
}

void ToolManager::UnregisterTool(const std::string& name) {
    auto it = std::find_if(tools_.begin(), tools_.end(),
        [&name](const auto& tool) { return tool->GetName() == name; });
    
    if (it != tools_.end()) {
        // Deactivate if this was the active tool
        if (active_tool_ && active_tool_->GetName() == name) {
            DeactivateCurrentTool();
        }
        
        std::cout << "Unregistered tool: " << (*it)->GetDisplayName() << std::endl;
        tools_.erase(it);
    }
}

std::shared_ptr<InteractionTool> ToolManager::GetTool(const std::string& name) {
    auto it = std::find_if(tools_.begin(), tools_.end(),
        [&name](const auto& tool) { return tool->GetName() == name; });
    
    return (it != tools_.end()) ? *it : nullptr;
}

bool ToolManager::ActivateTool(const std::string& name) {
    auto tool = GetTool(name);
    if (!tool) {
        std::cerr << "Error: Tool '" << name << "' not found" << std::endl;
        return false;
    }
    
    // Deactivate current tool if different
    if (active_tool_ && active_tool_ != tool) {
        active_tool_->OnDeactivate();
    }
    
    // Activate new tool
    active_tool_ = tool;
    active_tool_->OnActivate();
    
    return true;
}

void ToolManager::DeactivateCurrentTool() {
    if (active_tool_) {
        active_tool_->OnDeactivate();
        active_tool_.reset();
    }
}

std::vector<std::shared_ptr<InteractionTool>> ToolManager::GetAllTools() const {
    return tools_;
}

void ToolManager::RenderActiveTool(const glm::mat4& projection, const glm::mat4& view) {
    if (active_tool_ && active_tool_->GetState() != InteractionTool::State::kInactive) {
        active_tool_->OnRender(projection, view);
    }
}

void ToolManager::OnToolStateChanged(InteractionTool* tool, 
                                   InteractionTool::State old_state, 
                                   InteractionTool::State new_state) {
    if (state_callback_) {
        state_callback_(tool, old_state, new_state);
    }
}

} // namespace quickviz