/*
 * @file virtual_scene.cpp
 * @date August 27, 2025
 * @brief Implementation of VirtualScene class
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/virtual_scene.hpp"
#include <algorithm>
#include <glm/gtc/quaternion.hpp>

namespace quickviz {

VirtualScene::VirtualScene() = default;
VirtualScene::~VirtualScene() = default;

void VirtualScene::SetRenderBackend(std::unique_ptr<RenderInterface> backend) {
    backend_ = std::move(backend);
}

void VirtualScene::AddObject(const std::string& id, std::unique_ptr<VirtualObject> object) {
    if (!object || id.empty()) return;
    
    // Remove existing object with same ID
    RemoveObject(id);
    
    // Add to backend if available
    if (backend_) {
        backend_->CreateObject(id, object->GetType(), object->GetBackendData());
    }
    
    // Store object
    objects_[id] = std::move(object);
}

void VirtualScene::RemoveObject(const std::string& id) {
    auto it = objects_.find(id);
    if (it != objects_.end()) {
        // Remove from backend
        if (backend_) {
            it->second->RemoveFromBackend(backend_.get());
        }
        
        // Remove from selection if selected
        selected_objects_.erase(id);
        
        // Clear hover state if this object was hovered
        if (hovered_object_ == it->second.get()) {
            hovered_object_ = nullptr;
        }
        
        // Remove from objects
        objects_.erase(it);
    }
}

VirtualObject* VirtualScene::GetObject(const std::string& id) const {
    auto it = objects_.find(id);
    return (it != objects_.end()) ? it->second.get() : nullptr;
}

std::vector<std::string> VirtualScene::GetObjectIds() const {
    std::vector<std::string> ids;
    ids.reserve(objects_.size());
    for (const auto& pair : objects_) {
        ids.push_back(pair.first);
    }
    return ids;
}

void VirtualScene::ClearObjects() {
    // Clear backend first
    if (backend_) {
        for (const auto& pair : objects_) {
            pair.second->RemoveFromBackend(backend_.get());
        }
    }
    
    // Clear internal state
    objects_.clear();
    selected_objects_.clear();
    hovered_object_ = nullptr;
}

// Selection management
void VirtualScene::SetSelected(const std::string& id, bool selected) {
    auto* object = GetObject(id);
    if (!object) return;
    
    if (selected) {
        if (!config_.multi_selection_enabled) {
            ClearSelection();
        }
        selected_objects_.insert(id);
    } else {
        selected_objects_.erase(id);
    }
    
    // Update object state
    object->SetSelected(selected);
    UpdateBackendForObject(object);
}

void VirtualScene::ClearSelection() {
    for (const std::string& id : selected_objects_) {
        if (auto* object = GetObject(id)) {
            object->SetSelected(false);
            UpdateBackendForObject(object);
        }
    }
    selected_objects_.clear();
}

void VirtualScene::SelectAll() {
    if (!config_.multi_selection_enabled && objects_.size() > 1) {
        return; // Can't select multiple objects
    }
    
    for (const auto& pair : objects_) {
        selected_objects_.insert(pair.first);
        pair.second->SetSelected(true);
        UpdateBackendForObject(pair.second.get());
    }
}

void VirtualScene::SelectNone() {
    ClearSelection();
}

std::vector<std::string> VirtualScene::GetSelectedIds() const {
    return std::vector<std::string>(selected_objects_.begin(), selected_objects_.end());
}

size_t VirtualScene::GetSelectedCount() const {
    return selected_objects_.size();
}

bool VirtualScene::IsSelected(const std::string& id) const {
    return selected_objects_.find(id) != selected_objects_.end();
}

void VirtualScene::AddToSelection(const std::string& id) {
    if (config_.multi_selection_enabled || selected_objects_.empty()) {
        SetSelected(id, true);
    }
}

void VirtualScene::RemoveFromSelection(const std::string& id) {
    SetSelected(id, false);
}

void VirtualScene::ToggleSelection(const std::string& id) {
    SetSelected(id, !IsSelected(id));
}

// Interaction
VirtualObject* VirtualScene::Pick(float screen_x, float screen_y) {
    if (!backend_) return nullptr;
    
    std::string picked_id = backend_->PickObjectAt(screen_x, screen_y);
    return GetObject(picked_id);
}

std::vector<VirtualObject*> VirtualScene::PickMultiple(float screen_x, float screen_y, float radius) {
    // For now, simple implementation - just pick one object
    // In future, could implement area-based picking
    std::vector<VirtualObject*> result;
    if (auto* picked = Pick(screen_x, screen_y)) {
        result.push_back(picked);
    }
    return result;
}

// Scene operations
void VirtualScene::Update(float delta_time) {
    // Update backend for any dirty objects
    if (backend_ && config_.auto_update_backend) {
        for (const auto& pair : objects_) {
            UpdateBackendForObject(pair.second.get());
        }
    }
    
    // Could add animation updates here in future
}

void VirtualScene::Render() {
    if (backend_) {
        // For now, render to a default size
        // In real usage, size would come from the panel/window
        backend_->RenderToFramebuffer(800.0f, 600.0f);
    }
}

// Utility operations
BoundingBox VirtualScene::GetSceneBounds() const {
    if (objects_.empty()) return BoundingBox{};
    
    BoundingBox scene_bounds;
    bool first = true;
    
    for (const auto& pair : objects_) {
        BoundingBox obj_bounds = pair.second->GetBounds();
        if (first) {
            scene_bounds = obj_bounds;
            first = false;
        } else {
            scene_bounds.min = glm::min(scene_bounds.min, obj_bounds.min);
            scene_bounds.max = glm::max(scene_bounds.max, obj_bounds.max);
        }
    }
    
    return scene_bounds;
}

BoundingBox VirtualScene::GetSelectionBounds() const {
    if (selected_objects_.empty()) return BoundingBox{};
    
    BoundingBox selection_bounds;
    bool first = true;
    
    for (const std::string& id : selected_objects_) {
        if (auto* object = GetObject(id)) {
            BoundingBox obj_bounds = object->GetBounds();
            if (first) {
                selection_bounds = obj_bounds;
                first = false;
            } else {
                selection_bounds.min = glm::min(selection_bounds.min, obj_bounds.min);
                selection_bounds.max = glm::max(selection_bounds.max, obj_bounds.max);
            }
        }
    }
    
    return selection_bounds;
}

glm::vec3 VirtualScene::GetSelectionCentroid() const {
    return GetSelectionBounds().GetCenter();
}

// Transform operations on selection
void VirtualScene::TranslateSelection(const glm::vec3& delta) {
    for (const std::string& id : selected_objects_) {
        if (auto* object = GetObject(id)) {
            // Get current position and add delta
            glm::vec4 current_pos = object->GetState().transform[3];
            glm::vec3 new_position = glm::vec3(current_pos) + delta;
            object->SetPosition(new_position);
            UpdateBackendForObject(object);
        }
    }
}

void VirtualScene::RotateSelection(const glm::quat& rotation, const glm::vec3& center) {
    for (const std::string& id : selected_objects_) {
        if (auto* object = GetObject(id)) {
            // Get current position relative to center
            glm::vec4 current_pos = object->GetState().transform[3];
            glm::vec3 relative_pos = glm::vec3(current_pos) - center;
            
            // Rotate position around center  
            glm::vec3 rotated_pos = rotation * relative_pos;
            glm::vec3 new_position = center + rotated_pos;
            
            // Apply rotation to object
            object->SetPosition(new_position);
            object->SetRotation(rotation); // This should compose with existing rotation
            UpdateBackendForObject(object);
        }
    }
}

void VirtualScene::ScaleSelection(const glm::vec3& scale, const glm::vec3& center) {
    for (const std::string& id : selected_objects_) {
        if (auto* object = GetObject(id)) {
            // Get current position relative to center
            glm::vec4 current_pos = object->GetState().transform[3];
            glm::vec3 relative_pos = glm::vec3(current_pos) - center;
            
            // Scale position around center
            glm::vec3 scaled_pos = relative_pos * scale;
            glm::vec3 new_position = center + scaled_pos;
            
            // Apply scale to object
            object->SetPosition(new_position);
            object->SetScale(scale); // This should compose with existing scale
            UpdateBackendForObject(object);
        }
    }
}

// Private methods
void VirtualScene::UpdateBackendForObject(VirtualObject* object) {
    if (object && backend_) {
        object->UpdateBackend(backend_.get());
    }
}

void VirtualScene::UpdateHoverState(float screen_x, float screen_y) {
    // Clear previous hover
    if (hovered_object_) {
        hovered_object_->SetHovered(false);
        UpdateBackendForObject(hovered_object_);
        hovered_object_ = nullptr;
    }
    
    // Set new hover
    if (config_.enable_hover_tracking) {
        hovered_object_ = Pick(screen_x, screen_y);
        if (hovered_object_) {
            hovered_object_->SetHovered(true);
            UpdateBackendForObject(hovered_object_);
        }
    }
}

void VirtualScene::DispatchEvent(const VirtualEvent& event) {
    // TODO: Implement event dispatching in Step 4
    // event_dispatcher_.Dispatch(event);
}

// Interaction processing methods (for VirtualScenePanel integration)
void VirtualScene::ProcessClick(float screen_x, float screen_y, int button) {
    VirtualObject* clicked_object = Pick(screen_x, screen_y);
    if (clicked_object && clicked_object->OnClick) {
        glm::vec2 screen_pos(screen_x, screen_y);
        glm::vec3 world_pos(0.0f); // TODO: Convert screen to world coordinates
        clicked_object->OnClick(clicked_object, screen_pos, world_pos);
    }
}

void VirtualScene::ProcessDrag(float screen_x, float screen_y, const glm::vec2& delta) {
    if (hovered_object_ && hovered_object_->OnDrag) {
        glm::vec3 world_delta(delta.x, delta.y, 0.0f); // TODO: Convert screen delta to world delta
        hovered_object_->OnDrag(hovered_object_, world_delta);
    }
}

void VirtualScene::ProcessHover(float screen_x, float screen_y) {
    VirtualObject* previous_hovered = hovered_object_;
    UpdateHoverState(screen_x, screen_y);
    
    // Handle hover exit
    if (previous_hovered && previous_hovered != hovered_object_) {
        if (previous_hovered->OnHover) {
            previous_hovered->OnHover(previous_hovered, false); // exiting hover
        }
    }
    
    // Handle hover enter
    if (hovered_object_ && hovered_object_ != previous_hovered) {
        if (hovered_object_->OnHover) {
            hovered_object_->OnHover(hovered_object_, true); // entering hover
        }
    }
}

// Validation helpers
bool VirtualScene::IsValidObject(const std::string& id) const {
    return !id.empty() && objects_.find(id) != objects_.end();
}

void VirtualScene::ValidateSelection() {
    // Remove any selected objects that no longer exist
    auto it = selected_objects_.begin();
    while (it != selected_objects_.end()) {
        if (!IsValidObject(*it)) {
            it = selected_objects_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace quickviz