/*
 * @file virtual_object.cpp
 * @date August 27, 2025
 * @brief Implementation of VirtualObject base class
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/virtual_object.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace quickviz {

void VirtualObject::SetTransform(const glm::mat4& transform) {
    state_.transform = transform;
    MarkDirty();
}

void VirtualObject::SetPosition(const glm::vec3& position) {
    // Extract current rotation and scale, set new position
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    // For simplicity, assume uniform scale and extract scale from first column
    glm::vec3 scale = glm::vec3(glm::length(glm::vec3(state_.transform[0])),
                                glm::length(glm::vec3(state_.transform[1])), 
                                glm::length(glm::vec3(state_.transform[2])));
    
    // Rebuild transform with new position, keeping rotation and scale
    glm::mat3 rotation_scale = glm::mat3(state_.transform);
    state_.transform = translation * glm::mat4(rotation_scale);
    MarkDirty();
}

void VirtualObject::SetRotation(const glm::quat& rotation) {
    glm::vec3 position = glm::vec3(state_.transform[3]);
    glm::vec3 scale = glm::vec3(glm::length(glm::vec3(state_.transform[0])),
                                glm::length(glm::vec3(state_.transform[1])),
                                glm::length(glm::vec3(state_.transform[2])));
    
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 rotation_matrix = glm::mat4_cast(rotation);
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), scale);
    
    state_.transform = translation * rotation_matrix * scale_matrix;
    MarkDirty();
}

void VirtualObject::SetScale(const glm::vec3& scale) {
    glm::vec3 position = glm::vec3(state_.transform[3]);
    
    // Extract rotation by normalizing the rotation+scale part
    glm::mat3 rotation_scale = glm::mat3(state_.transform);
    glm::vec3 current_scale = glm::vec3(glm::length(glm::vec3(rotation_scale[0])),
                                        glm::length(glm::vec3(rotation_scale[1])),
                                        glm::length(glm::vec3(rotation_scale[2])));
    glm::mat3 rotation = glm::mat3(rotation_scale[0] / current_scale[0],
                                   rotation_scale[1] / current_scale[1],
                                   rotation_scale[2] / current_scale[2]);
    
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 rotation_matrix = glm::mat4(rotation);
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), scale);
    
    state_.transform = translation * rotation_matrix * scale_matrix;
    MarkDirty();
}

void VirtualObject::SetVisible(bool visible) {
    if (state_.visible != visible) {
        state_.visible = visible;
        MarkDirty();
    }
}

void VirtualObject::SetSelected(bool selected) {
    if (state_.selected != selected) {
        state_.selected = selected;
        MarkDirty();
    }
}

void VirtualObject::SetHovered(bool hovered) {
    if (state_.hovered != hovered) {
        state_.hovered = hovered;
        MarkDirty();
    }
}

void VirtualObject::SetColor(const glm::vec3& color) {
    if (state_.color != color) {
        state_.color = color;
        MarkDirty();
    }
}

VirtualObjectData VirtualObject::GetBackendData() const {
    VirtualObjectData data;
    data.transform = state_.transform;
    data.visible = state_.visible;
    data.color = state_.color;
    data.alpha = state_.alpha;
    data.highlighted = state_.selected || state_.hovered;
    return data;
}

// BoundingBox utility methods
bool BoundingBox::Contains(const glm::vec3& point) const {
    return point.x >= min.x && point.x <= max.x &&
           point.y >= min.y && point.y <= max.y &&
           point.z >= min.z && point.z <= max.z;
}

// Ray-casting Intersects removed - using GPU ID-buffer selection exclusively

} // namespace quickviz