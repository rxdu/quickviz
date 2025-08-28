/*
 * @file virtual_sphere.cpp  
 * @date August 27, 2025
 * @brief Implementation of VirtualSphere
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/virtual_sphere.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

namespace quickviz {

VirtualSphere::VirtualSphere(const std::string& id, float radius)
    : VirtualObject(id), radius_(radius) {
    SetType(VirtualObjectType::Sphere);
    UpdateBounds();
}

void VirtualSphere::SetRadius(float radius) {
    if (radius_ != radius) {
        radius_ = std::max(0.01f, radius); // Minimum radius
        UpdateBounds();
        MarkDirty();
    }
}

void VirtualSphere::SetTessellation(int tessellation) {
    if (tessellation_ != tessellation) {
        tessellation_ = std::max(4, tessellation); // Minimum tessellation
        MarkDirty();
    }
}

void VirtualSphere::SetPosition(const glm::vec3& position) {
    VirtualObject::SetPosition(position); // Call base implementation
    UpdateBounds(); // Update bounds after position change
}

BoundingBox VirtualSphere::GetBounds() const {
    return bounds_;
}

// Ray-casting HitTest removed - using GPU ID-buffer selection exclusively

void VirtualSphere::UpdateBackend(RenderInterface* backend) {
    if (backend && IsBackendUpdateNeeded()) {
        backend->UpdateObject(GetId(), GetBackendData());
        ClearBackendUpdateFlag();
    }
}

void VirtualSphere::RemoveFromBackend(RenderInterface* backend) {
    if (backend) {
        backend->RemoveObject(GetId());
    }
}

VirtualObjectData VirtualSphere::GetBackendData() const {
    VirtualObjectData data = VirtualObject::GetBackendData();
    
    // Set sphere-specific geometry parameters
    data.geometry.radius = radius_;
    data.geometry.tessellation = tessellation_;
    
    return data;
}

void VirtualSphere::UpdateBounds() {
    // Get position from transform
    glm::vec3 center = glm::vec3(GetState().transform[3]);
    
    // Simple sphere bounding box
    glm::vec3 radius_vec(radius_);
    bounds_.min = center - radius_vec;
    bounds_.max = center + radius_vec;
}

} // namespace quickviz