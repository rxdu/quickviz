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

bool VirtualSphere::HitTest(const Ray& ray, float& distance) const {
    // Get sphere center from transform
    glm::vec3 center = glm::vec3(GetState().transform[3]);
    
    // Ray-sphere intersection
    glm::vec3 oc = ray.origin - center;
    float a = glm::dot(ray.direction, ray.direction);
    float b = 2.0f * glm::dot(oc, ray.direction);
    float c = glm::dot(oc, oc) - radius_ * radius_;
    
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;
    
    float sqrt_discriminant = sqrt(discriminant);
    float t1 = (-b - sqrt_discriminant) / (2.0f * a);
    float t2 = (-b + sqrt_discriminant) / (2.0f * a);
    
    // Return closest positive intersection
    if (t1 > 0) {
        distance = t1;
        return true;
    } else if (t2 > 0) {
        distance = t2;
        return true;
    }
    
    return false;
}

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