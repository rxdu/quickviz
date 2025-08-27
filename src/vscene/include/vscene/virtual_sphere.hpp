/*
 * @file virtual_sphere.hpp
 * @date August 27, 2025
 * @brief Virtual sphere object implementation
 * 
 * VirtualSphere demonstrates the virtual object interface with a simple
 * sphere primitive. This shows how geometric objects integrate with the
 * virtual scene system.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VIRTUAL_SPHERE_HPP
#define QUICKVIZ_VIRTUAL_SPHERE_HPP

#include "vscene/virtual_object.hpp"

namespace quickviz {

/**
 * @brief Virtual sphere object
 * 
 * VirtualSphere represents a 3D sphere in the virtual scene. It demonstrates
 * the virtual object pattern with geometric properties, hit testing, and
 * backend synchronization with the existing gldraw Sphere class.
 */
class VirtualSphere : public VirtualObject {
public:
    explicit VirtualSphere(const std::string& id, float radius = 1.0f);
    ~VirtualSphere() override = default;

    // Sphere-specific properties
    float GetRadius() const { return radius_; }
    void SetRadius(float radius);
    
    int GetTessellation() const { return tessellation_; }
    void SetTessellation(int tessellation);
    
    // Override position setting to update bounds
    void SetPosition(const glm::vec3& position) override;

    // VirtualObject interface implementation
    BoundingBox GetBounds() const override;
    bool HitTest(const Ray& ray, float& distance) const override;
    void UpdateBackend(RenderInterface* backend) override;
    void RemoveFromBackend(RenderInterface* backend) override;
    VirtualObjectData GetBackendData() const override;

private:
    float radius_;
    int tessellation_ = 16; // Number of subdivisions
    
    // Helper methods
    void UpdateBounds();
    BoundingBox bounds_;
};

/**
 * @brief Factory function for creating virtual spheres
 */
inline std::unique_ptr<VirtualSphere> CreateVirtualSphere(const std::string& id, float radius = 1.0f) {
    return std::make_unique<VirtualSphere>(id, radius);
}

} // namespace quickviz

#endif // QUICKVIZ_VIRTUAL_SPHERE_HPP