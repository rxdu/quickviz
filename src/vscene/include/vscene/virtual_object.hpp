/*
 * @file virtual_object.hpp
 * @date August 27, 2025
 * @brief Base class for all virtual scene objects
 * 
 * Virtual objects represent application-level scene entities that can be
 * interacted with, selected, and manipulated. They delegate rendering to
 * the backend while maintaining their own state and interaction logic.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VIRTUAL_OBJECT_HPP
#define QUICKVIZ_VIRTUAL_OBJECT_HPP

#include <string>
#include <functional>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "vscene/render_interface.hpp"
#include "vscene/virtual_object_types.hpp"

namespace quickviz {

// Forward declarations
struct Ray;
struct BoundingBox;
struct VirtualObjectData;

/**
 * @brief Base class for all virtual scene objects
 * 
 * VirtualObject represents the application-level abstraction of scene entities.
 * It maintains state (transform, visibility, selection) and handles interaction
 * events while delegating actual rendering to the backend.
 * 
 * Key responsibilities:
 * - Transform and visibility management
 * - Interaction state (selected, hovered, etc.)
 * - Hit testing for selection
 * - Event callbacks for application integration
 * - Backend synchronization (when data changes)
 */
class VirtualObject {
public:
    // Object state management
    struct State {
        glm::mat4 transform = glm::mat4(1.0f);
        bool visible = true;
        bool selected = false;
        bool hovered = false;
        glm::vec3 color = glm::vec3(1.0f);
        float alpha = 1.0f;
    };

    // Event callback types
    using ClickCallback = std::function<void(VirtualObject*, glm::vec2 screen_pos, glm::vec3 world_pos)>;
    using DragCallback = std::function<void(VirtualObject*, glm::vec3 world_delta)>;
    using HoverCallback = std::function<void(VirtualObject*, bool entering)>;

public:
    VirtualObject(const std::string& id) : id_(id) {}
    virtual ~VirtualObject() = default;

    // Identity
    const std::string& GetId() const { return id_; }
    VirtualObjectType GetType() const { return type_; }
    const char* GetTypeString() const { return ToString(type_); }

    // State access
    const State& GetState() const { return state_; }
    void SetTransform(const glm::mat4& transform);
    virtual void SetPosition(const glm::vec3& position);
    void SetRotation(const glm::quat& rotation);
    void SetScale(const glm::vec3& scale);
    void SetVisible(bool visible);
    void SetSelected(bool selected);
    void SetHovered(bool hovered);
    void SetColor(const glm::vec3& color);

    // Interaction interface (pure virtual - must be implemented)
    virtual BoundingBox GetBounds() const = 0;
    virtual bool HitTest(const Ray& ray, float& distance) const = 0;
    
    // Backend synchronization (pure virtual)
    virtual void UpdateBackend(RenderInterface* backend) = 0;
    virtual void RemoveFromBackend(RenderInterface* backend) = 0;
    
    // Convert object state to backend data format
    virtual VirtualObjectData GetBackendData() const;

    // Event callbacks
    ClickCallback OnClick;
    DragCallback OnDrag;  
    HoverCallback OnHover;
    
    // Backend update tracking (public for testing)
    bool IsBackendUpdateNeeded() const { return needs_backend_update_; }
    void ClearBackendUpdateFlag() { needs_backend_update_ = false; }

protected:
    // For derived classes to set their type
    void SetType(VirtualObjectType type) { type_ = type; }
    
    // Mark object as needing backend update
    void MarkDirty() { needs_backend_update_ = true; }

private:
    std::string id_;
    VirtualObjectType type_ = VirtualObjectType::Custom;
    State state_;
    bool needs_backend_update_ = true; // Initially needs update
};

// Common geometric types
// Note: Ray is now defined in render_interface.hpp

struct BoundingBox {
    glm::vec3 min = glm::vec3(0.0f);
    glm::vec3 max = glm::vec3(0.0f);
    
    bool Contains(const glm::vec3& point) const;
    bool Intersects(const Ray& ray, float& distance) const;
    glm::vec3 GetCenter() const { return (min + max) * 0.5f; }
    glm::vec3 GetSize() const { return max - min; }
};

} // namespace quickviz

#endif // QUICKVIZ_VIRTUAL_OBJECT_HPP