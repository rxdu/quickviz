/*
 * @file gl_render_backend.cpp
 * @date August 27, 2025
 * @brief OpenGL render backend implementation
 *
 * Concrete implementation of RenderInterface using GlSceneManager
 * and OpenGL objects from the gldraw module.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/gl_render_backend.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/interface/opengl_object.hpp"
#include <memory>

namespace quickviz {

GlRenderBackend::GlRenderBackend() {
    // Create the underlying GlSceneManager
    scene_manager_ = std::make_unique<GlSceneManager>("VirtualScene3D", GlSceneManager::Mode::k3D);
    owns_scene_manager_ = true;
}

GlRenderBackend::GlRenderBackend(GlSceneManager* external_scene_manager)
    : external_scene_manager_(external_scene_manager), owns_scene_manager_(false) {
    // Use the provided external scene manager
}

GlRenderBackend::~GlRenderBackend() = default;

void GlRenderBackend::CreateObject(const std::string& id, VirtualObjectType type, 
                                   const VirtualObjectData& initial_data) {
    // Create the appropriate OpenGL object based on type
    auto gl_object = CreateGlObjectForType(type, initial_data);
    if (!gl_object) {
        return; // Unsupported type
    }
    
    // Add to scene manager
    GetActiveSceneManager()->AddOpenGLObject(id, std::move(gl_object));
    
    // Store the mapping
    virtual_to_gl_object_map_[id] = id; // For now, use same name
}

void GlRenderBackend::UpdateObject(const std::string& id, const VirtualObjectData& data) {
    // Get the OpenGL object
    OpenGlObject* gl_object = GetActiveSceneManager()->GetOpenGLObject(id);
    if (!gl_object) {
        return; // Object doesn't exist
    }
    
    // We need to determine the type to know how to update it
    // For now, we'll try to cast to known types
    // In a more sophisticated implementation, we'd store type information
    
    // Try Sphere first
    if (auto* sphere = dynamic_cast<Sphere*>(gl_object)) {
        UpdateSphereFromData(sphere, data);
    }
    // Add more types as they become available
}

void GlRenderBackend::RemoveObject(const std::string& id) {
    GetActiveSceneManager()->RemoveOpenGLObject(id);
    virtual_to_gl_object_map_.erase(id);
}

void GlRenderBackend::ClearAllObjects() {
    GetActiveSceneManager()->ClearOpenGLObjects();
    virtual_to_gl_object_map_.clear();
}

void GlRenderBackend::RenderToFramebuffer(float width, float height) {
    GetActiveSceneManager()->RenderToFramebuffer(width, height);
}

uint32_t GlRenderBackend::GetFramebufferTexture() const {
    return GetActiveSceneManager()->GetFramebufferTexture();
}

std::string GlRenderBackend::PickObjectAt(float screen_x, float screen_y) {
    // For now, return empty string - object picking not implemented in GlSceneManager
    // This would require adding ID-based picking to GlSceneManager
    return "";
}

// Ray-casting methods removed - using GPU ID-buffer selection exclusively

void GlRenderBackend::SetBackgroundColor(float r, float g, float b, float a) {
    GetActiveSceneManager()->SetBackgroundColor(r, g, b, a);
}

// Private helper methods

std::unique_ptr<OpenGlObject> GlRenderBackend::CreateGlObjectForType(VirtualObjectType type, 
                                                                     const VirtualObjectData& data) {
    switch (type) {
        case VirtualObjectType::Sphere: {
            // Extract position from transform matrix
            glm::vec3 position = glm::vec3(data.transform[3]);
            float radius = data.geometry.radius;
            
            auto sphere = std::make_unique<Sphere>(position, radius);
            UpdateSphereFromData(sphere.get(), data);
            return sphere;
        }
        
        // Add more types as needed
        case VirtualObjectType::Box:
        case VirtualObjectType::Cylinder:
        default:
            // Unsupported type for now
            return nullptr;
    }
}

void GlRenderBackend::UpdateGlObjectFromData(OpenGlObject* gl_object, VirtualObjectType type,
                                           const VirtualObjectData& data) {
    switch (type) {
        case VirtualObjectType::Sphere: {
            if (auto* sphere = dynamic_cast<Sphere*>(gl_object)) {
                UpdateSphereFromData(sphere, data);
            }
            break;
        }
        // Add more types as needed
        default:
            break;
    }
}

void GlRenderBackend::UpdateSphereFromData(Sphere* sphere, const VirtualObjectData& data) {
    if (!sphere) return;
    
    // Update transform
    sphere->SetTransform(data.transform);
    
    // Update radius from geometry data
    sphere->SetRadius(data.geometry.radius);
    
    // Update appearance
    sphere->SetColor(data.color);
    sphere->SetOpacity(data.alpha);
    
    // Handle visibility by setting opacity
    if (!data.visible) {
        sphere->SetOpacity(0.0f);
    }
    
    // Handle highlighting - make it brighter or use wireframe
    if (data.highlighted) {
        // Make it brighter by increasing the color values
        glm::vec3 highlight_color = glm::min(data.color * 1.5f, glm::vec3(1.0f));
        sphere->SetColor(highlight_color);
        
        // Also enable wireframe overlay for highlighting
        sphere->SetRenderMode(GeometricPrimitive::RenderMode::kSolid);
        sphere->SetWireframeColor(glm::vec3(1.0f, 1.0f, 0.0f)); // Yellow wireframe
    } else {
        sphere->SetRenderMode(GeometricPrimitive::RenderMode::kSolid);
    }
}

} // namespace quickviz