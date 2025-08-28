/*
 * @file gl_render_backend.hpp
 * @date August 27, 2025
 * @brief Abstract rendering backend interface for virtual scene
 * 
 * The render backend abstracts the underlying rendering system (OpenGL, Vulkan, etc.)
 * allowing virtual objects to be rendered using different technologies while maintaining
 * the same application-level interface.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_RENDER_BACKEND_HPP
#define QUICKVIZ_RENDER_BACKEND_HPP

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <glm/glm.hpp>

#include "vscene/render_interface.hpp"
#include "vscene/virtual_object_types.hpp"

namespace quickviz {

// Forward declarations  
class OpenGlObject;
class GlSceneManager;
class Sphere;

/**
 * @brief OpenGL-based render backend implementation
 * 
 * GlDrawBackend wraps the existing GlSceneManager to provide virtual scene
 * integration. It creates appropriate OpenGL objects for virtual object types
 * and manages the mapping between virtual objects and render objects.
 */
class GlRenderBackend : public RenderInterface {
public:
    GlRenderBackend();  // Creates its own GlSceneManager
    explicit GlRenderBackend(GlSceneManager* external_scene_manager);  // Uses existing GlSceneManager
    ~GlRenderBackend() override;

    // RenderInterface interface
    void CreateObject(const std::string& id, VirtualObjectType type, 
                     const VirtualObjectData& initial_data) override;
    void UpdateObject(const std::string& id, const VirtualObjectData& data) override;
    void RemoveObject(const std::string& id) override;
    void ClearAllObjects() override;
    
    void RenderToFramebuffer(float width, float height) override;
    uint32_t GetFramebufferTexture() const override;
    
    std::string PickObjectAt(float screen_x, float screen_y) override;
    
    void SetBackgroundColor(float r, float g, float b, float a) override;

    // Access to underlying scene manager (for camera control, etc.)
    GlSceneManager* GetSceneManager() const { return GetActiveSceneManager(); }

private:
    std::unique_ptr<GlSceneManager> scene_manager_;  // Owned scene manager (when owns_scene_manager_ = true)
    GlSceneManager* external_scene_manager_ = nullptr;  // External scene manager (when owns_scene_manager_ = false)
    bool owns_scene_manager_ = true;
    
    // Mapping from virtual object types to OpenGL objects
    std::map<std::string, std::string> virtual_to_gl_object_map_;  // virtual_id -> gl_object_name
    
    // Helper to get the active scene manager
    GlSceneManager* GetActiveSceneManager() const {
        return owns_scene_manager_ ? scene_manager_.get() : external_scene_manager_;
    }
    
    // Helper methods for object creation
    std::unique_ptr<OpenGlObject> CreateGlObjectForType(VirtualObjectType type, 
                                                        const VirtualObjectData& data);
    void UpdateGlObjectFromData(OpenGlObject* gl_object, VirtualObjectType type,
                               const VirtualObjectData& data);
    
    // Type-specific update methods
    void UpdateSphereFromData(Sphere* sphere, const VirtualObjectData& data);
};

} // namespace quickviz

#endif // QUICKVIZ_RENDER_BACKEND_HPP