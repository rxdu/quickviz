/*
 * @file virtual_scene_panel.hpp
 * @date August 27, 2025
 * @brief Virtual scene panel for ImGui integration
 * 
 * VirtualScenePanel integrates the virtual scene system with ImGui,
 * handling input events and providing the UI interface for 3D scene
 * interaction. This is the evolution of SceneViewPanel for virtual scenes.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VIRTUAL_SCENE_PANEL_HPP
#define QUICKVIZ_VIRTUAL_SCENE_PANEL_HPP

#include <memory>
#include <string>

#include "imview/panel.hpp"
#include "vscene/virtual_scene.hpp"

namespace quickviz {

/**
 * @brief ImGui panel for virtual scene interaction
 * 
 * VirtualScenePanel provides the UI integration layer for virtual scenes.
 * It handles ImGui input events, converts them to virtual scene operations,
 * and renders the scene using the configured backend.
 * 
 * This is the evolution of SceneViewPanel specifically designed for
 * virtual scene interaction and high-level object manipulation.
 */
class VirtualScenePanel : public Panel {
public:
    explicit VirtualScenePanel(const std::string& name);
    ~VirtualScenePanel() override;

    // Panel interface
    void Draw() override;
    
    /**
     * @brief Render content without Begin/End calls
     */
    void RenderInsideWindow();

    // Virtual scene access
    VirtualScene* GetVirtualScene() const { return virtual_scene_.get(); }
    EventDispatcher* GetEventDispatcher() const { return virtual_scene_->GetEventDispatcher(); }

    // Backend management
    void SetRenderBackend(std::unique_ptr<RenderInterface> backend);
    RenderInterface* GetRenderBackend() const { return virtual_scene_->GetRenderBackend(); }

    // Configuration
    struct Config {
        bool show_selection_outline = true;
        bool show_hover_feedback = true;
        bool enable_multi_selection = true;
        bool show_debug_info = false;
        glm::vec3 selection_color = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow
        glm::vec3 hover_color = glm::vec3(0.0f, 1.0f, 1.0f);      // Cyan
    };

    void SetConfig(const Config& config) { config_ = config; }
    const Config& GetConfig() const { return config_; }

    // Convenience methods (delegate to virtual scene)
    void AddObject(const std::string& id, std::unique_ptr<VirtualObject> object) {
        virtual_scene_->AddObject(id, std::move(object));
    }
    
    VirtualObject* GetObject(const std::string& id) const {
        return virtual_scene_->GetObject(id);
    }
    
    std::vector<std::string> GetSelectedIds() const {
        return virtual_scene_->GetSelectedIds();
    }

protected:
    /**
     * @brief Handle ImGui input events and convert to virtual scene operations
     */
    void HandleInput();
    
    /**
     * @brief Render debug information overlay
     */
    void RenderDebugOverlay();
    
    /**
     * @brief Update interaction state (hover, selection feedback)
     */
    void UpdateInteractionState();

private:
    std::unique_ptr<VirtualScene> virtual_scene_;
    Config config_;
    
    // Input state tracking
    struct InputState {
        glm::vec2 last_mouse_pos;
        bool dragging = false;
        VirtualObject* drag_object = nullptr;
        glm::vec3 drag_start_world_pos;
    };
    InputState input_state_;
    
    // Helper methods
    glm::vec2 GetLocalMousePosition() const;
    bool IsMouseInContentArea() const;
    void ProcessMouseClick(int button);
    void ProcessMouseDrag();
    void ProcessMouseHover();
    
    // Event helpers
    void DispatchClickEvent(VirtualObject* object, int button);
    void DispatchDragEvent(VirtualObject* object);
    void DispatchHoverEvent(VirtualObject* object, bool entering);
};

} // namespace quickviz

#endif // QUICKVIZ_VIRTUAL_SCENE_PANEL_HPP