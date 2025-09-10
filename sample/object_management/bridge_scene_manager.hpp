/*
 * @file bridge_scene_manager.hpp
 * @date Sep 10, 2025
 * @brief Scene manager that demonstrates SceneManagerBridge integration
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BRIDGE_SCENE_MANAGER_HPP
#define QUICKVIZ_BRIDGE_SCENE_MANAGER_HPP

#include "gldraw/gl_scene_panel.hpp"
#include "scenegraph/integration/scene_manager_bridge.hpp"
#include <memory>
#include <string>

namespace quickviz {
class BridgeControlPanel;

/**
 * @brief Scene manager that uses SceneManagerBridge for state management
 * 
 * This class demonstrates how to integrate the SceneManagerBridge with
 * existing OpenGL rendering while providing undo/redo capabilities.
 */
class BridgeSceneManager : public GlScenePanel {
public:
    BridgeSceneManager(const std::string& name, SceneManager::Mode mode = SceneManager::Mode::k3D);
    
    /**
     * @brief Get the bridge for external control
     * @return Pointer to the scene manager bridge
     */
    SceneManagerBridge* GetBridge() { return bridge_.get(); }
    const SceneManagerBridge* GetBridge() const { return bridge_.get(); }
    
    /**
     * @brief Set control panel for bidirectional communication
     */
    void SetControlPanel(BridgeControlPanel* panel) { control_panel_ = panel; }
    
    void Draw() override;

private:
    std::unique_ptr<SceneManagerBridge> bridge_;
    BridgeControlPanel* control_panel_ = nullptr;
    
    // Demo state
    float demo_rotation_angle_ = 0.0f;
    
    // Status overlay refresh
    float status_refresh_timer_ = 0.0f;
    const float kStatusRefreshInterval = 0.1f;  // 10 FPS refresh rate for status
    
    // Cached status text to avoid per-frame string operations
    std::string cached_mode_text_;
    std::string cached_undo_text_;
    std::string cached_stats_text_;
    ImU32 cached_mode_color_ = IM_COL32(255, 255, 255, 255);
    
    void UpdateDemoAnimations();
    void DrawStatusOverlay();
};

} // namespace quickviz

#endif // QUICKVIZ_BRIDGE_SCENE_MANAGER_HPP