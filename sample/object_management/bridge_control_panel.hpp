/*
 * @file bridge_control_panel.hpp
 * @date Sep 10, 2025
 * @brief Control panel for testing SceneManagerBridge functionality
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BRIDGE_CONTROL_PANEL_HPP
#define QUICKVIZ_BRIDGE_CONTROL_PANEL_HPP

#include "imview/panel.hpp"
#include "scenegraph/integration/scene_manager_bridge.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/mesh.hpp"
#include <memory>
#include <vector>

namespace quickviz {

/**
 * @brief Comprehensive control panel for testing SceneManagerBridge
 * 
 * Provides interactive controls for:
 * - Operation mode switching
 * - Object manipulation and transformation
 * - Undo/redo operations
 * - Statistics monitoring
 * - Dynamic object creation/deletion
 */
class BridgeControlPanel : public Panel {
public:
    BridgeControlPanel(const std::string& name, SceneManagerBridge* bridge);
    
    void Draw() override;
    
    /**
     * @brief Get the auto rotate demo state
     */
    bool IsAutoRotateDemoEnabled() const { return auto_rotate_demo_; }

private:
    SceneManagerBridge* bridge_;
    
    // UI State
    int current_mode_index_ = 0;  // For operation mode combo box
    bool show_statistics_ = true;
    bool auto_rotate_demo_ = false;
    
    // Object manipulation
    ObjectId selected_object_id_ = kInvalidObjectId;
    std::vector<std::string> object_names_;
    int selected_object_index_ = 0;
    
    // Transform controls
    float transform_values_[9] = {0.0f}; // translation, rotation, scale
    bool lock_aspect_ratio_ = true;
    
    // Object creation
    char new_object_name_[64] = "new_object";
    int object_type_index_ = 0; // 0=sphere, 1=cube, 2=point_cloud
    
    // Compound operations
    bool recording_compound_ = false;
    char compound_name_[64] = "Multi Transform";
    
    // Statistics refresh
    float stats_refresh_timer_ = 0.0f;
    const float kStatsRefreshInterval = 1.0f; // 1 second
    
    // Cached statistics values to avoid per-frame string formatting
    std::string cached_stats_commands_;
    std::string cached_stats_undo_;
    std::string cached_stats_redo_;
    std::string cached_stats_undo_depth_;
    std::string cached_stats_redo_depth_;
    std::string cached_stats_memory_;
    std::string cached_stats_compressed_;
    std::string cached_stats_discarded_;
    std::string cached_bridge_memory_;
    bool has_compressed_stats_ = false;
    bool has_discarded_stats_ = false;
    
    // Drawing methods
    void DrawModeControls();
    void DrawObjectList();
    void DrawTransformControls();
    void DrawUndoRedoControls();
    void DrawObjectCreation();
    void DrawCompoundOperations();
    void DrawStatistics();
    void DrawAdvancedControls();
    
    // Helper methods
    void RefreshObjectList();
    void UpdateSelectedObject();
    std::shared_ptr<PointCloud> CreateRandomPointCloud(const std::string& name);
    std::shared_ptr<Sphere> CreateRandomSphere(const std::string& name);
    std::shared_ptr<Mesh> CreateRandomCube(const std::string& name);
    void ApplyTransformToSelected();
    const char* GetModeString(OperationMode mode) const;
    const char* GetObjectTypeString(int type_index) const;
};

} // namespace quickviz

#endif // QUICKVIZ_BRIDGE_CONTROL_PANEL_HPP