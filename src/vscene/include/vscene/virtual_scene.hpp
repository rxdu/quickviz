/*
 * @file virtual_scene.hpp
 * @date August 27, 2025
 * @brief Virtual scene manager for high-level 3D scene operations
 * 
 * VirtualScene provides the main interface for managing virtual objects,
 * handling interaction events, and coordinating with the render backend.
 * It serves as the central hub for all virtual scene operations.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VIRTUAL_SCENE_HPP
#define QUICKVIZ_VIRTUAL_SCENE_HPP

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "vscene/virtual_object.hpp"
#include "vscene/render_interface.hpp"
#include "vscene/event_system.hpp"

namespace quickviz {

/**
 * @brief Virtual scene manager
 * 
 * VirtualScene manages a collection of virtual objects and coordinates their
 * rendering through the backend interface. It handles object lifecycle,
 * selection management, interaction processing, and event dispatching.
 * 
 * Key responsibilities:
 * - Virtual object lifecycle management
 * - Selection and interaction state
 * - Event processing and dispatching
 * - Backend coordination
 * - Scene graph operations
 */
class VirtualScene {
public:
    VirtualScene();
    ~VirtualScene();

    // Backend management
    void SetRenderBackend(std::unique_ptr<RenderInterface> backend);
    RenderInterface* GetRenderBackend() const { return backend_.get(); }

    // Object management
    void AddObject(const std::string& id, std::unique_ptr<VirtualObject> object);
    void RemoveObject(const std::string& id);
    VirtualObject* GetObject(const std::string& id) const;
    std::vector<std::string> GetObjectIds() const;
    void ClearObjects();

    // Selection management
    void SetSelected(const std::string& id, bool selected);
    void ClearSelection();
    void SelectAll();
    void SelectNone();
    std::vector<std::string> GetSelectedIds() const;
    size_t GetSelectedCount() const;
    bool IsSelected(const std::string& id) const;

    // Multi-selection operations
    void AddToSelection(const std::string& id);
    void RemoveFromSelection(const std::string& id);
    void ToggleSelection(const std::string& id);

    // Interaction
    VirtualObject* Pick(float screen_x, float screen_y);
    std::vector<VirtualObject*> PickMultiple(float screen_x, float screen_y, float radius = 5.0f);
    VirtualObject* GetHoveredObject() const { return hovered_object_; }

    // Scene operations
    void Update(float delta_time);
    void Render();

    // Event system access
    EventDispatcher* GetEventDispatcher() { return &event_dispatcher_; }
    const EventDispatcher* GetEventDispatcher() const { return &event_dispatcher_; }

    // Utility operations
    BoundingBox GetSceneBounds() const;
    BoundingBox GetSelectionBounds() const;
    glm::vec3 GetSelectionCentroid() const;
    
    // Transform operations on selection
    void TranslateSelection(const glm::vec3& delta);
    void RotateSelection(const glm::quat& rotation, const glm::vec3& center);
    void ScaleSelection(const glm::vec3& scale, const glm::vec3& center);

public:
    // Configuration options
    struct Config {
        bool auto_update_backend = true;    // Automatically sync changes to backend
        bool enable_hover_tracking = true; // Track mouse hover events
        float hover_distance = 2.0f;       // Distance threshold for hover detection
        bool multi_selection_enabled = true; // Allow multiple objects to be selected
    };
    
    void SetConfig(const Config& config) { config_ = config; }
    const Config& GetConfig() const { return config_; }

private:
    // Internal state
    std::unordered_map<std::string, std::unique_ptr<VirtualObject>> objects_;
    std::unordered_set<std::string> selected_objects_;
    VirtualObject* hovered_object_ = nullptr;
    
    // Systems
    std::unique_ptr<RenderInterface> backend_;
    EventDispatcher event_dispatcher_;
    Config config_;

    // Internal methods
    void UpdateBackendForObject(VirtualObject* object);
    void UpdateHoverState(float screen_x, float screen_y);
    void DispatchEvent(const VirtualEvent& event);
    
    // Interaction processing
    void ProcessClick(float screen_x, float screen_y, int button);
    void ProcessDrag(float screen_x, float screen_y, const glm::vec2& delta);
    void ProcessHover(float screen_x, float screen_y);

    // Validation helpers
    bool IsValidObject(const std::string& id) const;
    void ValidateSelection();
    
    friend class VirtualScenePanel; // Allow VirtualScenePanel to call interaction methods
};

/**
 * @brief Statistics and debugging information for virtual scene
 */
struct VirtualSceneStats {
    size_t total_objects = 0;
    size_t selected_objects = 0;
    size_t visible_objects = 0;
    size_t render_calls_last_frame = 0;
    float last_frame_time_ms = 0.0f;
    
    // Memory usage estimates
    size_t estimated_gpu_memory_bytes = 0;
    size_t estimated_cpu_memory_bytes = 0;
};

} // namespace quickviz

#endif // QUICKVIZ_VIRTUAL_SCENE_HPP