/*
 * @file mock_render_backend.hpp
 * @date August 27, 2025
 * @brief Mock render backend for testing virtual scene functionality
 *
 * Provides a simple mock implementation of RenderInterface for testing
 * virtual scene operations without requiring OpenGL setup.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_MOCK_RENDER_BACKEND_HPP
#define QUICKVIZ_MOCK_RENDER_BACKEND_HPP

#include <unordered_map>
#include <vector>
#include <string>

#include "vscene/render_interface.hpp"
#include "vscene/virtual_object_types.hpp"

namespace quickviz {

/**
 * @brief Mock render backend for testing
 * 
 * Records all operations for verification in tests without requiring
 * actual rendering infrastructure. Useful for testing VirtualScene
 * logic and object management.
 */
class MockRenderBackend : public RenderInterface {
public:
    MockRenderBackend() = default;
    ~MockRenderBackend() override = default;

    // RenderInterface interface
    void CreateObject(const std::string& id, VirtualObjectType type, 
                      const VirtualObjectData& initial_data) override {
        created_objects_[id] = {type, initial_data};
        operation_log_.push_back("CREATE:" + id + ":" + ToString(type));
    }
    
    void UpdateObject(const std::string& id, const VirtualObjectData& data) override {
        if (created_objects_.find(id) != created_objects_.end()) {
            created_objects_[id].data = data;
            operation_log_.push_back("UPDATE:" + id);
        }
    }
    
    void RemoveObject(const std::string& id) override {
        created_objects_.erase(id);
        operation_log_.push_back("REMOVE:" + id);
    }
    
    void ClearAllObjects() override {
        created_objects_.clear();
        operation_log_.push_back("CLEAR_ALL");
    }
    
    void RenderToFramebuffer(float width, float height) override {
        last_render_size_ = glm::vec2(width, height);
        render_call_count_++;
        operation_log_.push_back("RENDER:" + std::to_string(width) + "x" + std::to_string(height));
    }
    
    std::string PickObjectAt(float screen_x, float screen_y) override {
        last_pick_position_ = glm::vec2(screen_x, screen_y);
        operation_log_.push_back("PICK:" + std::to_string(screen_x) + "," + std::to_string(screen_y));
        return mock_picked_object_;
    }
    
    uint32_t GetFramebufferTexture() const override {
        return 0; // Mock texture ID
    }
    
    Ray GetMouseRay(float screen_x, float screen_y, float width, float height) const override {
        // Simple mock ray - in real implementation this would use camera projection
        return Ray{glm::vec3(screen_x, screen_y, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    }
    
    void SetBackgroundColor(float r, float g, float b, float a) override {
        mock_background_color_ = glm::vec4(r, g, b, a);
        operation_log_.push_back("SET_BG_COLOR:" + std::to_string(r) + "," + std::to_string(g) + 
                                "," + std::to_string(b) + "," + std::to_string(a));
    }

    // Testing utilities
    void SetMockPickedObject(const std::string& id) { 
        mock_picked_object_ = id; 
    }
    
    bool HasObject(const std::string& id) const {
        return created_objects_.find(id) != created_objects_.end();
    }
    
    VirtualObjectData GetObjectData(const std::string& id) const {
        auto it = created_objects_.find(id);
        return (it != created_objects_.end()) ? it->second.data : VirtualObjectData{};
    }
    
    VirtualObjectType GetObjectType(const std::string& id) const {
        auto it = created_objects_.find(id);
        return (it != created_objects_.end()) ? it->second.type : VirtualObjectType::Custom;
    }
    
    const char* GetObjectTypeString(const std::string& id) const {
        auto it = created_objects_.find(id);
        return (it != created_objects_.end()) ? ToString(it->second.type) : "unknown";
    }
    
    size_t GetObjectCount() const { 
        return created_objects_.size(); 
    }
    
    int GetRenderCallCount() const { 
        return render_call_count_; 
    }
    
    glm::vec2 GetLastRenderSize() const { 
        return last_render_size_; 
    }
    
    glm::vec2 GetLastPickPosition() const { 
        return last_pick_position_; 
    }
    
    const std::vector<std::string>& GetOperationLog() const { 
        return operation_log_; 
    }
    
    void ClearLog() { 
        operation_log_.clear(); 
    }
    
    void Reset() {
        created_objects_.clear();
        operation_log_.clear();
        render_call_count_ = 0;
        last_render_size_ = glm::vec2(0.0f);
        last_pick_position_ = glm::vec2(0.0f);
        mock_picked_object_.clear();
    }

private:
    struct ObjectRecord {
        VirtualObjectType type;
        VirtualObjectData data;
    };
    
    std::unordered_map<std::string, ObjectRecord> created_objects_;
    std::vector<std::string> operation_log_;
    std::string mock_picked_object_;
    int render_call_count_ = 0;
    glm::vec2 last_render_size_{0.0f};
    glm::vec2 last_pick_position_{0.0f};
    glm::vec4 mock_background_color_{0.0f, 0.0f, 0.0f, 1.0f};
};

} // namespace quickviz

#endif // QUICKVIZ_MOCK_RENDER_BACKEND_HPP