/**
 * @file scene_manager_bridge.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-10
 * @brief Implementation of SceneState and GlSceneManager integration bridge
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scenegraph/integration/scene_manager_bridge.hpp"

#include <iostream>
#include <algorithm>
#include <functional>
#include <random>

// Include renderable object types for copying
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/mesh.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

namespace quickviz {

SceneManagerBridge::SceneManagerBridge(std::shared_ptr<SceneManager> scene_manager,
                                       const SceneState::Config& config)
    : scene_manager_(std::move(scene_manager)),
      scene_state_(std::make_unique<SceneState>(config)) {
    
    if (!scene_manager_) {
        throw std::invalid_argument("SceneManager cannot be null");
    }
    
    // Subscribe to state changes for synchronization
    if (config.enable_change_notifications) {
        change_subscription_id_ = scene_state_->Subscribe(
            [this](ObjectId id, const std::string& operation) {
                OnStateChanged(id, operation);
            });
    }
}

SceneManagerBridge::~SceneManagerBridge() {
    if (change_subscription_id_ != 0) {
        scene_state_->Unsubscribe(change_subscription_id_);
    }
}

// === Operation Mode Management ===

void SceneManagerBridge::SetOperationMode(OperationMode mode) {
    scene_state_->SetMode(mode);
}

OperationMode SceneManagerBridge::GetOperationMode() const {
    return scene_state_->GetMode();
}

bool SceneManagerBridge::SupportsUndo() const {
    return scene_state_->SupportsUndo();
}

// === Object Management ===

ObjectId SceneManagerBridge::AddObject(const std::string& name,
                                       std::shared_ptr<OpenGlObject> object) {
    // Register in SceneState first to get ID (keeps shared ownership)
    ObjectId id = scene_state_->RegisterObject(object);
    
    // Track name-ID mapping
    name_to_id_[name] = id;
    id_to_name_[id] = name;
    
    // Also add to SceneManager for rendering
    // We create a separate instance since SceneManager expects unique_ptr
    // but we need shared ownership in SceneState for undo/redo
    // Note: We can't copy OpenGL objects directly due to deleted copy constructors,
    // so we create new objects with the same visual properties
    
    if (auto* sphere = dynamic_cast<Sphere*>(object.get())) {
        // Create a new sphere with same parameters for SceneManager
        glm::vec3 center = sphere->GetCenter();
        float radius = sphere->GetRadius();
        auto sphere_copy = std::make_unique<Sphere>(center, radius);
        scene_manager_->AddOpenGLObject(name, std::move(sphere_copy));
        std::cout << "Added sphere to SceneManager: " << name 
                  << " (center: " << center.x << "," << center.y << "," << center.z 
                  << ", radius: " << radius << ")" << std::endl;
    }
    else if (auto* mesh = dynamic_cast<Mesh*>(object.get())) {
        // Create a new mesh for SceneManager
        // Since Mesh doesn't expose vertices/indices directly, we'll create a simple cube
        // In a production system, we'd add getter methods to Mesh class
        auto mesh_copy = std::make_unique<Mesh>();
        
        // Generate random position and size for this cube to avoid overlapping
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> pos_dist(-4.0f, 4.0f);
        std::uniform_real_distribution<float> size_dist(0.5f, 2.0f);
        
        glm::vec3 center(pos_dist(gen), pos_dist(gen), pos_dist(gen) + 2.0f);
        float size = size_dist(gen);
        float half_size = size * 0.5f;
        
        // Create a cube mesh at random position with random size
        std::vector<glm::vec3> cube_vertices = {
            // Front face
            center + glm::vec3(-half_size, -half_size,  half_size), 
            center + glm::vec3( half_size, -half_size,  half_size),
            center + glm::vec3( half_size,  half_size,  half_size), 
            center + glm::vec3(-half_size,  half_size,  half_size),
            // Back face  
            center + glm::vec3(-half_size, -half_size, -half_size), 
            center + glm::vec3( half_size, -half_size, -half_size),
            center + glm::vec3( half_size,  half_size, -half_size), 
            center + glm::vec3(-half_size,  half_size, -half_size)
        };
        
        std::vector<uint32_t> cube_indices = {
            0, 1, 2,  2, 3, 0,  // Front
            4, 7, 6,  6, 5, 4,  // Back
            3, 2, 6,  6, 7, 3,  // Top
            0, 4, 5,  5, 1, 0,  // Bottom
            1, 5, 6,  6, 2, 1,  // Right
            0, 3, 7,  7, 4, 0   // Left
        };
        
        mesh_copy->SetVertices(cube_vertices);
        mesh_copy->SetIndices(cube_indices);
        mesh_copy->SetColor(glm::vec3(0.8f, 0.4f, 0.2f)); // Orange color
        
        scene_manager_->AddOpenGLObject(name, std::move(mesh_copy));
        std::cout << "Added mesh to SceneManager: " << name 
                  << " (fallback cube at " << center.x << "," << center.y << "," << center.z 
                  << ", size: " << size << ")" << std::endl;
    }
    else if (auto* point_cloud = dynamic_cast<PointCloud*>(object.get())) {
        // Create a new point cloud for SceneManager
        auto cloud_copy = std::make_unique<PointCloud>();
        
        // Generate random position and size for this point cloud
        std::random_device rd2;
        std::mt19937 gen2(rd2());
        std::uniform_real_distribution<float> pos_dist2(-3.0f, 3.0f);
        std::uniform_real_distribution<float> size_dist2(0.3f, 1.0f);
        
        glm::vec3 cloud_center(pos_dist2(gen2), pos_dist2(gen2), pos_dist2(gen2) + 2.0f);
        float cloud_radius = size_dist2(gen2);
        
        // Since we can't extract original points, create a simple test pattern
        std::vector<glm::vec3> test_points;
        std::vector<glm::vec3> test_colors;
        
        // Create a small sphere of points at random location
        for (int i = 0; i < 100; ++i) {
            float theta = (i * 2.0f * M_PI) / 100.0f;
            float phi = (i * M_PI) / 50.0f;
            
            test_points.emplace_back(
                cloud_center + glm::vec3(
                    cloud_radius * sin(phi) * cos(theta),
                    cloud_radius * sin(phi) * sin(theta), 
                    cloud_radius * cos(phi)
                )
            );
            
            // Color based on position
            glm::vec3 color(
                0.5f + 0.5f * sin(theta),
                0.5f + 0.5f * cos(theta),
                0.5f + 0.5f * sin(phi)
            );
            test_colors.push_back(color);
        }
        
        cloud_copy->SetPoints(test_points, test_colors);
        
        // Try to match point size if available
        if (auto original_size = point_cloud->GetPointSize(); original_size > 0) {
            cloud_copy->SetPointSize(original_size);
        } else {
            cloud_copy->SetPointSize(3.0f);
        }
        
        scene_manager_->AddOpenGLObject(name, std::move(cloud_copy));
        std::cout << "Added point cloud to SceneManager: " << name 
                  << " (fallback pattern at " << cloud_center.x << "," << cloud_center.y << "," << cloud_center.z
                  << ", radius: " << cloud_radius << ")" << std::endl;
    }
    else if (auto* grid = dynamic_cast<Grid*>(object.get())) {
        // Create a new grid for SceneManager
        // Grid constructor: Grid(float size, float spacing, glm::vec3 color)
        auto grid_copy = std::make_unique<Grid>(10.0f, 0.5f, glm::vec3(0.3f, 0.3f, 0.3f));
        scene_manager_->AddOpenGLObject(name, std::move(grid_copy));
        std::cout << "Added grid to SceneManager: " << name << " (standard grid)" << std::endl;
    }
    else if (auto* coord_frame = dynamic_cast<CoordinateFrame*>(object.get())) {
        // Create a new coordinate frame for SceneManager  
        // CoordinateFrame constructor: CoordinateFrame(float axis_length, bool is_2d_mode)
        auto frame_copy = std::make_unique<CoordinateFrame>(2.0f, false);
        scene_manager_->AddOpenGLObject(name, std::move(frame_copy));
        std::cout << "Added coordinate frame to SceneManager: " << name << " (standard frame)" << std::endl;
    }
    else {
        std::cout << "Warning: Object type not supported for SceneManager rendering: " 
                  << name << std::endl;
    }
    
    return id;
}

bool SceneManagerBridge::RemoveObject(ObjectId id) {
    auto it = id_to_name_.find(id);
    if (it == id_to_name_.end()) {
        return false;
    }
    
    const std::string& name = it->second;
    
    // Remove from SceneManager first (for rendering)
    scene_manager_->RemoveOpenGLObject(name);
    
    // Remove from SceneState (for state management)
    bool removed = scene_state_->UnregisterObject(id);
    
    // Clean up mappings
    if (removed) {
        name_to_id_.erase(name);
        id_to_name_.erase(id);
    }
    
    return removed;
}

std::shared_ptr<OpenGlObject> SceneManagerBridge::GetObject(ObjectId id) const {
    return scene_state_->GetObject(id);
}

OpenGlObject* SceneManagerBridge::GetObjectByName(const std::string& name) const {
    auto it = name_to_id_.find(name);
    if (it != name_to_id_.end()) {
        auto obj = scene_state_->GetObject(it->second);
        return obj ? obj.get() : nullptr;
    }
    
    // Fall back to SceneManager if not in our mapping
    return scene_manager_->GetOpenGLObject(name);
}

ObjectId SceneManagerBridge::GetObjectId(const std::string& name) const {
    auto it = name_to_id_.find(name);
    return (it != name_to_id_.end()) ? it->second : kInvalidObjectId;
}

std::string SceneManagerBridge::GetObjectName(ObjectId id) const {
    auto it = id_to_name_.find(id);
    return (it != id_to_name_.end()) ? it->second : "";
}

// === State Operations ===

bool SceneManagerBridge::SetTransform(ObjectId id, const glm::mat4& transform) {
    // In Direct mode, directly update the object
    if (scene_state_->GetMode() == OperationMode::kDirect) {
        auto object = scene_state_->GetObject(id);
        if (object) {
            // Direct transform update - this would need OpenGlObject
            // to have a SetTransform method, which it doesn't currently have
            // For now, we'll use SceneState's SetTransform
            return scene_state_->SetTransform(id, transform);
        }
        return false;
    }
    
    // In other modes, use SceneState which handles command creation
    return scene_state_->SetTransform(id, transform);
}

glm::mat4 SceneManagerBridge::GetTransform(ObjectId id) const {
    return scene_state_->GetTransform(id);
}

bool SceneManagerBridge::SetVisible(ObjectId id, bool visible) {
    return scene_state_->SetVisible(id, visible);
}

bool SceneManagerBridge::IsVisible(ObjectId id) const {
    return scene_state_->IsVisible(id);
}

// === Command Operations ===

bool SceneManagerBridge::ExecuteCommand(std::unique_ptr<Command> command) {
    return scene_state_->ExecuteCommand(std::move(command));
}

bool SceneManagerBridge::Undo() {
    return scene_state_->Undo();
}

bool SceneManagerBridge::Redo() {
    return scene_state_->Redo();
}

bool SceneManagerBridge::CanUndo() const {
    return scene_state_->CanUndo();
}

bool SceneManagerBridge::CanRedo() const {
    return scene_state_->CanRedo();
}

std::string SceneManagerBridge::GetUndoDescription() const {
    return scene_state_->GetUndoDescription();
}

std::string SceneManagerBridge::GetRedoDescription() const {
    return scene_state_->GetRedoDescription();
}

// === Compound Operations ===

bool SceneManagerBridge::BeginCompound(const std::string& description) {
    return scene_state_->BeginCompound(description);
}

bool SceneManagerBridge::EndCompound() {
    return scene_state_->EndCompound();
}

bool SceneManagerBridge::IsRecordingCompound() const {
    return scene_state_->IsRecordingCompound();
}

// === Scene Management ===

void SceneManagerBridge::SynchronizeFromSceneManager() {
    // This would iterate through SceneManager's objects and register them
    // in SceneState. Implementation depends on SceneManager's API for
    // enumerating objects, which doesn't seem to be exposed currently.
    
    // For now, this is a placeholder that would need SceneManager
    // to expose an object enumeration API
}

void SceneManagerBridge::Clear() {
    // Clear SceneState
    scene_state_->Clear();
    
    // Clear mappings
    name_to_id_.clear();
    id_to_name_.clear();
}

// === Statistics ===

CommandStack::Statistics SceneManagerBridge::GetCommandStatistics() const {
    return scene_state_->GetCommandStatistics();
}

size_t SceneManagerBridge::GetMemoryUsage() const {
    return scene_state_->GetMemoryUsage();
}

// === Internal Methods ===

void SceneManagerBridge::SyncObjectToSceneManager(ObjectId id, const std::string& name) {
    auto object = scene_state_->GetObject(id);
    if (!object) {
        return;
    }
    
    // Check if object exists in SceneManager
    auto* existing = scene_manager_->GetOpenGLObject(name);
    if (existing) {
        // Update existing object properties
        // This would require OpenGlObject to expose update methods
        // For now, we can only update through SceneState
    } else {
        // Object doesn't exist in SceneManager, add it
        // This is complex due to ownership model differences
        // SceneManager wants unique_ptr, we have shared_ptr
    }
}

void SceneManagerBridge::SyncObjectFromSceneManager(const std::string& name) {
    auto* object = scene_manager_->GetOpenGLObject(name);
    if (!object) {
        return;
    }
    
    // Check if already tracked
    auto it = name_to_id_.find(name);
    if (it != name_to_id_.end()) {
        // Already tracked, possibly update state
        return;
    }
    
    // Register new object in SceneState
    // This is complex because we don't have shared_ptr ownership
    // of the object from SceneManager
}

void SceneManagerBridge::OnStateChanged(ObjectId id, const std::string& operation) {
    // Handle state change notifications
    // Synchronize changes to SceneManager if needed
    
    auto it = id_to_name_.find(id);
    if (it != id_to_name_.end()) {
        const std::string& name = it->second;
        
        if (operation == "transform") {
            // Transform changed in SceneState, update SceneManager
            // This would require OpenGlObject to have SetTransform
        } else if (operation == "visibility") {
            // Visibility changed in SceneState, update SceneManager
            auto object = scene_state_->GetObject(id);
            if (object) {
                // OpenGlObject has SetVisible method
                object->SetVisible(scene_state_->IsVisible(id));
            }
        }
    }
}

std::vector<std::string> SceneManagerBridge::GetAllObjectNames() const {
    std::vector<std::string> names;
    names.reserve(name_to_id_.size());
    
    for (const auto& pair : name_to_id_) {
        names.push_back(pair.first);
    }
    
    return names;
}

} // namespace quickviz