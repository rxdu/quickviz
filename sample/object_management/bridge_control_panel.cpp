/*
 * @file bridge_control_panel.cpp
 * @date Sep 10, 2025
 * @brief Implementation of Bridge Control Panel
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "bridge_control_panel.hpp"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <random>
#include <sstream>
#include <chrono>

namespace quickviz {

BridgeControlPanel::BridgeControlPanel(const std::string& name, SceneManagerBridge* bridge)
    : Panel(name), bridge_(bridge) {
    
    // Initialize with current mode
    current_mode_index_ = static_cast<int>(bridge_->GetOperationMode());
    
    // Refresh object list
    RefreshObjectList();
}

void BridgeControlPanel::Draw() {
    if (!bridge_) return;
    
    stats_refresh_timer_ += ImGui::GetIO().DeltaTime;
    
    if (ImGui::Begin(GetName().c_str())) {
        // Main sections
        DrawModeControls();
        ImGui::Separator();
        
        DrawObjectList();
        ImGui::Separator();
        
        DrawTransformControls();
        ImGui::Separator();
        
        DrawUndoRedoControls();
        ImGui::Separator();
        
        DrawObjectCreation();
        ImGui::Separator();
        
        DrawCompoundOperations();
        ImGui::Separator();
        
        DrawStatistics();
        ImGui::Separator();
        
        DrawAdvancedControls();
    }
    ImGui::End();
}

void BridgeControlPanel::DrawModeControls() {
    ImGui::Text("Operation Mode");
    
    const char* modes[] = { "Direct (Real-time)", "Immediate (Tracked)", "Recorded (Undo/Redo)" };
    
    if (ImGui::Combo("Mode", &current_mode_index_, modes, IM_ARRAYSIZE(modes))) {
        OperationMode new_mode = static_cast<OperationMode>(current_mode_index_);
        bridge_->SetOperationMode(new_mode);
        std::cout << "Switched to " << GetModeString(new_mode) << " mode" << std::endl;
    }
    
    // Show current mode info
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(%s)", 
                      bridge_->SupportsUndo() ? "Undo enabled" : "No undo");
    
    // Mode description
    const char* description = "";
    switch (bridge_->GetOperationMode()) {
        case OperationMode::kDirect:
            description = "Zero overhead mode for real-time visualization";
            break;
        case OperationMode::kImmediate:
            description = "State tracked but no history kept";
            break;
        case OperationMode::kRecorded:
            description = "Full command history with undo/redo support";
            break;
    }
    ImGui::TextWrapped("%s", description);
}

void BridgeControlPanel::DrawObjectList() {
    ImGui::Text("Scene Objects");
    
    if (ImGui::Button("Refresh List")) {
        RefreshObjectList();
    }
    ImGui::SameLine();
    ImGui::Text("(%zu objects)", object_names_.size());
    
    if (!object_names_.empty()) {
        // Object selection combo
        std::vector<const char*> names_cstr;
        for (const auto& name : object_names_) {
            names_cstr.push_back(name.c_str());
        }
        
        if (ImGui::Combo("Select Object", &selected_object_index_, names_cstr.data(), names_cstr.size())) {
            UpdateSelectedObject();
        }
        
        // Show selected object info
        if (selected_object_id_ != kInvalidObjectId) {
            ImGui::Text("Selected: %s (ID: %u)", 
                       object_names_[selected_object_index_].c_str(), 
                       selected_object_id_);
            
            bool visible = bridge_->IsVisible(selected_object_id_);
            if (ImGui::Checkbox("Visible", &visible)) {
                bridge_->SetVisible(selected_object_id_, visible);
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Remove Object")) {
                if (bridge_->RemoveObject(selected_object_id_)) {
                    std::cout << "Removed object: " << object_names_[selected_object_index_] << std::endl;
                    RefreshObjectList();
                    selected_object_id_ = kInvalidObjectId;
                }
            }
        }
    } else {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No objects in scene");
    }
}

void BridgeControlPanel::DrawTransformControls() {
    ImGui::Text("Transform Controls");
    
    if (selected_object_id_ == kInvalidObjectId) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No object selected");
        return;
    }
    
    // Translation controls
    ImGui::Text("Translation:");
    ImGui::SliderFloat3("Position", &transform_values_[0], -10.0f, 10.0f);
    
    // Rotation controls (in degrees)
    ImGui::Text("Rotation (degrees):");
    ImGui::SliderFloat3("Rotation", &transform_values_[3], -180.0f, 180.0f);
    
    // Scale controls
    ImGui::Text("Scale:");
    if (lock_aspect_ratio_) {
        if (ImGui::SliderFloat("Uniform Scale", &transform_values_[6], 0.1f, 3.0f)) {
            transform_values_[7] = transform_values_[8] = transform_values_[6];
        }
    } else {
        ImGui::SliderFloat3("Scale XYZ", &transform_values_[6], 0.1f, 3.0f);
    }
    ImGui::Checkbox("Lock Aspect Ratio", &lock_aspect_ratio_);
    
    // Apply buttons
    if (ImGui::Button("Apply Transform")) {
        ApplyTransformToSelected();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Transform")) {
        memset(transform_values_, 0, sizeof(transform_values_));
        transform_values_[6] = transform_values_[7] = transform_values_[8] = 1.0f; // Default scale
        ApplyTransformToSelected();
    }
    
    // Quick transform buttons
    ImGui::Text("Quick Actions:");
    if (ImGui::Button("Move Up")) {
        transform_values_[2] += 0.5f;
        ApplyTransformToSelected();
    }
    ImGui::SameLine();
    if (ImGui::Button("Move Down")) {
        transform_values_[2] -= 0.5f;
        ApplyTransformToSelected();
    }
    ImGui::SameLine();
    if (ImGui::Button("Rotate 45°")) {
        transform_values_[5] += 45.0f; // Z rotation
        ApplyTransformToSelected();
    }
}

void BridgeControlPanel::DrawUndoRedoControls() {
    ImGui::Text("Undo/Redo Controls");
    
    if (!bridge_->SupportsUndo()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), 
                          "Not available in current mode");
        return;
    }
    
    // Undo button
    bool can_undo = bridge_->CanUndo();
    if (!can_undo) ImGui::BeginDisabled();
    if (ImGui::Button("Undo")) {
        if (bridge_->Undo()) {
            std::cout << "Undo successful" << std::endl;
        }
    }
    if (!can_undo) ImGui::EndDisabled();
    
    ImGui::SameLine();
    
    // Redo button
    bool can_redo = bridge_->CanRedo();
    if (!can_redo) ImGui::BeginDisabled();
    if (ImGui::Button("Redo")) {
        if (bridge_->Redo()) {
            std::cout << "Redo successful" << std::endl;
        }
    }
    if (!can_redo) ImGui::EndDisabled();
    
    // Show descriptions
    if (can_undo) {
        ImGui::Text("Next undo: %s", bridge_->GetUndoDescription().c_str());
    }
    if (can_redo) {
        ImGui::Text("Next redo: %s", bridge_->GetRedoDescription().c_str());
    }
}

void BridgeControlPanel::DrawObjectCreation() {
    ImGui::Text("Create New Objects");
    
    ImGui::InputText("Object Name", new_object_name_, sizeof(new_object_name_));
    
    const char* types[] = { "Sphere", "Cube", "Point Cloud" };
    ImGui::Combo("Object Type", &object_type_index_, types, IM_ARRAYSIZE(types));
    
    if (ImGui::Button("Create Object")) {
        std::string name = std::string(new_object_name_);
        if (name.empty() || name == "new_object") {
            // Generate unique name with timestamp to avoid conflicts
            auto now = std::chrono::steady_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            name = std::string(GetObjectTypeString(object_type_index_)) + "_" + std::to_string(ms % 10000);
        }
        
        ObjectId new_id = kInvalidObjectId;
        
        switch (object_type_index_) {
            case 0: { // Sphere
                auto sphere = CreateRandomSphere(name);
                new_id = bridge_->AddObject(name, sphere);
                break;
            }
            case 1: { // Cube
                auto cube = CreateRandomCube(name);
                new_id = bridge_->AddObject(name, cube);
                break;
            }
            case 2: { // Point Cloud
                auto cloud = CreateRandomPointCloud(name);
                new_id = bridge_->AddObject(name, cloud);
                break;
            }
        }
        
        if (new_id != kInvalidObjectId) {
            std::cout << "Created " << GetObjectTypeString(object_type_index_) 
                      << ": " << name << " (ID: " << new_id << ")" << std::endl;
            RefreshObjectList();
            
            // Select the new object
            for (size_t i = 0; i < object_names_.size(); ++i) {
                if (object_names_[i] == name) {
                    selected_object_index_ = static_cast<int>(i);
                    UpdateSelectedObject();
                    break;
                }
            }
        }
    }
}

void BridgeControlPanel::DrawCompoundOperations() {
    ImGui::Text("Compound Operations");
    
    if (!bridge_->SupportsUndo()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), 
                          "Not available in current mode");
        return;
    }
    
    ImGui::InputText("Compound Name", compound_name_, sizeof(compound_name_));
    
    bool is_recording = bridge_->IsRecordingCompound();
    
    if (!is_recording) {
        if (ImGui::Button("Begin Compound")) {
            if (bridge_->BeginCompound(compound_name_)) {
                recording_compound_ = true;
                std::cout << "Started compound operation: " << compound_name_ << std::endl;
            }
        }
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.6f, 1.0f), "Recording compound operation...");
        
        if (ImGui::Button("End Compound")) {
            if (bridge_->EndCompound()) {
                recording_compound_ = false;
                std::cout << "Ended compound operation: " << compound_name_ << std::endl;
            }
        }
    }
    
    // Demo compound operation
    if (!is_recording && ImGui::Button("Demo: Transform All Spheres")) {
        bridge_->BeginCompound("Transform All Spheres");
        
        // Find all spheres and transform them
        for (const auto& name : object_names_) {
            ObjectId id = bridge_->GetObjectId(name);
            if (id != kInvalidObjectId && name.find("sphere") != std::string::npos) {
                glm::mat4 transform = glm::translate(glm::mat4(1.0f), 
                                                   glm::vec3((rand() % 200 - 100) / 50.0f,
                                                           (rand() % 200 - 100) / 50.0f,
                                                           (rand() % 100) / 50.0f + 1.0f));
                bridge_->SetTransform(id, transform);
            }
        }
        
        bridge_->EndCompound();
        std::cout << "Executed demo compound operation" << std::endl;
    }
}

void BridgeControlPanel::DrawStatistics() {
    ImGui::Text("Bridge Statistics");
    
    ImGui::Checkbox("Show Statistics", &show_statistics_);
    
    // Only show detailed statistics if checkbox is enabled
    if (!show_statistics_) {
        return;  // Exit early, but checkbox remains visible
    }
    
    // Update cached values periodically
    if (stats_refresh_timer_ >= kStatsRefreshInterval) {
        stats_refresh_timer_ = 0.0f;
        
        if (bridge_->SupportsUndo()) {
            auto stats = bridge_->GetCommandStatistics();
            
            // Format and cache all statistics strings
            char buffer[128];
            
            snprintf(buffer, sizeof(buffer), "Commands executed: %zu", stats.total_commands_executed);
            cached_stats_commands_ = buffer;
            
            snprintf(buffer, sizeof(buffer), "Undo operations: %zu", stats.undo_count);
            cached_stats_undo_ = buffer;
            
            snprintf(buffer, sizeof(buffer), "Redo operations: %zu", stats.redo_count);
            cached_stats_redo_ = buffer;
            
            snprintf(buffer, sizeof(buffer), "Available undos: %zu", stats.current_undo_depth);
            cached_stats_undo_depth_ = buffer;
            
            snprintf(buffer, sizeof(buffer), "Available redos: %zu", stats.current_redo_depth);
            cached_stats_redo_depth_ = buffer;
            
            snprintf(buffer, sizeof(buffer), "Memory usage: %.2f KB", stats.memory_usage_bytes / 1024.0f);
            cached_stats_memory_ = buffer;
            
            // Handle optional statistics
            has_compressed_stats_ = stats.commands_compressed > 0;
            if (has_compressed_stats_) {
                snprintf(buffer, sizeof(buffer), "Commands compressed: %zu", stats.commands_compressed);
                cached_stats_compressed_ = buffer;
            }
            
            has_discarded_stats_ = stats.commands_discarded > 0;
            if (has_discarded_stats_) {
                snprintf(buffer, sizeof(buffer), "Commands discarded: %zu", stats.commands_discarded);
                cached_stats_discarded_ = buffer;
            }
        } else {
            // Cache the "no statistics" message
            cached_stats_commands_ = "No statistics in current mode";
            cached_stats_undo_.clear();
            cached_stats_redo_.clear();
            cached_stats_undo_depth_.clear();
            cached_stats_redo_depth_.clear();
            cached_stats_memory_.clear();
            has_compressed_stats_ = false;
            has_discarded_stats_ = false;
        }
        
        // Cache bridge memory usage
        size_t memory_usage = bridge_->GetMemoryUsage();
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Bridge memory: %.2f KB", memory_usage / 1024.0f);
        cached_bridge_memory_ = buffer;
    }
    
    // Always display cached statistics (no flickering)
    if (bridge_->SupportsUndo()) {
        if (!cached_stats_commands_.empty()) {
            ImGui::Text("%s", cached_stats_commands_.c_str());
            ImGui::Text("%s", cached_stats_undo_.c_str());
            ImGui::Text("%s", cached_stats_redo_.c_str());
            ImGui::Text("%s", cached_stats_undo_depth_.c_str());
            ImGui::Text("%s", cached_stats_redo_depth_.c_str());
            ImGui::Text("%s", cached_stats_memory_.c_str());
            
            if (has_compressed_stats_) {
                ImGui::Text("%s", cached_stats_compressed_.c_str());
            }
            if (has_discarded_stats_) {
                ImGui::Text("%s", cached_stats_discarded_.c_str());
            }
        }
    } else {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), 
                          "%s", cached_stats_commands_.c_str());
    }
    
    if (!cached_bridge_memory_.empty()) {
        ImGui::Text("%s", cached_bridge_memory_.c_str());
    }
}

void BridgeControlPanel::DrawAdvancedControls() {
    ImGui::Text("Advanced Controls");
    
    if (ImGui::Button("Clear All Objects")) {
        bridge_->Clear();
        RefreshObjectList();
        selected_object_id_ = kInvalidObjectId;
        std::cout << "Cleared all objects" << std::endl;
    }
    
    ImGui::SameLine();
    if (ImGui::Button("Reset Scene")) {
        // This would reset the scene to initial state
        bridge_->Clear();
        // Re-add default objects would go here
        RefreshObjectList();
        std::cout << "Reset scene" << std::endl;
    }
    
    ImGui::Checkbox("Auto Rotate Demo", &auto_rotate_demo_);
    if (auto_rotate_demo_) {
        ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f), 
                          "Demo animation active (Direct mode only)");
    }
}

// Helper method implementations

void BridgeControlPanel::RefreshObjectList() {
    object_names_.clear();
    
    // Get all object names from the bridge
    object_names_ = bridge_->GetAllObjectNames();
    
    // Ensure selected index is valid
    if (selected_object_index_ >= static_cast<int>(object_names_.size())) {
        selected_object_index_ = 0;
        selected_object_id_ = kInvalidObjectId;
    }
    
    if (!object_names_.empty() && selected_object_index_ >= 0) {
        UpdateSelectedObject();
    }
}

void BridgeControlPanel::UpdateSelectedObject() {
    if (selected_object_index_ >= 0 && 
        selected_object_index_ < static_cast<int>(object_names_.size())) {
        
        const std::string& name = object_names_[selected_object_index_];
        selected_object_id_ = bridge_->GetObjectId(name);
        
        // Reset transform values to current object transform
        if (selected_object_id_ != kInvalidObjectId) {
            // We can't easily extract transform components from glm::mat4
            // Reset to identity for now
            memset(transform_values_, 0, sizeof(transform_values_));
            transform_values_[6] = transform_values_[7] = transform_values_[8] = 1.0f;
        }
    } else {
        selected_object_id_ = kInvalidObjectId;
    }
}

void BridgeControlPanel::ApplyTransformToSelected() {
    if (selected_object_id_ == kInvalidObjectId) return;
    
    // Build transformation matrix
    glm::mat4 transform = glm::mat4(1.0f);
    
    // Apply translation
    transform = glm::translate(transform, glm::vec3(transform_values_[0], 
                                                  transform_values_[1], 
                                                  transform_values_[2]));
    
    // Apply rotation (convert degrees to radians)
    transform = glm::rotate(transform, glm::radians(transform_values_[3]), glm::vec3(1, 0, 0));
    transform = glm::rotate(transform, glm::radians(transform_values_[4]), glm::vec3(0, 1, 0));
    transform = glm::rotate(transform, glm::radians(transform_values_[5]), glm::vec3(0, 0, 1));
    
    // Apply scale
    transform = glm::scale(transform, glm::vec3(transform_values_[6], 
                                              transform_values_[7], 
                                              transform_values_[8]));
    
    bridge_->SetTransform(selected_object_id_, transform);
}

std::shared_ptr<PointCloud> BridgeControlPanel::CreateRandomPointCloud(const std::string& name) {
    auto cloud = std::make_shared<PointCloud>();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-3.0f, 3.0f);
    std::uniform_real_distribution<float> color_dist(0.0f, 1.0f);
    
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    size_t num_points = 500 + (gen() % 1000);
    for (size_t i = 0; i < num_points; ++i) {
        points.emplace_back(pos_dist(gen), pos_dist(gen), pos_dist(gen) + 2.0f);
        colors.emplace_back(color_dist(gen), color_dist(gen), color_dist(gen));
    }
    
    cloud->SetPoints(points, colors);
    cloud->SetPointSize(2.0f);
    
    return cloud;
}

std::shared_ptr<Sphere> BridgeControlPanel::CreateRandomSphere(const std::string& name) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> size_dist(0.3f, 1.5f);
    
    glm::vec3 center(pos_dist(gen), pos_dist(gen), pos_dist(gen) + 2.0f);
    float radius = size_dist(gen);
    
    return std::make_shared<Sphere>(center, radius);
}

std::shared_ptr<Mesh> BridgeControlPanel::CreateRandomCube(const std::string& name) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> size_dist(0.5f, 2.0f);
    
    auto mesh = std::make_shared<Mesh>();
    
    glm::vec3 center(pos_dist(gen), pos_dist(gen), pos_dist(gen) + 2.0f);
    float size = size_dist(gen);
    float half_size = size * 0.5f;
    
    std::vector<glm::vec3> vertices = {
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
    
    std::vector<uint32_t> indices = {
        0, 1, 2,  2, 3, 0,  // Front
        4, 7, 6,  6, 5, 4,  // Back
        3, 2, 6,  6, 7, 3,  // Top
        0, 4, 5,  5, 1, 0,  // Bottom
        1, 5, 6,  6, 2, 1,  // Right
        0, 3, 7,  7, 4, 0   // Left
    };
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

const char* BridgeControlPanel::GetModeString(OperationMode mode) const {
    switch (mode) {
        case OperationMode::kDirect: return "Direct";
        case OperationMode::kImmediate: return "Immediate";
        case OperationMode::kRecorded: return "Recorded";
        default: return "Unknown";
    }
}

const char* BridgeControlPanel::GetObjectTypeString(int type_index) const {
    switch (type_index) {
        case 0: return "Sphere";
        case 1: return "Cube";
        case 2: return "Point Cloud";
        default: return "Unknown";
    }
}

} // namespace quickviz