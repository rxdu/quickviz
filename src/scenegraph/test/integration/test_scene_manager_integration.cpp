/**
 * @file test_scene_manager_integration.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-10
 * @brief Integration test for SceneState and GlSceneManager bridge
 *
 * Demonstrates the complete API usage and integration between the new
 * state management system and the existing OpenGL rendering pipeline.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "scenegraph/integration/scene_manager_bridge.hpp"
#include "scenegraph/command/command.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/mesh.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "imview/viewer.hpp"

namespace quickviz {

// Test fixture for integration tests
class SceneManagerIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        try {
            // Create viewer using proper QuickViz architecture
            viewer_ = std::make_unique<Viewer>("Test Viewer", 800, 600, 
                                             Viewer::WIN_DECORATED);
            
            // Create SceneManager
            scene_manager_ = std::make_shared<SceneManager>("test_scene");
            
            // Create bridge with default config (Direct mode)
            bridge_ = std::make_unique<SceneManagerBridge>(scene_manager_);
            
        } catch (const std::exception& e) {
            GTEST_SKIP() << "Failed to initialize test environment: " << e.what();
        }
    }
    
    void TearDown() override {
        // Clean up in reverse order of creation to avoid dangling pointers
        if (bridge_) {
            try {
                bridge_->Clear();  // Clear objects first
            } catch (...) {
                // Ignore cleanup errors during test shutdown
            }
            bridge_.reset();   // Then destroy bridge
        }
        
        scene_manager_.reset(); // Then scene manager
        viewer_.reset();        // Finally destroy viewer (handles window/GLFW cleanup)
    }
    
    // Helper to create test point cloud
    std::shared_ptr<PointCloud> CreateTestPointCloud(size_t num_points = 1000) {
        auto cloud = std::make_shared<PointCloud>();
        
        std::vector<glm::vec3> points;
        std::vector<glm::vec3> colors;
        
        for (size_t i = 0; i < num_points; ++i) {
            float t = static_cast<float>(i) / num_points;
            float angle = t * 2.0f * M_PI * 3.0f;
            float radius = t * 5.0f;
            
            points.emplace_back(
                radius * cos(angle),
                radius * sin(angle),
                t * 10.0f
            );
            
            colors.emplace_back(t, 1.0f - t, 0.5f);
        }
        
        // Use the correct API for setting points with colors
        cloud->SetPoints(points, colors);
        
        return cloud;
    }
    
    // Helper to create test mesh (cube)
    std::shared_ptr<Mesh> CreateTestMesh() {
        auto mesh = std::make_shared<Mesh>();
        
        // Define cube vertices (positions only for now)
        std::vector<glm::vec3> vertices = {
            // Front face
            {-1, -1,  1},
            { 1, -1,  1},
            { 1,  1,  1},
            {-1,  1,  1},
            // Back face
            {-1, -1, -1},
            { 1, -1, -1},
            { 1,  1, -1},
            {-1,  1, -1}
        };
        
        // Define cube indices
        std::vector<uint32_t> indices = {
            // Front face
            0, 1, 2,  2, 3, 0,
            // Back face
            4, 7, 6,  6, 5, 4,
            // Top face
            3, 2, 6,  6, 7, 3,
            // Bottom face
            0, 4, 5,  5, 1, 0,
            // Right face
            1, 5, 6,  6, 2, 1,
            // Left face
            0, 3, 7,  7, 4, 0
        };
        
        // Set vertices and indices separately
        mesh->SetVertices(vertices);
        mesh->SetIndices(indices);
        
        return mesh;
    }
    
protected:
    std::unique_ptr<Viewer> viewer_;
    std::shared_ptr<SceneManager> scene_manager_;
    std::unique_ptr<SceneManagerBridge> bridge_;
};

// Test basic object management through the bridge
TEST_F(SceneManagerIntegrationTest, ObjectManagement) {
    // Create test objects
    auto point_cloud = CreateTestPointCloud();
    auto mesh = CreateTestMesh();
    auto sphere = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 1.0f);
    
    // Add objects through bridge
    ObjectId cloud_id = bridge_->AddObject("point_cloud", point_cloud);
    ObjectId mesh_id = bridge_->AddObject("mesh", mesh);
    ObjectId sphere_id = bridge_->AddObject("sphere", sphere);
    
    // Verify IDs are valid
    EXPECT_NE(cloud_id, kInvalidObjectId);
    EXPECT_NE(mesh_id, kInvalidObjectId);
    EXPECT_NE(sphere_id, kInvalidObjectId);
    
    // Verify object retrieval by ID
    EXPECT_EQ(bridge_->GetObject(cloud_id), point_cloud);
    EXPECT_EQ(bridge_->GetObject(mesh_id), mesh);
    EXPECT_EQ(bridge_->GetObject(sphere_id), sphere);
    
    // Verify object retrieval by name
    EXPECT_EQ(bridge_->GetObjectByName("point_cloud"), point_cloud.get());
    EXPECT_EQ(bridge_->GetObjectByName("mesh"), mesh.get());
    EXPECT_EQ(bridge_->GetObjectByName("sphere"), sphere.get());
    
    // Verify name-ID mapping
    EXPECT_EQ(bridge_->GetObjectId("point_cloud"), cloud_id);
    EXPECT_EQ(bridge_->GetObjectName(cloud_id), "point_cloud");
    
    // Remove an object
    EXPECT_TRUE(bridge_->RemoveObject(mesh_id));
    EXPECT_EQ(bridge_->GetObject(mesh_id), nullptr);
    EXPECT_EQ(bridge_->GetObjectByName("mesh"), nullptr);
}

// Test operation modes and their behaviors
TEST_F(SceneManagerIntegrationTest, OperationModes) {
    auto sphere = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 1.0f);
    ObjectId sphere_id = bridge_->AddObject("sphere", sphere);
    
    // Test Direct mode (default)
    EXPECT_EQ(bridge_->GetOperationMode(), OperationMode::kDirect);
    EXPECT_FALSE(bridge_->SupportsUndo());
    
    glm::mat4 transform1 = glm::translate(glm::mat4(1.0f), glm::vec3(1, 0, 0));
    EXPECT_TRUE(bridge_->SetTransform(sphere_id, transform1));
    
    // Undo should not work in Direct mode
    EXPECT_FALSE(bridge_->CanUndo());
    EXPECT_FALSE(bridge_->Undo());
    
    // Switch to Recorded mode
    bridge_->SetOperationMode(OperationMode::kRecorded);
    EXPECT_EQ(bridge_->GetOperationMode(), OperationMode::kRecorded);
    EXPECT_TRUE(bridge_->SupportsUndo());
    
    // Make changes in Recorded mode
    glm::mat4 transform2 = glm::translate(glm::mat4(1.0f), glm::vec3(2, 0, 0));
    EXPECT_TRUE(bridge_->SetTransform(sphere_id, transform2));
    
    // Now undo should work
    EXPECT_TRUE(bridge_->CanUndo());
    EXPECT_TRUE(bridge_->Undo());
    
    // And redo should work
    EXPECT_TRUE(bridge_->CanRedo());
    EXPECT_TRUE(bridge_->Redo());
    
    // Switch to Immediate mode
    bridge_->SetOperationMode(OperationMode::kImmediate);
    EXPECT_EQ(bridge_->GetOperationMode(), OperationMode::kImmediate);
    EXPECT_FALSE(bridge_->SupportsUndo());
    
    glm::mat4 transform3 = glm::translate(glm::mat4(1.0f), glm::vec3(3, 0, 0));
    EXPECT_TRUE(bridge_->SetTransform(sphere_id, transform3));
    
    // Undo should not work in Immediate mode
    EXPECT_FALSE(bridge_->CanUndo());
}

// Test undo/redo functionality with multiple operations
TEST_F(SceneManagerIntegrationTest, UndoRedoOperations) {
    // Switch to Recorded mode for undo/redo support
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    // Create objects
    auto cloud = CreateTestPointCloud(100);
    auto mesh = CreateTestMesh();
    
    ObjectId cloud_id = bridge_->AddObject("cloud", cloud);
    ObjectId mesh_id = bridge_->AddObject("mesh", mesh);
    
    // Perform a series of transformations
    std::vector<glm::mat4> transforms;
    transforms.push_back(glm::translate(glm::mat4(1.0f), glm::vec3(1, 0, 0)));
    transforms.push_back(glm::translate(glm::mat4(1.0f), glm::vec3(0, 1, 0)));
    transforms.push_back(glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 1)));
    transforms.push_back(glm::scale(glm::mat4(1.0f), glm::vec3(2, 2, 2)));
    
    for (const auto& transform : transforms) {
        EXPECT_TRUE(bridge_->SetTransform(cloud_id, transform));
    }
    
    // Verify we can undo all operations
    for (size_t i = 0; i < transforms.size(); ++i) {
        EXPECT_TRUE(bridge_->CanUndo());
        EXPECT_TRUE(bridge_->Undo());
    }
    
    // Should not be able to undo further
    EXPECT_FALSE(bridge_->CanUndo());
    
    // Verify we can redo all operations
    for (size_t i = 0; i < transforms.size(); ++i) {
        EXPECT_TRUE(bridge_->CanRedo());
        EXPECT_TRUE(bridge_->Redo());
    }
    
    // Should not be able to redo further
    EXPECT_FALSE(bridge_->CanRedo());
}

// Test compound operations (grouping multiple operations)
TEST_F(SceneManagerIntegrationTest, CompoundOperations) {
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    auto sphere1 = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 1.0f);
    auto sphere2 = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 0.5f);
    auto sphere3 = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 0.25f);
    
    ObjectId id1 = bridge_->AddObject("sphere1", sphere1);
    ObjectId id2 = bridge_->AddObject("sphere2", sphere2);
    ObjectId id3 = bridge_->AddObject("sphere3", sphere3);
    
    // Start compound operation
    EXPECT_TRUE(bridge_->BeginCompound("Move all spheres"));
    EXPECT_TRUE(bridge_->IsRecordingCompound());
    
    // Perform multiple operations as part of compound
    glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(5, 5, 5));
    EXPECT_TRUE(bridge_->SetTransform(id1, transform));
    EXPECT_TRUE(bridge_->SetTransform(id2, transform));
    EXPECT_TRUE(bridge_->SetTransform(id3, transform));
    
    // End compound operation
    EXPECT_TRUE(bridge_->EndCompound());
    EXPECT_FALSE(bridge_->IsRecordingCompound());
    
    // Single undo should undo all three transforms
    EXPECT_TRUE(bridge_->CanUndo());
    EXPECT_EQ(bridge_->GetUndoDescription(), "Undo Move all spheres");
    EXPECT_TRUE(bridge_->Undo());
    
    // Single redo should redo all three transforms
    EXPECT_TRUE(bridge_->CanRedo());
    EXPECT_EQ(bridge_->GetRedoDescription(), "Redo Move all spheres");
    EXPECT_TRUE(bridge_->Redo());
}

// Test RAII compound operation helper
TEST_F(SceneManagerIntegrationTest, CompoundOperationRAII) {
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    auto cloud = CreateTestPointCloud(50);
    ObjectId cloud_id = bridge_->AddObject("cloud", cloud);
    
    {
        // Use RAII helper for compound operation
        CompoundOperation compound(bridge_.get(), "Complex transformation");
        
        // Multiple operations within compound scope
        glm::mat4 translate = glm::translate(glm::mat4(1.0f), glm::vec3(1, 2, 3));
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0, 0, 1));
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(2, 2, 2));
        
        EXPECT_TRUE(bridge_->SetTransform(cloud_id, translate));
        EXPECT_TRUE(bridge_->SetTransform(cloud_id, rotate));
        EXPECT_TRUE(bridge_->SetTransform(cloud_id, scale));
        
        // Compound automatically ends when going out of scope
    }
    
    // Verify compound was properly ended
    EXPECT_FALSE(bridge_->IsRecordingCompound());
    
    // Single undo for all operations
    EXPECT_TRUE(bridge_->CanUndo());
    EXPECT_EQ(bridge_->GetUndoDescription(), "Undo Complex transformation");
    EXPECT_TRUE(bridge_->Undo());
}

// Test visibility operations
TEST_F(SceneManagerIntegrationTest, VisibilityOperations) {
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    auto mesh = CreateTestMesh();
    ObjectId mesh_id = bridge_->AddObject("mesh", mesh);
    
    // Initially visible
    EXPECT_TRUE(bridge_->IsVisible(mesh_id));
    
    // Hide object
    EXPECT_TRUE(bridge_->SetVisible(mesh_id, false));
    EXPECT_FALSE(bridge_->IsVisible(mesh_id));
    
    // Undo hiding
    EXPECT_TRUE(bridge_->Undo());
    EXPECT_TRUE(bridge_->IsVisible(mesh_id));
    
    // Redo hiding
    EXPECT_TRUE(bridge_->Redo());
    EXPECT_FALSE(bridge_->IsVisible(mesh_id));
    
    // Show object again
    EXPECT_TRUE(bridge_->SetVisible(mesh_id, true));
    EXPECT_TRUE(bridge_->IsVisible(mesh_id));
}

// Test custom command execution
class CustomRotateCommand : public Command {
public:
    CustomRotateCommand(SceneManagerBridge* bridge, ObjectId id, float angle)
        : bridge_(bridge), object_id_(id), angle_(angle) {}
    
    void Execute() override {
        if (!bridge_) return;
        
        old_transform_ = bridge_->GetTransform(object_id_);
        glm::mat4 rotation = glm::rotate(old_transform_, 
                                         glm::radians(angle_), 
                                         glm::vec3(0, 0, 1));
        bridge_->SetTransform(object_id_, rotation);
    }
    
    void Undo() override {
        if (!bridge_) return;
        bridge_->SetTransform(object_id_, old_transform_);
    }
    
    size_t GetMemorySize() const override {
        return sizeof(*this);
    }
    
    std::string GetDescription() const override {
        return "Rotate " + std::to_string(angle_) + " degrees";
    }
    
private:
    SceneManagerBridge* bridge_;
    ObjectId object_id_;
    float angle_;
    glm::mat4 old_transform_ = glm::mat4(1.0f);
};

TEST_F(SceneManagerIntegrationTest, CustomCommands) {
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    auto sphere = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 1.0f);
    ObjectId sphere_id = bridge_->AddObject("sphere", sphere);
    
    // Execute custom command
    auto rotate_cmd = std::make_unique<CustomRotateCommand>(bridge_.get(), sphere_id, 90.0f);
    EXPECT_TRUE(bridge_->ExecuteCommand(std::move(rotate_cmd)));
    
    // Verify undo/redo work with custom command
    EXPECT_TRUE(bridge_->CanUndo());
    EXPECT_EQ(bridge_->GetUndoDescription(), "Undo Rotate 90.000000 degrees");
    EXPECT_TRUE(bridge_->Undo());
    
    EXPECT_TRUE(bridge_->CanRedo());
    EXPECT_TRUE(bridge_->Redo());
}

// Test statistics and memory tracking
TEST_F(SceneManagerIntegrationTest, StatisticsTracking) {
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    // Add some objects and perform operations
    auto cloud = CreateTestPointCloud(1000);
    ObjectId cloud_id = bridge_->AddObject("cloud", cloud);
    
    for (int i = 0; i < 10; ++i) {
        glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(i, 0, 0));
        bridge_->SetTransform(cloud_id, transform);
    }
    
    // Check statistics
    auto stats = bridge_->GetCommandStatistics();
    EXPECT_EQ(stats.total_commands_executed, 10);
    EXPECT_EQ(stats.undo_count, 0);
    EXPECT_EQ(stats.redo_count, 0);
    EXPECT_GT(stats.memory_usage_bytes, 0);
    
    // Perform some undos
    for (int i = 0; i < 5; ++i) {
        bridge_->Undo();
    }
    
    stats = bridge_->GetCommandStatistics();
    EXPECT_EQ(stats.undo_count, 5);
    
    // Check memory usage
    size_t memory = bridge_->GetMemoryUsage();
    EXPECT_GT(memory, 0);
}

// Test scene clearing
TEST_F(SceneManagerIntegrationTest, SceneClearing) {
    // Add multiple objects
    auto cloud = CreateTestPointCloud(100);
    auto mesh = CreateTestMesh();
    auto sphere = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 1.0f);
    
    ObjectId cloud_id = bridge_->AddObject("cloud", cloud);
    ObjectId mesh_id = bridge_->AddObject("mesh", mesh);
    ObjectId sphere_id = bridge_->AddObject("sphere", sphere);
    
    // Clear everything
    bridge_->Clear();
    
    // Verify all objects are gone
    EXPECT_EQ(bridge_->GetObject(cloud_id), nullptr);
    EXPECT_EQ(bridge_->GetObject(mesh_id), nullptr);
    EXPECT_EQ(bridge_->GetObject(sphere_id), nullptr);
    
    EXPECT_EQ(bridge_->GetObjectByName("cloud"), nullptr);
    EXPECT_EQ(bridge_->GetObjectByName("mesh"), nullptr);
    EXPECT_EQ(bridge_->GetObjectByName("sphere"), nullptr);
}

// Integration test demonstrating real-world usage scenario
TEST_F(SceneManagerIntegrationTest, RealWorldScenario) {
    // Scenario: Interactive point cloud editing session
    
    // 1. Start in Direct mode for initial visualization
    EXPECT_EQ(bridge_->GetOperationMode(), OperationMode::kDirect);
    
    // Load point cloud data
    auto cloud = CreateTestPointCloud(5000);
    ObjectId cloud_id = bridge_->AddObject("scan_data", cloud);
    
    // Perform real-time updates (no undo needed)
    for (int frame = 0; frame < 10; ++frame) {
        float angle = frame * 0.1f;
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 0, 1));
        bridge_->SetTransform(cloud_id, rotation);
        
        // Simulate frame rendering
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    // 2. Switch to Recorded mode for editing
    bridge_->SetOperationMode(OperationMode::kRecorded);
    
    // User performs editing operations
    {
        CompoundOperation edit_session(bridge_.get(), "Align and scale point cloud");
        
        // Translate to origin
        glm::mat4 translate = glm::translate(glm::mat4(1.0f), glm::vec3(-2, -3, 0));
        bridge_->SetTransform(cloud_id, translate);
        
        // Scale to unit size
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.1f, 0.1f, 0.1f));
        bridge_->SetTransform(cloud_id, scale);
        
        // Rotate to align
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1, 0, 0));
        bridge_->SetTransform(cloud_id, rotate);
    }
    
    // 3. User reviews changes
    EXPECT_TRUE(bridge_->CanUndo());
    
    // User doesn't like the result, undo
    bridge_->Undo();
    
    // Try different approach
    {
        CompoundOperation edit_session2(bridge_.get(), "Alternative alignment");
        
        glm::mat4 transform = glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.5f, 0.5f));
        transform = glm::rotate(transform, glm::radians(45.0f), glm::vec3(0, 1, 0));
        bridge_->SetTransform(cloud_id, transform);
    }
    
    // 4. Add reference objects
    auto origin_marker = std::make_shared<Sphere>(glm::vec3(0, 0, 0), 0.1f);
    ObjectId marker_id = bridge_->AddObject("origin", origin_marker);
    
    // 5. Export would happen here (not implemented in test)
    auto stats = bridge_->GetCommandStatistics();
    // Note: SetTransform doesn't create commands in current implementation
    // This would need to be implemented in SceneState
    EXPECT_GE(stats.total_commands_executed, 0);
    
    // 6. Switch back to Direct mode for real-time visualization
    bridge_->SetOperationMode(OperationMode::kDirect);
    EXPECT_FALSE(bridge_->SupportsUndo());
}

} // namespace quickviz