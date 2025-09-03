/**
 * @file test_scene_state.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Unit tests for SceneState modal operation modes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>

#include "scenegraph/state/scene_state.hpp"

namespace quickviz {
namespace test {

// Mock OpenGlObject for testing
class MockOpenGlObject : public OpenGlObject {
public:
    MOCK_METHOD(void, AllocateGpuResources, (), (override));
    MOCK_METHOD(void, ReleaseGpuResources, (), (noexcept, override));
    MOCK_METHOD(void, OnDraw, (const glm::mat4& projection, const glm::mat4& view, const glm::mat4& coord_transform), (override));
    MOCK_METHOD(bool, IsGpuResourcesAllocated, (), (const, noexcept, override));
    
    // We'll use the base class implementations for transform/visibility
    // since they have the actual state we want to test
};

class SceneStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        scene_state_ = std::make_unique<SceneState>();
        mock_object1_ = std::make_shared<MockOpenGlObject>();
        mock_object2_ = std::make_shared<MockOpenGlObject>();
    }
    
    void TearDown() override {
        scene_state_.reset();
    }
    
    std::unique_ptr<SceneState> scene_state_;
    std::shared_ptr<MockOpenGlObject> mock_object1_;
    std::shared_ptr<MockOpenGlObject> mock_object2_;
};

// === Basic Object Management Tests ===

TEST_F(SceneStateTest, ObjectRegistration) {
    EXPECT_EQ(scene_state_->GetObjectCount(), 0);
    
    ObjectId id1 = scene_state_->RegisterObject(mock_object1_);
    EXPECT_NE(id1, kInvalidObjectId);
    EXPECT_EQ(scene_state_->GetObjectCount(), 1);
    EXPECT_TRUE(scene_state_->HasObject(id1));
    EXPECT_EQ(scene_state_->GetObject(id1), mock_object1_);
    
    ObjectId id2 = scene_state_->RegisterObject(mock_object2_);
    EXPECT_NE(id2, kInvalidObjectId);
    EXPECT_NE(id1, id2);  // IDs should be unique
    EXPECT_EQ(scene_state_->GetObjectCount(), 2);
}

TEST_F(SceneStateTest, ObjectUnregistration) {
    ObjectId id = scene_state_->RegisterObject(mock_object1_);
    EXPECT_EQ(scene_state_->GetObjectCount(), 1);
    
    bool success = scene_state_->UnregisterObject(id);
    EXPECT_TRUE(success);
    EXPECT_EQ(scene_state_->GetObjectCount(), 0);
    EXPECT_FALSE(scene_state_->HasObject(id));
    EXPECT_EQ(scene_state_->GetObject(id), nullptr);
}

TEST_F(SceneStateTest, NullObjectRejection) {
    EXPECT_THROW(scene_state_->RegisterObject(nullptr), std::invalid_argument);
}

// === Mode Switching Tests ===

TEST_F(SceneStateTest, ModeManagement) {
    // Default mode should be Direct
    EXPECT_EQ(scene_state_->GetMode(), OperationMode::kDirect);
    EXPECT_FALSE(scene_state_->SupportsUndo());
    
    // Switch to Recorded mode
    scene_state_->SetMode(OperationMode::kRecorded);
    EXPECT_EQ(scene_state_->GetMode(), OperationMode::kRecorded);
    EXPECT_TRUE(scene_state_->SupportsUndo());
    
    // Switch to Immediate mode
    scene_state_->SetMode(OperationMode::kImmediate);
    EXPECT_EQ(scene_state_->GetMode(), OperationMode::kImmediate);
    EXPECT_FALSE(scene_state_->SupportsUndo());
}

// === Transform Operations Tests ===

TEST_F(SceneStateTest, TransformOperationsDirectMode) {
    scene_state_->SetMode(OperationMode::kDirect);
    ObjectId id = scene_state_->RegisterObject(mock_object1_);
    
    glm::mat4 new_transform = glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 2.0f, 3.0f));
    
    bool success = scene_state_->SetTransform(id, new_transform);
    EXPECT_TRUE(success);
    
    glm::mat4 retrieved_transform = scene_state_->GetTransform(id);
    EXPECT_EQ(retrieved_transform, new_transform);
    
    // In direct mode, no undo should be available
    EXPECT_FALSE(scene_state_->CanUndo());
}

TEST_F(SceneStateTest, TransformOperationsRecordedMode) {
    scene_state_->SetMode(OperationMode::kRecorded);
    ObjectId id = scene_state_->RegisterObject(mock_object1_);
    
    glm::mat4 original_transform = scene_state_->GetTransform(id);
    glm::mat4 new_transform = glm::translate(glm::mat4(1.0f), glm::vec3(5.0f, 10.0f, 15.0f));
    
    bool success = scene_state_->SetTransform(id, new_transform);
    EXPECT_TRUE(success);
    
    glm::mat4 retrieved_transform = scene_state_->GetTransform(id);
    EXPECT_EQ(retrieved_transform, new_transform);
    
    // In recorded mode, undo should be available
    EXPECT_TRUE(scene_state_->CanUndo());
    EXPECT_FALSE(scene_state_->CanRedo());
    
    // Test undo
    bool undo_success = scene_state_->Undo();
    EXPECT_TRUE(undo_success);
    
    glm::mat4 undone_transform = scene_state_->GetTransform(id);
    EXPECT_EQ(undone_transform, original_transform);
    
    // After undo, redo should be available
    EXPECT_FALSE(scene_state_->CanUndo());
    EXPECT_TRUE(scene_state_->CanRedo());
    
    // Test redo
    bool redo_success = scene_state_->Redo();
    EXPECT_TRUE(redo_success);
    
    glm::mat4 redone_transform = scene_state_->GetTransform(id);
    EXPECT_EQ(redone_transform, new_transform);
}

// === Visibility Operations Tests ===

TEST_F(SceneStateTest, VisibilityOperations) {
    scene_state_->SetMode(OperationMode::kRecorded);
    ObjectId id = scene_state_->RegisterObject(mock_object1_);
    
    // Objects should be visible by default
    EXPECT_TRUE(scene_state_->IsVisible(id));
    
    // Test hiding object
    bool success = scene_state_->SetVisible(id, false);
    EXPECT_TRUE(success);
    EXPECT_FALSE(scene_state_->IsVisible(id));
    
    // Test undo of visibility change
    EXPECT_TRUE(scene_state_->CanUndo());
    scene_state_->Undo();
    EXPECT_TRUE(scene_state_->IsVisible(id));
}

// === Compound Operations Tests ===

TEST_F(SceneStateTest, CompoundOperations) {
    scene_state_->SetMode(OperationMode::kRecorded);
    ObjectId id = scene_state_->RegisterObject(mock_object1_);
    
    glm::mat4 original_transform = scene_state_->GetTransform(id);
    glm::mat4 new_transform = glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 1.0f, 1.0f));
    
    // Begin compound operation
    bool begin_success = scene_state_->BeginCompound("Test Compound Operation");
    EXPECT_TRUE(begin_success);
    EXPECT_TRUE(scene_state_->IsRecordingCompound());
    
    // Perform multiple operations
    scene_state_->SetTransform(id, new_transform);
    scene_state_->SetVisible(id, false);
    
    // End compound operation
    bool end_success = scene_state_->EndCompound();
    EXPECT_TRUE(end_success);
    EXPECT_FALSE(scene_state_->IsRecordingCompound());
    
    // Verify both operations were applied
    EXPECT_EQ(scene_state_->GetTransform(id), new_transform);
    EXPECT_FALSE(scene_state_->IsVisible(id));
    
    // Single undo should revert both operations
    EXPECT_TRUE(scene_state_->CanUndo());
    scene_state_->Undo();
    
    EXPECT_EQ(scene_state_->GetTransform(id), original_transform);
    EXPECT_TRUE(scene_state_->IsVisible(id));
}

// Note: CompoundScope RAII test commented out - would need access to internal CommandStack
// CompoundScope is designed for direct CommandStack usage, not SceneState

// === Configuration Tests ===

TEST_F(SceneStateTest, ConfigurationManagement) {
    SceneState::Config config;
    config.mode = OperationMode::kRecorded;
    config.max_commands = 500;
    config.memory_limit_bytes = 50 * 1024 * 1024;
    config.enable_change_notifications = true;
    
    scene_state_->SetConfig(config);
    
    const auto& retrieved_config = scene_state_->GetConfig();
    EXPECT_EQ(retrieved_config.mode, OperationMode::kRecorded);
    EXPECT_EQ(retrieved_config.max_commands, 500);
    EXPECT_EQ(retrieved_config.memory_limit_bytes, 50 * 1024 * 1024);
    EXPECT_TRUE(retrieved_config.enable_change_notifications);
}

// === Error Handling Tests ===

TEST_F(SceneStateTest, InvalidObjectOperations) {
    ObjectId invalid_id = 999999;
    
    // Operations on non-existent objects should fail gracefully
    EXPECT_FALSE(scene_state_->SetTransform(invalid_id, glm::mat4(1.0f)));
    EXPECT_FALSE(scene_state_->SetVisible(invalid_id, false));
    EXPECT_FALSE(scene_state_->HasObject(invalid_id));
    EXPECT_EQ(scene_state_->GetObject(invalid_id), nullptr);
    
    // Transform should return identity for non-existent object
    glm::mat4 transform = scene_state_->GetTransform(invalid_id);
    EXPECT_EQ(transform, glm::mat4(1.0f));
    
    // Visibility should return false for non-existent object
    EXPECT_FALSE(scene_state_->IsVisible(invalid_id));
}

TEST_F(SceneStateTest, UndoRedoInNonRecordedMode) {
    scene_state_->SetMode(OperationMode::kDirect);
    
    // Undo/redo operations should fail in non-recorded modes
    EXPECT_FALSE(scene_state_->CanUndo());
    EXPECT_FALSE(scene_state_->CanRedo());
    EXPECT_FALSE(scene_state_->Undo());
    EXPECT_FALSE(scene_state_->Redo());
    EXPECT_TRUE(scene_state_->GetUndoDescription().empty());
    EXPECT_TRUE(scene_state_->GetRedoDescription().empty());
}

// === Memory and Statistics Tests ===

TEST_F(SceneStateTest, MemoryUsageTracking) {
    size_t initial_usage = scene_state_->GetMemoryUsage();
    EXPECT_GT(initial_usage, 0);
    
    // Register objects and verify memory usage increases
    ObjectId id1 = scene_state_->RegisterObject(mock_object1_);
    ObjectId id2 = scene_state_->RegisterObject(mock_object2_);
    
    size_t usage_with_objects = scene_state_->GetMemoryUsage();
    EXPECT_GT(usage_with_objects, initial_usage);
}

TEST_F(SceneStateTest, ClearOperation) {
    ObjectId id1 = scene_state_->RegisterObject(mock_object1_);
    ObjectId id2 = scene_state_->RegisterObject(mock_object2_);
    
    EXPECT_EQ(scene_state_->GetObjectCount(), 2);
    
    scene_state_->Clear();
    
    EXPECT_EQ(scene_state_->GetObjectCount(), 0);
    EXPECT_FALSE(scene_state_->HasObject(id1));
    EXPECT_FALSE(scene_state_->HasObject(id2));
}

} // namespace test
} // namespace quickviz