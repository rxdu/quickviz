/*
 * test_virtual_object_unit.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for VirtualObject base functionality (Step 1)
 * 
 * This test validates the basic VirtualObject and VirtualSphere implementation
 * without any rendering backend dependencies. Tests object properties, 
 * transforms, hit testing, and data conversion.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "vscene/virtual_sphere.hpp"
#include "vscene/render_interface.hpp"

using namespace quickviz;

class VirtualObjectTest : public ::testing::Test {
protected:
    void SetUp() override {
        sphere = std::make_unique<VirtualSphere>("test_sphere", 1.0f);
    }

    std::unique_ptr<VirtualSphere> sphere;
};

// Test basic object creation and properties
TEST_F(VirtualObjectTest, BasicProperties) {
    EXPECT_EQ(sphere->GetId(), "test_sphere");
    EXPECT_EQ(sphere->GetType(), VirtualObjectType::Sphere);
    EXPECT_FLOAT_EQ(sphere->GetRadius(), 1.0f);
    EXPECT_EQ(sphere->GetTessellation(), 16); // Default
    EXPECT_TRUE(sphere->GetState().visible);
    EXPECT_FALSE(sphere->GetState().selected);
    EXPECT_FALSE(sphere->GetState().hovered);
}

// Test position setting and transform matrix
TEST_F(VirtualObjectTest, PositionAndTransform) {
    glm::vec3 test_position(2.0f, 3.0f, 4.0f);
    sphere->SetPosition(test_position);
    
    // Check transform matrix has correct position
    glm::mat4 transform = sphere->GetState().transform;
    glm::vec3 position = glm::vec3(transform[3]);
    
    EXPECT_FLOAT_EQ(position.x, 2.0f);
    EXPECT_FLOAT_EQ(position.y, 3.0f);
    EXPECT_FLOAT_EQ(position.z, 4.0f);
}

// Test color and visibility
TEST_F(VirtualObjectTest, AppearanceProperties) {
    glm::vec3 test_color(0.5f, 0.7f, 0.9f);
    sphere->SetColor(test_color);
    sphere->SetVisible(false);
    
    const auto& state = sphere->GetState();
    EXPECT_EQ(state.color, test_color);
    EXPECT_FALSE(state.visible);
}

// Test selection and hover states
TEST_F(VirtualObjectTest, InteractionStates) {
    EXPECT_FALSE(sphere->GetState().selected);
    EXPECT_FALSE(sphere->GetState().hovered);
    
    sphere->SetSelected(true);
    EXPECT_TRUE(sphere->GetState().selected);
    
    sphere->SetHovered(true);  
    EXPECT_TRUE(sphere->GetState().hovered);
    
    sphere->SetSelected(false);
    sphere->SetHovered(false);
    EXPECT_FALSE(sphere->GetState().selected);
    EXPECT_FALSE(sphere->GetState().hovered);
}

// Test sphere-specific properties
TEST_F(VirtualObjectTest, SphereProperties) {
    sphere->SetRadius(2.5f);
    EXPECT_FLOAT_EQ(sphere->GetRadius(), 2.5f);
    
    sphere->SetTessellation(32);
    EXPECT_EQ(sphere->GetTessellation(), 32);
    
    // Test minimum radius enforcement
    sphere->SetRadius(-1.0f);
    EXPECT_GE(sphere->GetRadius(), 0.01f); // Should clamp to minimum
    
    // Test minimum tessellation enforcement  
    sphere->SetTessellation(2);
    EXPECT_GE(sphere->GetTessellation(), 4); // Should clamp to minimum
}

// Test bounding box calculation
TEST_F(VirtualObjectTest, BoundingBox) {
    sphere->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    sphere->SetRadius(0.5f);
    
    BoundingBox bounds = sphere->GetBounds();
    
    // Check bounds are centered on position with radius offset
    EXPECT_FLOAT_EQ(bounds.min.x, 0.5f);  // 1.0 - 0.5
    EXPECT_FLOAT_EQ(bounds.min.y, 1.5f);  // 2.0 - 0.5
    EXPECT_FLOAT_EQ(bounds.min.z, 2.5f);  // 3.0 - 0.5
    
    EXPECT_FLOAT_EQ(bounds.max.x, 1.5f);  // 1.0 + 0.5
    EXPECT_FLOAT_EQ(bounds.max.y, 2.5f);  // 2.0 + 0.5
    EXPECT_FLOAT_EQ(bounds.max.z, 3.5f);  // 3.0 + 0.5
    
    // Test bounding box utilities
    EXPECT_TRUE(bounds.Contains(glm::vec3(1.0f, 2.0f, 3.0f))); // Center
    EXPECT_FALSE(bounds.Contains(glm::vec3(0.0f, 0.0f, 0.0f))); // Outside
}

// Test ray-sphere intersection
TEST_F(VirtualObjectTest, HitTesting) {
    sphere->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    sphere->SetRadius(1.0f);
    
    // Ray from distance hitting center
    Ray ray_hit = { glm::vec3(-3.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f) };
    float distance_hit;
    EXPECT_TRUE(sphere->HitTest(ray_hit, distance_hit));
    EXPECT_FLOAT_EQ(distance_hit, 2.0f); // Should hit at distance 2 (3 - 1)
    
    // Ray missing sphere
    Ray ray_miss = { glm::vec3(-3.0f, 2.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f) };
    float distance_miss;
    EXPECT_FALSE(sphere->HitTest(ray_miss, distance_miss));
    
    // Ray starting inside sphere
    Ray ray_inside = { glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f) };
    float distance_inside;
    EXPECT_TRUE(sphere->HitTest(ray_inside, distance_inside));
    EXPECT_FLOAT_EQ(distance_inside, 1.0f); // Should hit at exit point
}

// Test backend data conversion
TEST_F(VirtualObjectTest, BackendDataConversion) {
    sphere->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    sphere->SetColor(glm::vec3(0.5f, 0.6f, 0.7f));
    sphere->SetRadius(1.5f);
    sphere->SetTessellation(24);
    sphere->SetSelected(true);
    sphere->SetVisible(false);
    
    VirtualObjectData data = sphere->GetBackendData();
    
    // Check transform  
    glm::vec3 position = glm::vec3(data.transform[3]);
    EXPECT_FLOAT_EQ(position.x, 1.0f);
    EXPECT_FLOAT_EQ(position.y, 2.0f);
    EXPECT_FLOAT_EQ(position.z, 3.0f);
    
    // Check appearance
    EXPECT_EQ(data.color, glm::vec3(0.5f, 0.6f, 0.7f));
    EXPECT_FALSE(data.visible);
    EXPECT_TRUE(data.highlighted); // Should be true because selected
    
    // Check geometry data
    EXPECT_FLOAT_EQ(data.geometry.radius, 1.5f);
    EXPECT_EQ(data.geometry.tessellation, 24);
}

// Test dirty flag system
TEST_F(VirtualObjectTest, DirtyFlagSystem) {
    // Initially should need update (newly created)
    EXPECT_TRUE(sphere->IsBackendUpdateNeeded());
    
    // Clear flag
    sphere->ClearBackendUpdateFlag();
    EXPECT_FALSE(sphere->IsBackendUpdateNeeded());
    
    // Modifying properties should set dirty flag
    sphere->SetPosition(glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_TRUE(sphere->IsBackendUpdateNeeded());
    
    sphere->ClearBackendUpdateFlag();
    sphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_TRUE(sphere->IsBackendUpdateNeeded());
    
    sphere->ClearBackendUpdateFlag();
    sphere->SetRadius(2.0f);
    EXPECT_TRUE(sphere->IsBackendUpdateNeeded());
}

// Test BoundingBox utilities
TEST(BoundingBoxTest, Utilities) {
    BoundingBox box;
    box.min = glm::vec3(-1.0f, -1.0f, -1.0f);
    box.max = glm::vec3(1.0f, 1.0f, 1.0f);
    
    // Test Contains
    EXPECT_TRUE(box.Contains(glm::vec3(0.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(box.Contains(glm::vec3(0.5f, 0.5f, 0.5f)));
    EXPECT_FALSE(box.Contains(glm::vec3(2.0f, 0.0f, 0.0f)));
    
    // Test GetCenter and GetSize
    EXPECT_EQ(box.GetCenter(), glm::vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.GetSize(), glm::vec3(2.0f, 2.0f, 2.0f));
    
    // Test ray intersection
    Ray ray = { glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f) };
    float distance;
    EXPECT_TRUE(box.Intersects(ray, distance));
    EXPECT_FLOAT_EQ(distance, 1.0f); // Should intersect at x = -1
}

/*
 * Expected Output:
 * 
 * [==========] Running 8 tests from 2 test suites.
 * [----------] Global test environment set-up.
 * [----------] 7 tests from VirtualObjectTest
 * [ RUN      ] VirtualObjectTest.BasicProperties
 * [       OK ] VirtualObjectTest.BasicProperties
 * [ RUN      ] VirtualObjectTest.PositionAndTransform
 * [       OK ] VirtualObjectTest.PositionAndTransform
 * [ RUN      ] VirtualObjectTest.AppearanceProperties
 * [       OK ] VirtualObjectTest.AppearanceProperties
 * [ RUN      ] VirtualObjectTest.InteractionStates
 * [       OK ] VirtualObjectTest.InteractionStates  
 * [ RUN      ] VirtualObjectTest.SphereProperties
 * [       OK ] VirtualObjectTest.SphereProperties
 * [ RUN      ] VirtualObjectTest.BoundingBox
 * [       OK ] VirtualObjectTest.BoundingBox
 * [ RUN      ] VirtualObjectTest.HitTesting
 * [       OK ] VirtualObjectTest.HitTesting
 * [ RUN      ] VirtualObjectTest.BackendDataConversion
 * [       OK ] VirtualObjectTest.BackendDataConversion
 * [ RUN      ] VirtualObjectTest.DirtyFlagSystem
 * [       OK ] VirtualObjectTest.DirtyFlagSystem
 * [----------] 7 tests from VirtualObjectTest (X ms total)
 * [----------] 1 test from BoundingBoxTest
 * [ RUN      ] BoundingBoxTest.Utilities
 * [       OK ] BoundingBoxTest.Utilities
 * [----------] 1 test from BoundingBoxTest (X ms total)
 * [----------] Global test environment tear-down.
 * [==========] 8 tests from 2 test suites ran. (X ms total)
 * [  PASSED  ] 8 tests.
 */