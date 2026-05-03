/*
 * @file test_geometric_primitive_types.cpp
 * @date 2025-08-26
 * @brief Unit tests for GeometricPrimitive types and enums (minimal OpenGL-free tests)
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "scene/renderable/geometric_primitive.hpp"

using namespace quickviz;

// =================================================================
// Material Structure Tests
// =================================================================

TEST(GeometricPrimitiveTypesTest, MaterialStructDefaults) {
    GeometricPrimitive::Material material;
    
    // Check default material values
    EXPECT_EQ(material.diffuse_color, glm::vec3(0.7f, 0.7f, 0.9f));
    EXPECT_EQ(material.wireframe_color, glm::vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(material.highlight_color, glm::vec3(1.0f, 1.0f, 0.0f));
    EXPECT_FLOAT_EQ(material.opacity, 1.0f);
    EXPECT_FLOAT_EQ(material.metallic, 0.0f);
    EXPECT_FLOAT_EQ(material.roughness, 0.5f);
    EXPECT_FLOAT_EQ(material.ambient_factor, 0.1f);
    EXPECT_FLOAT_EQ(material.diffuse_factor, 0.7f);
    EXPECT_FLOAT_EQ(material.specular_factor, 0.2f);
    EXPECT_TRUE(material.use_lighting);
    EXPECT_TRUE(material.cast_shadows);
    EXPECT_TRUE(material.receive_shadows);
}

TEST(GeometricPrimitiveTypesTest, MaterialStructAssignment) {
    GeometricPrimitive::Material material1;
    GeometricPrimitive::Material material2;
    
    // Modify material1
    material1.diffuse_color = glm::vec3(1.0f, 0.0f, 0.0f);
    material1.opacity = 0.5f;
    material1.metallic = 0.8f;
    material1.use_lighting = false;
    
    // Copy to material2
    material2 = material1;
    
    // Verify copy
    EXPECT_EQ(material2.diffuse_color, material1.diffuse_color);
    EXPECT_FLOAT_EQ(material2.opacity, material1.opacity);
    EXPECT_FLOAT_EQ(material2.metallic, material1.metallic);
    EXPECT_EQ(material2.use_lighting, material1.use_lighting);
}

// =================================================================
// Enum Tests
// =================================================================

TEST(GeometricPrimitiveTypesTest, RenderModeEnum) {
    // Test all render mode values exist
    GeometricPrimitive::RenderMode solid = GeometricPrimitive::RenderMode::kSolid;
    GeometricPrimitive::RenderMode wireframe = GeometricPrimitive::RenderMode::kWireframe;
    GeometricPrimitive::RenderMode transparent = GeometricPrimitive::RenderMode::kTransparent;
    GeometricPrimitive::RenderMode points = GeometricPrimitive::RenderMode::kPoints;
    GeometricPrimitive::RenderMode outline = GeometricPrimitive::RenderMode::kOutline;
    
    // Test they have different values
    EXPECT_NE(static_cast<int>(solid), static_cast<int>(wireframe));
    EXPECT_NE(static_cast<int>(solid), static_cast<int>(transparent));
    EXPECT_NE(static_cast<int>(solid), static_cast<int>(points));
    EXPECT_NE(static_cast<int>(solid), static_cast<int>(outline));
}

TEST(GeometricPrimitiveTypesTest, BlendModeEnum) {
    // Test all blend mode values exist
    GeometricPrimitive::BlendMode opaque = GeometricPrimitive::BlendMode::kOpaque;
    GeometricPrimitive::BlendMode alpha = GeometricPrimitive::BlendMode::kAlpha;
    GeometricPrimitive::BlendMode additive = GeometricPrimitive::BlendMode::kAdditive;
    GeometricPrimitive::BlendMode multiply = GeometricPrimitive::BlendMode::kMultiply;
    
    // Test they have different values
    EXPECT_NE(static_cast<int>(opaque), static_cast<int>(alpha));
    EXPECT_NE(static_cast<int>(opaque), static_cast<int>(additive));
    EXPECT_NE(static_cast<int>(opaque), static_cast<int>(multiply));
}

// =================================================================
// Utility Function Tests
// =================================================================

TEST(GeometricPrimitiveTypesTest, RenderModeToString) {
    // Test render mode to string conversion
    EXPECT_STREQ(RenderModeToString(GeometricPrimitive::RenderMode::kSolid), "Solid");
    EXPECT_STREQ(RenderModeToString(GeometricPrimitive::RenderMode::kWireframe), "Wireframe");
    EXPECT_STREQ(RenderModeToString(GeometricPrimitive::RenderMode::kTransparent), "Transparent");
    EXPECT_STREQ(RenderModeToString(GeometricPrimitive::RenderMode::kPoints), "Points");
    EXPECT_STREQ(RenderModeToString(GeometricPrimitive::RenderMode::kOutline), "Outline");
}

TEST(GeometricPrimitiveTypesTest, BlendModeToString) {
    // Test blend mode to string conversion
    EXPECT_STREQ(BlendModeToString(GeometricPrimitive::BlendMode::kOpaque), "Opaque");
    EXPECT_STREQ(BlendModeToString(GeometricPrimitive::BlendMode::kAlpha), "Alpha");
    EXPECT_STREQ(BlendModeToString(GeometricPrimitive::BlendMode::kAdditive), "Additive");
    EXPECT_STREQ(BlendModeToString(GeometricPrimitive::BlendMode::kMultiply), "Multiply");
}

// =================================================================
// GLM Math Integration Tests
// =================================================================

TEST(GeometricPrimitiveTypesTest, MaterialVec3Math) {
    GeometricPrimitive::Material material;
    
    // Test vec3 operations work with material colors
    glm::vec3 red(1.0f, 0.0f, 0.0f);
    glm::vec3 green(0.0f, 1.0f, 0.0f);
    glm::vec3 yellow = red + green;
    
    material.diffuse_color = yellow;
    EXPECT_EQ(material.diffuse_color, glm::vec3(1.0f, 1.0f, 0.0f));
    
    // Test color scaling
    glm::vec3 dimmed = material.diffuse_color * 0.5f;
    EXPECT_EQ(dimmed, glm::vec3(0.5f, 0.5f, 0.0f));
}

TEST(GeometricPrimitiveTypesTest, BoundingBoxMath) {
    // Test bounding box calculations
    glm::vec3 min_bounds(-1.0f, -1.0f, -1.0f);
    glm::vec3 max_bounds(1.0f, 1.0f, 1.0f);
    
    // Test bounding box validity
    EXPECT_LE(min_bounds.x, max_bounds.x);
    EXPECT_LE(min_bounds.y, max_bounds.y);
    EXPECT_LE(min_bounds.z, max_bounds.z);
    
    // Test bounding box size calculation
    glm::vec3 size = max_bounds - min_bounds;
    EXPECT_EQ(size, glm::vec3(2.0f, 2.0f, 2.0f));
    
    // Test bounding box center calculation
    glm::vec3 center = (min_bounds + max_bounds) * 0.5f;
    EXPECT_EQ(center, glm::vec3(0.0f, 0.0f, 0.0f));
}

// =================================================================
// Geometry Math Tests (without actual objects)
// =================================================================

TEST(GeometricPrimitiveTypesTest, SphereMath) {
    // Test sphere volume and surface area calculations
    const float radius = 1.0f;
    const float expected_volume = (4.0f / 3.0f) * M_PI * radius * radius * radius;
    const float expected_surface_area = 4.0f * M_PI * radius * radius;
    
    // Just test the math constants work correctly
    EXPECT_NEAR(expected_volume, 4.188787, 0.001);
    EXPECT_NEAR(expected_surface_area, 12.566370, 0.001);
}

TEST(GeometricPrimitiveTypesTest, CylinderMath) {
    // Test cylinder volume and surface area calculations
    const float radius = 1.0f;
    const float height = 2.0f;
    const float expected_volume = M_PI * radius * radius * height;
    const float expected_surface_area = 2.0f * M_PI * radius * (radius + height);
    
    // Just test the math constants work correctly
    EXPECT_NEAR(expected_volume, 6.283185, 0.001);
    EXPECT_NEAR(expected_surface_area, 18.849555, 0.001);
}

TEST(GeometricPrimitiveTypesTest, BoxMath) {
    // Test bounding box volume and surface area calculations
    const float width = 2.0f;
    const float height = 2.0f;
    const float depth = 2.0f;
    const float expected_volume = width * height * depth;
    const float expected_surface_area = 2.0f * (width * height + width * depth + height * depth);
    
    EXPECT_FLOAT_EQ(expected_volume, 8.0f);
    EXPECT_FLOAT_EQ(expected_surface_area, 24.0f);
}

// =================================================================
// Transform Math Tests
// =================================================================

TEST(GeometricPrimitiveTypesTest, TransformMath) {
    // Test GLM transform operations work
    glm::mat4 identity = glm::mat4(1.0f);
    glm::mat4 translation = glm::translate(identity, glm::vec3(5.0f, 3.0f, 1.0f));
    glm::mat4 rotation = glm::rotate(identity, glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 scale = glm::scale(identity, glm::vec3(2.0f, 2.0f, 2.0f));
    
    // Test matrix multiplication
    glm::mat4 transform = translation * rotation * scale;
    
    // Test that transform is not identity
    EXPECT_NE(transform, identity);
    
    // Test basic matrix properties
    EXPECT_FLOAT_EQ(identity[0][0], 1.0f);
    EXPECT_FLOAT_EQ(identity[1][1], 1.0f);
    EXPECT_FLOAT_EQ(identity[2][2], 1.0f);
    EXPECT_FLOAT_EQ(identity[3][3], 1.0f);
}