/*
 * utest_object_types.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for VirtualObjectType enum and utilities
 * 
 * Tests the type-safe enum system for virtual objects including
 * conversions, utility functions, and type categorization.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <string>

#include "vscene/virtual_object_types.hpp"

using namespace quickviz;

// Test basic enum values and conversions
TEST(VirtualObjectTypesTest, BasicEnumValues) {
    EXPECT_EQ(static_cast<int>(VirtualObjectType::Sphere), 0);
    EXPECT_EQ(static_cast<int>(VirtualObjectType::Box), 1);
    EXPECT_EQ(static_cast<int>(VirtualObjectType::Mesh), 100);
    EXPECT_EQ(static_cast<int>(VirtualObjectType::CoordinateFrame), 200);
    EXPECT_EQ(static_cast<int>(VirtualObjectType::Group), 300);
    EXPECT_EQ(static_cast<int>(VirtualObjectType::Custom), 1000);
}

// Test ToString conversion
TEST(VirtualObjectTypesTest, ToStringConversion) {
    EXPECT_STREQ(ToString(VirtualObjectType::Sphere), "sphere");
    EXPECT_STREQ(ToString(VirtualObjectType::Box), "box");
    EXPECT_STREQ(ToString(VirtualObjectType::Cylinder), "cylinder");
    EXPECT_STREQ(ToString(VirtualObjectType::Mesh), "mesh");
    EXPECT_STREQ(ToString(VirtualObjectType::CoordinateFrame), "coordinateframe");
    EXPECT_STREQ(ToString(VirtualObjectType::Custom), "custom");
}

// Test FromString conversion
TEST(VirtualObjectTypesTest, FromStringConversion) {
    EXPECT_EQ(FromString("sphere"), VirtualObjectType::Sphere);
    EXPECT_EQ(FromString("box"), VirtualObjectType::Box);
    EXPECT_EQ(FromString("cylinder"), VirtualObjectType::Cylinder);
    EXPECT_EQ(FromString("mesh"), VirtualObjectType::Mesh);
    EXPECT_EQ(FromString("coordinateframe"), VirtualObjectType::CoordinateFrame);
    EXPECT_EQ(FromString("custom"), VirtualObjectType::Custom);
    
    // Unknown strings should return Custom
    EXPECT_EQ(FromString("unknown"), VirtualObjectType::Custom);
    EXPECT_EQ(FromString("invalid"), VirtualObjectType::Custom);
    EXPECT_EQ(FromString(""), VirtualObjectType::Custom);
}

// Test round-trip conversion (ToString -> FromString)
TEST(VirtualObjectTypesTest, RoundTripConversion) {
    VirtualObjectType types[] = {
        VirtualObjectType::Sphere,
        VirtualObjectType::Box,
        VirtualObjectType::Cylinder,
        VirtualObjectType::Mesh,
        VirtualObjectType::PointCloud,
        VirtualObjectType::CoordinateFrame,
        VirtualObjectType::Arrow,
        VirtualObjectType::Group,
        VirtualObjectType::Custom
    };
    
    for (auto type : types) {
        std::string type_str = ToString(type);
        VirtualObjectType converted_back = FromString(type_str);
        EXPECT_EQ(converted_back, type) << "Round-trip failed for " << type_str;
    }
}

// Test type categorization functions
TEST(VirtualObjectTypesTest, TypeCategorization) {
    // Geometric primitives
    EXPECT_TRUE(IsGeometricPrimitive(VirtualObjectType::Sphere));
    EXPECT_TRUE(IsGeometricPrimitive(VirtualObjectType::Box));
    EXPECT_TRUE(IsGeometricPrimitive(VirtualObjectType::Cylinder));
    EXPECT_TRUE(IsGeometricPrimitive(VirtualObjectType::Plane));
    
    // Complex shapes
    EXPECT_TRUE(IsComplexShape(VirtualObjectType::Mesh));
    EXPECT_TRUE(IsComplexShape(VirtualObjectType::PointCloud));
    EXPECT_TRUE(IsComplexShape(VirtualObjectType::LineStrip));
    
    // Composite objects
    EXPECT_TRUE(IsCompositeObject(VirtualObjectType::CoordinateFrame));
    EXPECT_TRUE(IsCompositeObject(VirtualObjectType::Arrow));
    EXPECT_TRUE(IsCompositeObject(VirtualObjectType::Billboard));
    
    // Cross-category checks
    EXPECT_FALSE(IsGeometricPrimitive(VirtualObjectType::Mesh));
    EXPECT_FALSE(IsComplexShape(VirtualObjectType::Sphere));
    EXPECT_FALSE(IsCompositeObject(VirtualObjectType::Box));
    
    // Special cases
    EXPECT_FALSE(IsGeometricPrimitive(VirtualObjectType::Group));
    EXPECT_FALSE(IsComplexShape(VirtualObjectType::Custom));
    EXPECT_FALSE(IsCompositeObject(VirtualObjectType::Custom));
}

// Test type safety - compile-time checks
TEST(VirtualObjectTypesTest, TypeSafety) {
    // These should all compile without issues
    VirtualObjectType type = VirtualObjectType::Sphere;
    
    // Test that we can compare types safely
    EXPECT_EQ(type, VirtualObjectType::Sphere);
    EXPECT_NE(type, VirtualObjectType::Box);
    
    // Test switch statement works
    const char* result;
    switch (type) {
        case VirtualObjectType::Sphere:
            result = "Found sphere";
            break;
        case VirtualObjectType::Box:
            result = "Found box";
            break;
        default:
            result = "Found other";
            break;
    }
    EXPECT_STREQ(result, "Found sphere");
}

// Test performance - enum comparisons should be fast
TEST(VirtualObjectTypesTest, PerformanceCheck) {
    VirtualObjectType type1 = VirtualObjectType::Sphere;
    VirtualObjectType type2 = VirtualObjectType::Box;
    VirtualObjectType type3 = VirtualObjectType::Sphere;
    
    // These are integer comparisons and should be very fast
    EXPECT_TRUE(type1 == type3);
    EXPECT_FALSE(type1 == type2);
    
    // Demonstrate that this is faster than string comparison
    // (In a real performance test, you'd measure actual timing)
    bool found_match = false;
    for (int i = 0; i < 1000; ++i) {
        if (type1 == VirtualObjectType::Sphere) {
            found_match = true;
        }
    }
    EXPECT_TRUE(found_match);
}