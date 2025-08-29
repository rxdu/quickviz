/*
 * @file virtual_object_types.hpp
 * @date August 27, 2025
 * @brief Type definitions for virtual objects
 *
 * Defines the supported virtual object types using type-safe enums
 * and utility functions for type conversions.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VIRTUAL_OBJECT_TYPES_HPP
#define QUICKVIZ_VIRTUAL_OBJECT_TYPES_HPP

#include <string>
#include <unordered_map>

namespace quickviz {

/**
 * @brief Supported virtual object types
 * 
 * Using enum class provides compile-time safety and better performance
 * compared to string-based type identification.
 */
enum class VirtualObjectType : int {
    // Geometric primitives
    Sphere = 0,
    Box = 1,
    Cylinder = 2,
    Capsule = 3,
    Plane = 4,
    
    // Complex shapes
    Mesh = 100,
    PointCloud = 101,
    LineStrip = 102,
    Triangle = 103,
    
    // Composite objects
    CoordinateFrame = 200,
    Arrow = 201,
    Billboard = 202,  // Replaces deprecated Text3D
    
    // Special types
    Group = 300,        // Container for multiple objects
    Custom = 1000       // For user-defined types
};

/**
 * @brief Convert VirtualObjectType to string representation
 * 
 * Useful for logging, serialization, and backend communication
 * where string identifiers are needed.
 */
inline const char* ToString(VirtualObjectType type) {
    switch (type) {
        case VirtualObjectType::Sphere: return "sphere";
        case VirtualObjectType::Box: return "box";
        case VirtualObjectType::Cylinder: return "cylinder";
        case VirtualObjectType::Capsule: return "capsule";
        case VirtualObjectType::Plane: return "plane";
        case VirtualObjectType::Mesh: return "mesh";
        case VirtualObjectType::PointCloud: return "pointcloud";
        case VirtualObjectType::LineStrip: return "linestrip";
        case VirtualObjectType::Triangle: return "triangle";
        case VirtualObjectType::CoordinateFrame: return "coordinateframe";
        case VirtualObjectType::Arrow: return "arrow";
        case VirtualObjectType::Billboard: return "billboard";
        case VirtualObjectType::Group: return "group";
        case VirtualObjectType::Custom: return "custom";
        default: return "unknown";
    }
}

/**
 * @brief Convert string to VirtualObjectType
 * 
 * Useful for parsing configuration files or user input.
 * Returns VirtualObjectType::Custom for unknown strings.
 */
inline VirtualObjectType FromString(const std::string& type_str) {
    static const std::unordered_map<std::string, VirtualObjectType> type_map = {
        {"sphere", VirtualObjectType::Sphere},
        {"box", VirtualObjectType::Box},
        {"cylinder", VirtualObjectType::Cylinder},
        {"capsule", VirtualObjectType::Capsule},
        {"plane", VirtualObjectType::Plane},
        {"mesh", VirtualObjectType::Mesh},
        {"pointcloud", VirtualObjectType::PointCloud},
        {"linestrip", VirtualObjectType::LineStrip},
        {"triangle", VirtualObjectType::Triangle},
        {"coordinateframe", VirtualObjectType::CoordinateFrame},
        {"arrow", VirtualObjectType::Arrow},
        {"billboard", VirtualObjectType::Billboard},
        {"group", VirtualObjectType::Group},
        {"custom", VirtualObjectType::Custom}
    };
    
    auto it = type_map.find(type_str);
    return (it != type_map.end()) ? it->second : VirtualObjectType::Custom;
}

/**
 * @brief Check if a type represents a geometric primitive
 */
inline bool IsGeometricPrimitive(VirtualObjectType type) {
    return static_cast<int>(type) >= 0 && static_cast<int>(type) < 100;
}

/**
 * @brief Check if a type represents a complex shape
 */
inline bool IsComplexShape(VirtualObjectType type) {
    return static_cast<int>(type) >= 100 && static_cast<int>(type) < 200;
}

/**
 * @brief Check if a type represents a composite object
 */
inline bool IsCompositeObject(VirtualObjectType type) {
    return static_cast<int>(type) >= 200 && static_cast<int>(type) < 300;
}

} // namespace quickviz

#endif // QUICKVIZ_VIRTUAL_OBJECT_TYPES_HPP