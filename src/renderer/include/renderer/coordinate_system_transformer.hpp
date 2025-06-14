/**
 * @file coordinate_system_transformer.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-17
 * @brief Handles transformations between different coordinate systems
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_COORDINATE_SYSTEM_TRANSFORMER_HPP
#define QUICKVIZ_COORDINATE_SYSTEM_TRANSFORMER_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace quickviz {

/**
 * @class CoordinateSystemTransformer
 * @brief Handles transformations between different coordinate systems
 * 
 * This class provides utilities to transform between the standard coordinate system
 * (Z-up, Y-forward, X-right) and the OpenGL coordinate system (Y-up, Z-forward, X-right).
 * 
 * The transformation is a -90 degree rotation around the X axis:
 * - Standard Z-up becomes OpenGL Y-up
 * - Standard Y-forward becomes OpenGL Z-forward
 * - Standard X-right remains OpenGL X-right
 */
class CoordinateSystemTransformer {
public:
    /**
     * @brief Get the transformation matrix from standard (Z-up) to OpenGL (Y-up)
     * 
     * @return glm::mat4 Transformation matrix
     */
    static glm::mat4 GetStandardToOpenGLTransform() {
        // Rotate -90 degrees around X axis: Z-up becomes Y-up, Y-forward becomes Z-forward
        return glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    }

    /**
     * @brief Get the transformation matrix from OpenGL (Y-up) to standard (Z-up)
     * 
     * @return glm::mat4 Transformation matrix
     */
    static glm::mat4 GetOpenGLToStandardTransform() {
        // Rotate 90 degrees around X axis: Y-up becomes Z-up, Z-forward becomes Y-forward
        return glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    }

    /**
     * @brief Transform a point from standard to OpenGL coordinate system
     * 
     * @param point Point in standard coordinate system
     * @return glm::vec3 Point in OpenGL coordinate system
     */
    static glm::vec3 TransformPointToOpenGL(const glm::vec3& point) {
        return glm::vec3(GetStandardToOpenGLTransform() * glm::vec4(point, 1.0f));
    }

    /**
     * @brief Transform a point from OpenGL to standard coordinate system
     * 
     * @param point Point in OpenGL coordinate system
     * @return glm::vec3 Point in standard coordinate system
     */
    static glm::vec3 TransformPointToStandard(const glm::vec3& point) {
        return glm::vec3(GetOpenGLToStandardTransform() * glm::vec4(point, 1.0f));
    }

    /**
     * @brief Transform a direction vector from standard to OpenGL coordinate system
     * 
     * @param direction Direction vector in standard coordinate system
     * @return glm::vec3 Direction vector in OpenGL coordinate system
     */
    static glm::vec3 TransformDirectionToOpenGL(const glm::vec3& direction) {
        return glm::vec3(GetStandardToOpenGLTransform() * glm::vec4(direction, 0.0f));
    }

    /**
     * @brief Transform a direction vector from OpenGL to standard coordinate system
     * 
     * @param direction Direction vector in OpenGL coordinate system
     * @return glm::vec3 Direction vector in standard coordinate system
     */
    static glm::vec3 TransformDirectionToStandard(const glm::vec3& direction) {
        return glm::vec3(GetOpenGLToStandardTransform() * glm::vec4(direction, 0.0f));
    }
};

} // namespace quickviz

#endif // QUICKVIZ_COORDINATE_SYSTEM_TRANSFORMER_HPP 