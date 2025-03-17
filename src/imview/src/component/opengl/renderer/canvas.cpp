/**
 * @file canvas.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/canvas.hpp"

#include <iostream>
#include <cmath>

#include "glad/glad.h"
#include <glm/gtc/type_ptr.hpp>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 coordSystemTransform;

out vec4 vertexColor;

void main() {
    gl_Position = projection * view * model * coordSystemTransform * vec4(aPos, 1.0);
    gl_PointSize = aSize;
    vertexColor = aColor;
}
)";

std::string fragment_shader_source = R"(
#version 330 core

in vec4 vertexColor;
out vec4 FragColor;

void main() {
    // Create a circular point by discarding fragments outside the circle
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    
    FragColor = vertexColor;
}
)";
}  // namespace

// Point structure to store point data
struct Point {
    glm::vec3 position;
    glm::vec4 color;
    float size;
};

Canvas::Canvas(float width, float height) : width_(width), height_(height) {
    AllocateGpuResources();
}

Canvas::~Canvas() {
    ReleaseGpuResources();
}

void Canvas::AddPoint(float x, float y, const glm::vec4& color, float thickness) {
    Point point;
    point.position = glm::vec3(x, y, 0.0f); // Use X-Y plane for 2D drawing
    point.color = color;
    point.size = thickness;
    
    points_.push_back(point);
    
    // Update the VBO with the new data
    if (vao_ != 0 && vbo_ != 0) {
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, points_.size() * sizeof(Point), points_.data(), GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
}

void Canvas::AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
                    float thickness, LineType line_type) {
    // For simplicity, we'll just add two points for now
    // In a more complete implementation, we would use line primitives
    AddPoint(x1, y1, color, thickness);
    AddPoint(x2, y2, color, thickness);
}

void Canvas::AddRectangle(float x, float y, float width, float height,
                         const glm::vec4& color, bool filled, float thickness,
                         LineType line_type) {
    // For simplicity, we'll just add four points for the corners
    // In a more complete implementation, we would use line or triangle primitives
    AddPoint(x, y, color, thickness);
    AddPoint(x + width, y, color, thickness);
    AddPoint(x + width, y + height, color, thickness);
    AddPoint(x, y + height, color, thickness);
}

void Canvas::AddCircle(float x, float y, float radius, const glm::vec4& color,
                      bool filled, float thickness, LineType line_type) {
    // For simplicity, we'll just add a single point at the center
    // In a more complete implementation, we would use triangle primitives
    AddPoint(x, y, color, radius * 2.0f);
}

void Canvas::AddEllipse(float x, float y, float rx, float ry, float angle,
                       float start_angle, float end_angle, const glm::vec4& color,
                       bool filled, float thickness, LineType line_type) {
    // For simplicity, we'll just add a single point at the center
    // In a more complete implementation, we would use triangle primitives
    AddPoint(x, y, color, std::max(rx, ry) * 2.0f);
}

void Canvas::AddPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color,
                       bool filled, float thickness, LineType line_type) {
    // For simplicity, we'll just add points for each vertex
    // In a more complete implementation, we would use line or triangle primitives
    for (const auto& point : points) {
        AddPoint(point.x, point.y, color, thickness);
    }
}

void Canvas::Clear() {
    points_.clear();
    
    // Update the VBO with the new data
    if (vao_ != 0 && vbo_ != 0) {
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
}

void Canvas::AllocateGpuResources() {
    // Compile and link shaders
    Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
    Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
    shader_.AttachShader(vertex_shader);
    shader_.AttachShader(fragment_shader);
    
    if (!shader_.LinkProgram()) {
        std::cerr << "ERROR::CANVAS::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
        throw std::runtime_error("Shader program linking failed");
    }
    
    // Create and set up VAO and VBO
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    glBindVertexArray(vao_);
    
    // Set up VBO for points
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    
    // Set up vertex attributes
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, position));
    glEnableVertexAttribArray(0);
    
    // Color attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, color));
    glEnableVertexAttribArray(1);
    
    // Size attribute
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, size));
    glEnableVertexAttribArray(2);
    
    // Unbind
    glBindVertexArray(0);
}

void Canvas::ReleaseGpuResources() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
}

void Canvas::OnDraw(const glm::mat4& projection, const glm::mat4& view, 
                   const glm::mat4& coord_transform) {
    if (points_.empty()) {
        return;
    }
    
    shader_.Use();
    shader_.SetUniform("projection", projection);
    shader_.SetUniform("view", view);
    shader_.SetUniform("model", glm::mat4(1.0f));
    shader_.SetUniform("coordSystemTransform", coord_transform);
    
    glBindVertexArray(vao_);
    
    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Enable point size
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    // Draw the points
    glDrawArrays(GL_POINTS, 0, points_.size());
    
    // Disable point size
    glDisable(GL_PROGRAM_POINT_SIZE);
    
    // Disable blending
    glDisable(GL_BLEND);
    
    glBindVertexArray(0);
}
}  // namespace quickviz
