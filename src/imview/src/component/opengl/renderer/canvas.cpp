/**
 * @file canvas.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/renderer/canvas.hpp"

#include "glad/glad.h"

#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

namespace quickviz {
namespace {
std::string vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aColor;
layout (location = 2) in float aSize;

uniform mat4 projection;
uniform mat4 view;

out vec4 vertexColor;

void main()
{
    // Place points in the X-Z plane (y=0) for consistency with 3D mode
    // This makes the points appear in the same reference frame as the grid
    vec4 clipPos = projection * view * vec4(aPos.x, 0.0, aPos.y, 1.0);
    gl_Position = clipPos;
    
    // Use the input size directly
    gl_PointSize = aSize;
    
    vertexColor = aColor;
}
)";

std::string fragment_shader_source = R"(
#version 330 core
in vec4 vertexColor;
out vec4 FragColor;

void main()
{
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
    float x, y;
    float r, g, b, a;
    float size;
};

Canvas::Canvas(float width, float height) : width_(width), height_(height) {
    AllocateGpuResources();
}

Canvas::~Canvas() {
    ReleaseGpuResources();
}

void Canvas::AllocateGpuResources() {
    // Create and compile shaders
    Shader vertex_shader(vertex_shader_source.c_str(), Shader::Type::kVertex);
    Shader fragment_shader(fragment_shader_source.c_str(), Shader::Type::kFragment);
    
    shader_.AttachShader(vertex_shader);
    shader_.AttachShader(fragment_shader);
    
    if (!shader_.LinkProgram()) {
        std::cerr << "ERROR::CANVAS::SHADER_PROGRAM_LINKING_FAILED" << std::endl;
        throw std::runtime_error("Shader program linking failed");
    }
    
    // Generate VAO and VBO
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    // Setup VAO and VBO
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    
    // Position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Color attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    // Size attribute
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    
    // Unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Canvas::ReleaseGpuResources() {
    // Delete VAO and VBO
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
}

void Canvas::OnDraw(const glm::mat4& projection, const glm::mat4& view) {
    if (points_.empty()) {
        return;
    }
    
    // Extract scale from view matrix
    float scaleX = view[0][0];
    float scaleY = view[1][1];
    float current_scale = (std::abs(scaleX) + std::abs(scaleY)) * 0.5f;
    
    // Apply a scaling factor to make the effect more pronounced
    float scaling_factor = 5.0f;
    
    // Create a copy of the points with scaled sizes
    std::vector<Point> scaled_points = points_;
    for (size_t i = 0; i < scaled_points.size(); ++i) {
        // Scale the point size based on the current scale
        scaled_points[i].size = original_sizes_[i] * current_scale * scaling_factor;
        
        // Ensure a minimum size
        if (scaled_points[i].size < 3.0f) {
            scaled_points[i].size = 3.0f;
        }
    }
    
    // Use shader program
    shader_.Use();
    
    // Set uniforms
    shader_.SetUniform("projection", projection);
    shader_.SetUniform("view", view);
    
    // Bind VAO
    glBindVertexArray(vao_);
    
    // Update VBO data with scaled points
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, scaled_points.size() * sizeof(Point), scaled_points.data(), GL_DYNAMIC_DRAW);
    
    // Enable point size
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    // Draw points
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(scaled_points.size()));
    
    // Disable point size
    glDisable(GL_PROGRAM_POINT_SIZE);
    
    // Unbind VAO
    glBindVertexArray(0);
}

void Canvas::AddPoint(float x, float y, const glm::vec4& color,
                      float thickness) {
    // Create a new point
    Point point;
    point.x = x;
    point.y = y;
    point.r = color.r;
    point.g = color.g;
    point.b = color.b;
    point.a = color.a;
    point.size = thickness;
    
    // Add point to the vector
    points_.push_back(point);
    
    // Store the original size
    original_sizes_.push_back(thickness);
}

void Canvas::Clear() {
    // Clear all points
    points_.clear();
    original_sizes_.clear();
}
}  // namespace quickviz
