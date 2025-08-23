/**
 * @file line_strip.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Implementation of line strip renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/line_strip.hpp"

#include <iostream>
#include <cmath>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {
const char* kLineVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 FragColor;

uniform mat4 mvp;
uniform bool usePerVertexColor;
uniform vec3 uniformColor;

void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
    FragColor = usePerVertexColor ? aColor : uniformColor;
}
)";

const char* kLineFragmentShader = R"(
#version 330 core
in vec3 FragColor;
out vec4 FinalColor;

void main() {
    FinalColor = vec4(FragColor, 1.0);
}
)";

const char* kPointVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;
uniform float pointSize;

void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
    gl_PointSize = pointSize;
}
)";

const char* kPointFragmentShader = R"(
#version 330 core
out vec4 FinalColor;
uniform vec3 pointColor;

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    if (length(coord) > 0.5)
        discard;
    FinalColor = vec4(pointColor, 1.0);
}
)";

const char* kArrowVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvp;

void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

const char* kArrowFragmentShader = R"(
#version 330 core
out vec4 FinalColor;
uniform vec3 arrowColor;

void main() {
    FinalColor = vec4(arrowColor, 1.0);
}
)";

}  // namespace

LineStrip::LineStrip() {}

LineStrip::~LineStrip() {
  if (IsGpuResourcesAllocated()) {
    ReleaseGpuResources();
  }
}

void LineStrip::SetPoints(const std::vector<glm::vec3>& points) {
  points_ = points;
  needs_update_ = true;
  needs_arrow_update_ = true;
}

void LineStrip::SetColors(const std::vector<glm::vec3>& colors) {
  colors_ = colors;
  use_per_vertex_colors_ = !colors.empty();
  needs_update_ = true;
}

void LineStrip::SetColor(const glm::vec3& color) {
  uniform_color_ = color;
  use_per_vertex_colors_ = false;
  needs_update_ = true;
}

void LineStrip::SetLineWidth(float width) {
  line_width_ = width;
}

void LineStrip::SetLineType(LineType type) {
  line_type_ = type;
}

void LineStrip::SetClosed(bool closed) {
  closed_ = closed;
  needs_update_ = true;
}

void LineStrip::SetShowPoints(bool show, float point_size) {
  show_points_ = show;
  point_size_ = point_size;
}

void LineStrip::SetShowArrows(bool show, float spacing) {
  show_arrows_ = show;
  arrow_spacing_ = spacing;
  needs_arrow_update_ = true;
}

void LineStrip::SetArrowSize(float size) {
  arrow_size_ = size;
  needs_arrow_update_ = true;
}

float LineStrip::GetTotalLength() const {
  float total = 0.0f;
  for (size_t i = 1; i < points_.size(); ++i) {
    total += glm::length(points_[i] - points_[i - 1]);
  }
  if (closed_ && points_.size() > 2) {
    total += glm::length(points_.front() - points_.back());
  }
  return total;
}

void LineStrip::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  try {
    // Compile and link line shader
    Shader line_vs(kLineVertexShader, Shader::Type::kVertex);
    Shader line_fs(kLineFragmentShader, Shader::Type::kFragment);
    if (!line_vs.Compile() || !line_fs.Compile()) {
      throw std::runtime_error("Line shader compilation failed");
    }
    line_shader_.AttachShader(line_vs);
    line_shader_.AttachShader(line_fs);
    if (!line_shader_.LinkProgram()) {
      throw std::runtime_error("Line shader linking failed");
    }

    // Compile and link point shader
    Shader point_vs(kPointVertexShader, Shader::Type::kVertex);
    Shader point_fs(kPointFragmentShader, Shader::Type::kFragment);
    if (!point_vs.Compile() || !point_fs.Compile()) {
      throw std::runtime_error("Point shader compilation failed");
    }
    point_shader_.AttachShader(point_vs);
    point_shader_.AttachShader(point_fs);
    if (!point_shader_.LinkProgram()) {
      throw std::runtime_error("Point shader linking failed");
    }

    // Compile and link arrow shader
    Shader arrow_vs(kArrowVertexShader, Shader::Type::kVertex);
    Shader arrow_fs(kArrowFragmentShader, Shader::Type::kFragment);
    if (!arrow_vs.Compile() || !arrow_fs.Compile()) {
      throw std::runtime_error("Arrow shader compilation failed");
    }
    arrow_shader_.AttachShader(arrow_vs);
    arrow_shader_.AttachShader(arrow_fs);
    if (!arrow_shader_.LinkProgram()) {
      throw std::runtime_error("Arrow shader linking failed");
    }

    // Create line VAO and VBOs
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vertex_vbo_);
    glGenBuffers(1, &color_vbo_);

    // Create arrow VAO and VBO if needed
    glGenVertexArrays(1, &arrow_vao_);
    glGenBuffers(1, &arrow_vbo_);

    UpdateGpuBuffers();
    
  } catch (const std::exception& e) {
    std::cerr << "LineStrip::AllocateGpuResources: " << e.what() << std::endl;
    ReleaseGpuResources();
    throw;
  }
}

void LineStrip::ReleaseGpuResources() noexcept {
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  if (vertex_vbo_ != 0) {
    glDeleteBuffers(1, &vertex_vbo_);
    vertex_vbo_ = 0;
  }
  if (color_vbo_ != 0) {
    glDeleteBuffers(1, &color_vbo_);
    color_vbo_ = 0;
  }
  if (arrow_vao_ != 0) {
    glDeleteVertexArrays(1, &arrow_vao_);
    arrow_vao_ = 0;
  }
  if (arrow_vbo_ != 0) {
    glDeleteBuffers(1, &arrow_vbo_);
    arrow_vbo_ = 0;
  }
}

void LineStrip::UpdateGpuBuffers() {
  if (!IsGpuResourcesAllocated() || points_.empty()) return;

  glBindVertexArray(vao_);

  // Update vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_);
  glBufferData(GL_ARRAY_BUFFER, points_.size() * sizeof(glm::vec3),
               points_.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
  glEnableVertexAttribArray(0);

  // Update color buffer if per-vertex colors are used
  if (use_per_vertex_colors_ && colors_.size() == points_.size()) {
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(glm::vec3),
                 colors_.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);
  }

  glBindVertexArray(0);
  needs_update_ = false;
}

void LineStrip::GenerateArrowGeometry() {
  if (!show_arrows_ || points_.size() < 2) return;

  arrow_positions_.clear();
  arrow_directions_.clear();

  float accumulated_distance = 0.0f;
  float next_arrow_distance = arrow_spacing_ * 0.5f;

  for (size_t i = 1; i < points_.size(); ++i) {
    glm::vec3 segment_start = points_[i - 1];
    glm::vec3 segment_end = points_[i];
    glm::vec3 segment_dir = segment_end - segment_start;
    float segment_length = glm::length(segment_dir);
    
    if (segment_length > 0) {
      segment_dir /= segment_length;  // Normalize

      float segment_start_distance = accumulated_distance;
      float segment_end_distance = accumulated_distance + segment_length;

      while (next_arrow_distance >= segment_start_distance && 
             next_arrow_distance <= segment_end_distance) {
        float t = (next_arrow_distance - segment_start_distance) / segment_length;
        glm::vec3 arrow_pos = segment_start + t * (segment_end - segment_start);
        arrow_positions_.push_back(arrow_pos);
        arrow_directions_.push_back(segment_dir);
        next_arrow_distance += arrow_spacing_;
      }

      accumulated_distance = segment_end_distance;
    }
  }

  needs_arrow_update_ = false;
}

void LineStrip::DrawLineStrip(const glm::mat4& mvp) {
  if (points_.empty()) return;

  line_shader_.Use();
  line_shader_.SetUniform("mvp", mvp);
  line_shader_.SetUniform("usePerVertexColor", use_per_vertex_colors_);
  line_shader_.SetUniform("uniformColor", uniform_color_);

  glBindVertexArray(vao_);
  glLineWidth(line_width_);

  // Note: OpenGL 3.3 core profile doesn't support line stipple
  // For dashed/dotted lines, we would need to generate geometry or use a shader technique
  // For now, we'll draw all lines as solid
  
  if (closed_ && points_.size() > 2) {
    glDrawArrays(GL_LINE_LOOP, 0, points_.size());
  } else {
    glDrawArrays(GL_LINE_STRIP, 0, points_.size());
  }

  glBindVertexArray(0);
}

void LineStrip::DrawPoints(const glm::mat4& mvp) {
  if (!show_points_ || points_.empty()) return;

  point_shader_.Use();
  point_shader_.SetUniform("mvp", mvp);
  point_shader_.SetUniform("pointColor", uniform_color_);
  point_shader_.SetUniform("pointSize", point_size_);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glBindVertexArray(vao_);
  glDrawArrays(GL_POINTS, 0, points_.size());
  glBindVertexArray(0);
  glDisable(GL_PROGRAM_POINT_SIZE);
}

void LineStrip::DrawArrows(const glm::mat4& mvp) {
  if (!show_arrows_ || arrow_positions_.empty()) return;

  arrow_shader_.Use();
  arrow_shader_.SetUniform("mvp", mvp);
  arrow_shader_.SetUniform("arrowColor", uniform_color_);

  std::vector<glm::vec3> arrow_vertices;
  
  for (size_t i = 0; i < arrow_positions_.size(); ++i) {
    glm::vec3 pos = arrow_positions_[i];
    glm::vec3 dir = arrow_directions_[i];
    
    // Create perpendicular vector
    glm::vec3 perp;
    if (std::abs(dir.y) < 0.9f) {
      perp = glm::normalize(glm::cross(dir, glm::vec3(0, 1, 0)));
    } else {
      perp = glm::normalize(glm::cross(dir, glm::vec3(1, 0, 0)));
    }
    
    // Create arrow head triangle
    glm::vec3 tip = pos + dir * arrow_size_;
    glm::vec3 base1 = pos - dir * arrow_size_ * 0.3f + perp * arrow_size_ * 0.3f;
    glm::vec3 base2 = pos - dir * arrow_size_ * 0.3f - perp * arrow_size_ * 0.3f;
    
    arrow_vertices.push_back(tip);
    arrow_vertices.push_back(base1);
    arrow_vertices.push_back(base2);
  }

  if (!arrow_vertices.empty()) {
    glBindVertexArray(arrow_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, arrow_vbo_);
    glBufferData(GL_ARRAY_BUFFER, arrow_vertices.size() * sizeof(glm::vec3),
                 arrow_vertices.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    
    glDrawArrays(GL_TRIANGLES, 0, arrow_vertices.size());
    glBindVertexArray(0);
  }
}

void LineStrip::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                       const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  if (needs_update_) {
    UpdateGpuBuffers();
  }

  if (needs_arrow_update_ && show_arrows_) {
    GenerateArrowGeometry();
  }

  glm::mat4 mvp = projection * view * coord_transform;

  // Draw line strip
  DrawLineStrip(mvp);

  // Draw points if enabled
  DrawPoints(mvp);

  // Draw arrows if enabled
  DrawArrows(mvp);
}

}  // namespace quickviz