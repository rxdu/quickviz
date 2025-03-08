#include "imview/component/opengl/point_cloud.hpp"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <stdexcept>
#include <iostream>

#include "imview/component/opengl/shader.hpp"

namespace quickviz {

namespace {
// Simplified shader for debugging
const char* vertex_shader_source = R"(
    #version 330 core
    layout (location = 0) in vec3 aPosition;
    layout (location = 1) in vec3 aColor;
    
    uniform mat4 projection;
    uniform mat4 view;
    
    out vec3 vColor;
    
    void main() {
        gl_Position = projection * view * vec4(aPosition, 1.0);
        gl_PointSize = 20.0;  // Hardcoded point size for testing
        vColor = aColor;
    }
)";

const char* fragment_shader_source = R"(
    #version 330 core
    in vec3 vColor;
    
    out vec4 FragColor;
    
    void main() {
        FragColor = vec4(vColor, 1.0);  // Hardcoded full opacity for testing
    }
)";
}  // namespace

PointCloud::PointCloud() {
  try {
    // Create and compile shaders using the Shader class
    Shader vertexShader(vertex_shader_source, Shader::Type::kVertex);
    Shader fragmentShader(fragment_shader_source, Shader::Type::kFragment);

    // Print shader source for debugging
    std::cout << "Vertex Shader:" << std::endl;
    vertexShader.Print();
    std::cout << "Fragment Shader:" << std::endl;
    fragmentShader.Print();

    if (!vertexShader.Compile()) {
      throw std::runtime_error(
          "Failed to compile vertex shader for point cloud");
    }

    if (!fragmentShader.Compile()) {
      throw std::runtime_error(
          "Failed to compile fragment shader for point cloud");
    }

    // Create shader program
    shader_.AttachShader(vertexShader);
    shader_.AttachShader(fragmentShader);

    if (!shader_.LinkProgram()) {
      throw std::runtime_error("Failed to link point cloud shader program");
    }

    // Create VAO and VBOs
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &position_vbo_);
    glGenBuffers(1, &color_vbo_);

    glBindVertexArray(vao_);

    // Setup position VBO
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    // Setup color VBO
    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    std::cout << "Point cloud graphics resources initialized successfully"
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error initializing point cloud resources: " << e.what()
              << std::endl;
    throw;
  }
}

PointCloud::~PointCloud() {
  if (vao_) glDeleteVertexArrays(1, &vao_);
  if (position_vbo_) glDeleteBuffers(1, &position_vbo_);
  if (color_vbo_) glDeleteBuffers(1, &color_vbo_);

  vao_ = 0;
  position_vbo_ = 0;
  color_vbo_ = 0;
}

void PointCloud::SetPoints(const std::vector<glm::vec3>& points) {
  points_ = points;
  needs_update_ = true;

  // If no colors set, create default white colors
  if (colors_.size() != points_.size()) {
    colors_.resize(points_.size(), glm::vec3(1.0f));
  }

  std::cout << "Set " << points.size() << " points" << std::endl;
}

void PointCloud::SetColors(const std::vector<glm::vec3>& colors) {
  if (colors.size() != points_.size()) {
    throw std::runtime_error("Colors array size must match points array size");
  }
  colors_ = colors;
  needs_update_ = true;
}

void PointCloud::SetScalarField(const std::vector<float>& scalars,
                                float min_val, float max_val) {
  if (scalars.size() != points_.size()) {
    throw std::runtime_error("Scalar field size must match points array size");
  }

  colors_.resize(points_.size());
  for (size_t i = 0; i < scalars.size(); ++i) {
    float t = (scalars[i] - min_val) / (max_val - min_val);
    t = std::max(0.0f, std::min(1.0f, t));

    // Simple rainbow colormap
    colors_[i] =
        glm::vec3(std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.75f)),  // Red
                  std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.5f)),   // Green
                  std::max(0.0f, 2.0f - 4.0f * std::abs(t - 0.25f))   // Blue
        );
  }
  needs_update_ = true;

  std::cout << "Set scalar field with range [" << min_val << ", " << max_val
            << "]" << std::endl;
}

void PointCloud::OnDraw(const glm::mat4& projection, const glm::mat4& view) {
  if (points_.empty()) {
    std::cout << "No points to draw" << std::endl;
    return;
  }

  // Print the first few points for debugging
  std::cout << "First 3 points:" << std::endl;
  for (int i = 0; i < std::min(3, static_cast<int>(points_.size())); i++) {
    std::cout << "  Point " << i << ": " << points_[i].x << ", " << points_[i].y
              << ", " << points_[i].z << std::endl;
  }

  if (needs_update_) {
    // Update VBO data
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo_);
    glBufferData(GL_ARRAY_BUFFER, points_.size() * sizeof(glm::vec3),
                 points_.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
    glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(glm::vec3),
                 colors_.data(), GL_STATIC_DRAW);

    needs_update_ = false;
    std::cout << "Updated VBO data with " << points_.size() << " points"
              << std::endl;
  }

  shader_.Use();

  // Use TrySetUniform instead of SetUniform to avoid exceptions
  shader_.TrySetUniform("projection", projection);
  shader_.TrySetUniform("view", view);
  shader_.TrySetUniform("pointSize", point_size_);
  shader_.TrySetUniform("opacity", opacity_);

  // Draw points
  glBindVertexArray(vao_);

  // Enable point size - use the correct constant
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  // Make sure depth testing is properly configured
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  // Enable blending if needed
  if (opacity_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Force an extremely large point size
  glPointSize(50.0f);

  // Draw with GL_POINTS
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points_.size()));
  std::cout << "Drawing " << points_.size() << " points with size 50.0"
            << std::endl;

  if (opacity_ < 1.0f) {
    glDisable(GL_BLEND);
  }

  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glBindVertexArray(0);
}

}  // namespace quickviz
