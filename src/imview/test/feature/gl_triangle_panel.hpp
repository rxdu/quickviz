/*
 * @file gl_triangle_panel.hpp
 * @date 10/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_GLTRIANGLE_PANEL_HPP
#define QUICKVIZ_GLTRIANGLE_PANEL_HPP

#include <iostream>

#include "glad/glad.h"
#include "imview/interface/renderable.hpp"

namespace quickviz {
class GLTrianglePanel : public Renderable {
 public:
  GLTrianglePanel() {};

  bool IsVisible() const override { return true; }
  bool IsContainer() const override { return false; }
  void OnRender() override {
    PrepareData(300, 300);
    Render();
  }

 private:
  GLuint VBO, VAO;
  GLuint shaderProgram;

  // Vertex Shader source code
  const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
void main()
{
    gl_Position = vec4(aPos, 1.0);
}
)";

  // Fragment Shader source code
  const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
void main()
{
    FragColor = vec4(1.0, 0.5, 0.2, 1.0);
}
)";

  // Function to check for shader compile errors
  void CheckShaderCompile(GLuint shader) {
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(shader, 512, NULL, infoLog);
      std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n"
                << infoLog << std::endl;
    }
  }

  // Function to check for program linking errors
  void CheckProgramLink(GLuint program) {
    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(program, 512, NULL, infoLog);
      std::cerr << "ERROR::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }
  }

  void PrepareData(int width, int height) {
    // Define the viewport dimensions
    glViewport(0, 0, width, height);

    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    CheckShaderCompile(vertexShader);

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    CheckShaderCompile(fragmentShader);

    // Link shaders to create a shader program
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    CheckProgramLink(shaderProgram);

    // Delete the shaders as they're linked into our program now and no longer
    // necessary
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Define the vertices of a triangle
    float vertices[] = {
        0.0f,  0.5f,  0.0f,  // Top vertex
        -0.5f, -0.5f, 0.0f,  // Bottom-left vertex
        0.5f,  -0.5f, 0.0f   // Bottom-right vertex
    };

    // Create a Vertex Buffer Object (VBO)
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // Bind the Vertex Array Object first, then bind and set vertex buffers, and
    // then configure vertex attributes
    glBindVertexArray(VAO);

    // Bind the VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Configure the vertex attributes (position)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(0);

    // Unbind VBO and VAO (optional)
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  void Render() {
    // Rendering
    glClearColor(0, 1, 0, 0.5);
    glClear(GL_COLOR_BUFFER_BIT);

    // Use the shader program and bind the VAO
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);

    // Draw the triangle
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }

  void Cleanup() {
    // Clean up
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_GLTRIANGLE_PANEL_HPP
