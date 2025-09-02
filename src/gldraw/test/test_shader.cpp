/*
 * @file test_shader.cpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/window.hpp"
#include "gldraw/shader.hpp"
#include "gldraw/shader_program.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  Shader vertex_shader("../data/glsl/vertex_shader.glsl",
                       Shader::Type::kVertex);
  //  vertex_shader.Print();
  if (vertex_shader.Compile())
    std::cout << "Vertex shader compiled successfully" << std::endl;
  else
    std::cout << "Vertex shader compilation failed" << std::endl;

  Shader fragment_shader("../data/glsl/fragment_shader.glsl",
                         Shader::Type::kFragment);
  if (fragment_shader.Compile())
    std::cout << "Fragment shader compiled successfully" << std::endl;
  else
    std::cout << "Fragment shader compilation failed" << std::endl;

  ShaderProgram shader_program;
  shader_program.AttachShader(vertex_shader);
  shader_program.AttachShader(fragment_shader);
  if (shader_program.LinkProgram())
    std::cout << "Shader program linked successfully" << std::endl;
  else
    std::cout << "Shader program linking failed" << std::endl;

  /////////////////////////////////////////////////////////////////////////////

  // data in main memory
  float vertices[] = {
      0.0f,  0.5f,  0.0f,  // Top vertex
      -0.5f, -0.5f, 0.0f,  // Bottom-left vertex
      0.5f,  -0.5f, 0.0f   // Bottom-right vertex
  };

  // transfer data to GPU
  GLuint VBO, VAO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);
  {
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }
  glBindVertexArray(0);

  while (!win.ShouldClose()) {
    win.PollEvents();

    glViewport(0, 0, width, height);

    glClearColor(0.3, 0.3, 0.3, 0.5);
    glClear(GL_COLOR_BUFFER_BIT);

    shader_program.Use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    win.SwapBuffers();
  }

  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);

  return 0;
}