/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <memory>

#include "glad/glad.h"
#include "imview/window.hpp"
#include "gldraw/frame_buffer.hpp"

using namespace quickviz;

// Vertex shader
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 TexCoord;

void main()
{
    gl_Position = vec4(aPos, 1.0);
    TexCoord = aTexCoord;
}
)";

// Fragment shader
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

in vec2 TexCoord;

uniform sampler2D texture1;

void main()
{
    FragColor = texture(texture1, TexCoord);
}
)";

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  // Create a framebuffer with multisampling
  auto frame_buffer = std::make_unique<FrameBuffer>(width, height, 4);

  // Set up vertex data for a quad that fills the screen
  float vertices[] = {
      // positions        // texture coords
      -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,  // bottom left
       1.0f, -1.0f, 0.0f, 1.0f, 0.0f,  // bottom right
       1.0f,  1.0f, 0.0f, 1.0f, 1.0f,  // top right
      -1.0f,  1.0f, 0.0f, 0.0f, 1.0f   // top left
  };
  
  unsigned int indices[] = {
      0, 1, 2,  // first triangle
      0, 2, 3   // second triangle
  };

  // Create and compile shaders
  unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
  glCompileShader(vertexShader);
  
  unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragmentShader);
  
  // Create shader program
  unsigned int shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);
  
  // Delete shaders as they're linked into the program now
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  // Create VAO, VBO, and EBO
  unsigned int VAO, VBO, EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);
  
  // Bind VAO first
  glBindVertexArray(VAO);
  
  // Bind and set VBO
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  
  // Bind and set EBO
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
  
  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  
  // Texture coord attribute
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  
  // Unbind VBO and VAO
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  while (!win.ShouldClose()) {
    win.PollEvents();

    // Bind and clear the framebuffer
    frame_buffer->Bind();
    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);  // Clear with red
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    frame_buffer->Unbind();

    // Render the framebuffer's texture directly to the screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Bind default framebuffer
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader program
    glUseProgram(shaderProgram);
    
    // Bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, frame_buffer->GetTextureId());
    
    // Draw quad
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    
    win.SwapBuffers();
  }

  // Clean up
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &EBO);
  glDeleteProgram(shaderProgram);

  return 0;
}
