/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "glad/glad.h"
#include "imview/window.hpp"
#include "imview/component/opengl/grid.hpp"
#include "imview/component/opengl/camera.hpp"

using namespace quickviz;

float deltaTime = 0.0f;  // Time between current frame and last frame
float lastFrame = 0.0f;  // Time of last frame

bool firstMouse = true;
float lastX = 1920 / 2.0f;
float lastY = 1080 / 2.0f;

Camera camera(glm::vec3(0.0f, 3.0f, 8.0f), -90.0f, -25.0f);

void ProcessInput(GLFWwindow* window) {
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    std::cout << "W key pressed" << std::endl;
    camera.ProcessKeyboard(Camera::Movement::kForward, deltaTime);
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    std::cout << "S key pressed" << std::endl;
    camera.ProcessKeyboard(Camera::Movement::kBackward, deltaTime);
  }
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    std::cout << "A key pressed" << std::endl;
    camera.ProcessKeyboard(Camera::Movement::kLeft, deltaTime);
  }
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    std::cout << "D key pressed" << std::endl;
    camera.ProcessKeyboard(Camera::Movement::kRight, deltaTime);
  }
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
    camera.Reset();
  }
}

void MouseCallback(GLFWwindow* window, double xpos, double ypos) {
  if (firstMouse) {
    lastX = xpos;
    lastY = ypos;
    firstMouse = false;
  }

  float xoffset = xpos - lastX;
  float yoffset =
      lastY - ypos;  // Reversed since y-coordinates go from bottom to top
  lastX = xpos;
  lastY = ypos;

  camera.ProcessMouseMovement(xoffset, yoffset);
}

void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  camera.ProcessMouseScroll(yoffset);
}

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  glfwSetCursorPosCallback(win.GetWindowObject(), MouseCallback);
  glfwSetScrollCallback(win.GetWindowObject(), ScrollCallback);
  glfwSetInputMode(win.GetWindowObject(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // set up grid
  Grid grid(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  grid.Initialize();

  while (!win.ShouldClose()) {
    win.PollEvents();

    // view
    ProcessInput(win.GetWindowObject());
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    glm::mat4 projection =
        camera.GetProjectionMatrix(static_cast<float>(width) / height);
    glm::mat4 view = camera.GetViewMatrix();

    // render
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    grid.Draw(projection, view);

    win.SwapBuffers();
  }

  return 0;
}
