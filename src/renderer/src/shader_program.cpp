/*
 * @file shader_program.cpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "renderer/shader_program.hpp"

#include <iostream>

#include "glad/glad.h"

namespace quickviz {
ShaderProgram::ShaderProgram() { program_id_ = glCreateProgram(); }

ShaderProgram::~ShaderProgram() { glDeleteProgram(program_id_); }

void ShaderProgram::AttachShader(const Shader& shader) {
  glAttachShader(program_id_, shader.GetShaderID());
}

bool ShaderProgram::LinkProgram() {
  glLinkProgram(program_id_);
  GLint success;
  glGetProgramiv(program_id_, GL_LINK_STATUS, &success);
  if (!success) {
    GLchar infoLog[512];
    glGetProgramInfoLog(program_id_, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
    return false;
  }
  return true;
}

void ShaderProgram::Use() const { glUseProgram(program_id_); }

void ShaderProgram::SetUniform(const std::string& name, bool value) {
  glUniform1i(GetUniformLocation(name), static_cast<int>(value));
}

void ShaderProgram::SetUniform(const std::string& name, int value) {
  glUniform1i(GetUniformLocation(name), value);
}

void ShaderProgram::SetUniform(const std::string& name, float value) {
  glUniform1f(GetUniformLocation(name), value);
}

void ShaderProgram::SetUniform(const std::string& name,
                               const glm::vec3& vector) {
  glUniform3fv(GetUniformLocation(name), 1, &vector[0]);
}

void ShaderProgram::SetUniform(const std::string& name,
                               const glm::vec4& vector) {
  glUniform4fv(GetUniformLocation(name), 1, &vector[0]);
}

void ShaderProgram::SetUniform(const std::string& name,
                               const glm::mat4& matrix) {
  glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, &matrix[0][0]);
}

uint32_t ShaderProgram::GetUniformLocation(const std::string& name) {
  // Use cache if location already retrieved
  if (uniform_location_cache_.find(name) != uniform_location_cache_.end())
    return uniform_location_cache_[name];

  // Query location and cache it
  GLint location = glGetUniformLocation(program_id_, name.c_str());
  if (location == -1) {
    throw std::runtime_error("Trying to access non-existent uniform: " + name);
  }
  uniform_location_cache_[name] = location;
  return location;
}
}  // namespace quickviz