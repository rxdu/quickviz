/*
 * @file shader.cpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/shader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

#include "glad/glad.h"

namespace quickviz {
namespace {
std::string LoadShaderSource(const std::string& file_path) {
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    file.open(file_path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    return buffer.str();
  } catch (std::ifstream::failure e) {
    std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ" << std::endl;
  }
  return "";
}
}  // namespace

Shader::Shader(const std::string& source_file, Shader::Type type)
    : source_file_(source_file), type_(type) {
  source_code_ = LoadShaderSource(source_file_);
  if (type_ == Shader::Type::kVertex)
    shader_id_ = glCreateShader(GL_VERTEX_SHADER);
  else if (type_ == Shader::Type::kFragment)
    shader_id_ = glCreateShader(GL_FRAGMENT_SHADER);
  const char* code = source_code_.c_str();
  glShaderSource(shader_id_, 1, &code, NULL);
}

Shader::~Shader() { glDeleteShader(shader_id_); }

void Shader::Print() const {
  std::cout << "Shader source file: " << source_file_ << std::endl;
  std::cout << "Shader type: " << static_cast<int>(type_) << std::endl;
  std::cout << "Shader source code: " << std::endl;
  std::cout << "------ start ------" << std::endl;
  std::cout << source_code_ << std::endl;
  std::cout << "------  end  ------" << std::endl;
}

bool Shader::Compile() {
  glCompileShader(shader_id_);

  GLint success;
  GLchar infoLog[512];
  glGetShaderiv(shader_id_, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader_id_, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
    return false;
  }
  return success;
}

std::string Shader::LoadSourceFile(const std::string& file_path) {
  return std::string();
}
}  // namespace quickviz