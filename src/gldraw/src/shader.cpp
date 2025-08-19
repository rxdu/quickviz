/*
 * @file shader.cpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "gldraw/shader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

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

Shader::Shader(const char* source_code, Type type)
    : source_code_(source_code), type_(type) {
  if (source_code == nullptr) {
    std::cout << "ERROR::SHADER::INVALID_SOURCE_CODE" << std::endl;
  }
  CreateShader();
}

Shader::Shader(const std::string& source_file, Shader::Type type)
    : source_file_(source_file), type_(type) {
  source_code_ = LoadShaderSource(source_file_);

  CreateShader();
}

Shader::~Shader() { glDeleteShader(shader_id_); }

void Shader::CreateShader() {
  if (source_code_.empty()) {
    std::cout << "ERROR::SHADER::INVALID_SOURCE_FILE" << std::endl;
    throw std::invalid_argument("Invalid shader source file");
  }

  if (type_ == Shader::Type::kVertex)
    shader_id_ = glCreateShader(GL_VERTEX_SHADER);
  else if (type_ == Shader::Type::kFragment)
    shader_id_ = glCreateShader(GL_FRAGMENT_SHADER);
  const char* code = source_code_.c_str();
  glShaderSource(shader_id_, 1, &code, NULL);
}

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
  GLchar infoLog[1024];
  glGetShaderiv(shader_id_, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader_id_, 1024, NULL, infoLog);
    
    // Provide more detailed error information
    std::string shader_type_str = (type_ == Type::kVertex) ? "VERTEX" : "FRAGMENT";
    std::cerr << "ERROR::SHADER::" << shader_type_str << "::COMPILATION_FAILED" << std::endl;
    std::cerr << "Shader source file: " << source_file_ << std::endl;
    std::cerr << "Compilation error: " << infoLog << std::endl;
    
    // Print the shader source code with line numbers for debugging
    std::cerr << "Shader source code:" << std::endl;
    std::cerr << "===================" << std::endl;
    std::istringstream iss(source_code_);
    std::string line;
    int line_number = 1;
    while (std::getline(iss, line)) {
      std::cerr << std::setfill('0') << std::setw(3) << line_number << ": " << line << std::endl;
      line_number++;
    }
    std::cerr << "===================" << std::endl;
    
    return false;
  }
  return success;
}
}  // namespace quickviz