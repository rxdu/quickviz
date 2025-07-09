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
    GLchar infoLog[1024];
    glGetProgramInfoLog(program_id_, 1024, NULL, infoLog);
    
    std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED" << std::endl;
    std::cerr << "Program ID: " << program_id_ << std::endl;
    std::cerr << "Linking error: " << infoLog << std::endl;
    
    // Check if shaders were properly compiled before linking
    GLint num_shaders;
    glGetProgramiv(program_id_, GL_ATTACHED_SHADERS, &num_shaders);
    std::cerr << "Number of attached shaders: " << num_shaders << std::endl;
    
    if (num_shaders > 0) {
      GLuint shaders[10];
      GLsizei count;
      glGetAttachedShaders(program_id_, 10, &count, shaders);
      
      for (int i = 0; i < count; i++) {
        GLint shader_type;
        glGetShaderiv(shaders[i], GL_SHADER_TYPE, &shader_type);
        
        GLint compile_status;
        glGetShaderiv(shaders[i], GL_COMPILE_STATUS, &compile_status);
        
        std::string type_str = (shader_type == GL_VERTEX_SHADER) ? "VERTEX" : 
                              (shader_type == GL_FRAGMENT_SHADER) ? "FRAGMENT" : "UNKNOWN";
        
        std::cerr << "Shader " << i << " (ID: " << shaders[i] << ", Type: " << type_str 
                  << ") - Compiled: " << (compile_status ? "YES" : "NO") << std::endl;
        
        if (!compile_status) {
          GLchar shader_info[512];
          glGetShaderInfoLog(shaders[i], 512, NULL, shader_info);
          std::cerr << "Shader " << i << " compilation error: " << shader_info << std::endl;
        }
      }
    }
    
    // Additional troubleshooting information
    std::cerr << std::endl << "=== SHADER LINKING TROUBLESHOOTING ===" << std::endl;
    std::cerr << "Common causes of linking failures:" << std::endl;
    std::cerr << "1. Shader compilation failed (see above)" << std::endl;
    std::cerr << "2. Vertex shader output doesn't match fragment shader input" << std::endl;
    std::cerr << "3. Missing main() function in shader" << std::endl;
    std::cerr << "4. OpenGL version mismatch (#version directive)" << std::endl;
    std::cerr << "5. Hardware/driver doesn't support required features" << std::endl;
    std::cerr << "=========================================" << std::endl;
    
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