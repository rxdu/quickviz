/*
 * @file shader_program.hpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SHADER_PROGRAM_HPP
#define QUICKVIZ_SHADER_PROGRAM_HPP

#include <glm/glm.hpp>

#include <cstdint>
#include <stdexcept>
#include <unordered_map>

#include "renderer/shader.hpp"

namespace quickviz {
class ShaderProgram {
 public:
  ShaderProgram();
  ~ShaderProgram();

  // public methods
  void AttachShader(const Shader& shader);
  bool LinkProgram();
  void Use() const;

  // Uniform setting functions
  void SetUniform(const std::string& name, bool value);
  void SetUniform(const std::string& name, int value);
  void SetUniform(const std::string& name, float value);
  void SetUniform(const std::string& name, const glm::vec3& vector);
  void SetUniform(const std::string& name, const glm::vec4& vector);
  void SetUniform(const std::string& name, const glm::mat4& matrix);

  // Safe versions of SetUniform that don't throw if uniform doesn't exist
  bool TrySetUniform(const std::string& name, bool value) {
    try {
        SetUniform(name, value);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

  bool TrySetUniform(const std::string& name, int value) {
    try {
        SetUniform(name, value);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

  bool TrySetUniform(const std::string& name, float value) {
    try {
        SetUniform(name, value);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

  bool TrySetUniform(const std::string& name, const glm::vec3& vector) {
    try {
        SetUniform(name, vector);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

  bool TrySetUniform(const std::string& name, const glm::vec4& vector) {
    try {
        SetUniform(name, vector);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

  bool TrySetUniform(const std::string& name, const glm::mat4& matrix) {
    try {
        SetUniform(name, matrix);
        return true;
    } catch (const std::runtime_error& e) {
        return false;
    }
  }

 private:
  uint32_t GetUniformLocation(const std::string& name);

  uint32_t program_id_;
  std::unordered_map<std::string, uint32_t> uniform_location_cache_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_SHADER_PROGRAM_HPP