/*
 * @file shader.hpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SHADER_HPP
#define XMOTION_SHADER_HPP

#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

#include "glad/glad.h"

namespace quickviz {
class Shader {
 public:
  enum class Type {
    kUnknown = 0,
    kVertex,
    kFragment,
  };

 public:
  Shader(const std::string& source, Type type);
  ~Shader();

  // public methods
  void Compile();
  GLuint GetShaderID() const { return shader_id_; }

 private:
  std::string LoadSourceFile(const std::string& file_path);

  GLuint shader_id_;
  Type type_;
};
}  // namespace quickviz

#endif  // XMOTION_SHADER_HPP