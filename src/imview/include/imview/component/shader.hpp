/*
 * @file shader.hpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SHADER_HPP
#define XMOTION_SHADER_HPP

#include <cstdint>
#include <string>
#include <unordered_map>

namespace quickviz {
class Shader {
 public:
  enum class Type : uint32_t {
    kUnknown = 0,
    kVertex,
    kFragment,
  };

 public:
  Shader(const std::string& source_file, Type type);
  ~Shader();

  // do not allow copy
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  // public methods
  void Print() const;
  bool Compile();
  uint32_t GetShaderID() const { return shader_id_; }

 private:
  std::string LoadSourceFile(const std::string& file_path);

  std::string source_file_;
  Type type_;
  std::string source_code_;
  uint32_t shader_id_;
};
}  // namespace quickviz

#endif  // XMOTION_SHADER_HPP