/*
 * texture.hpp
 *
 * Created on 4/3/25 9:33 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"

namespace quickviz {
class Texture : public OpenGlObject {
 public:
  Texture();
  ~Texture();

  // public methods
  enum class PixelFormat { kGray, kRgb, kRgba, kBgr, kBgra };
  bool LoadData(const std::string& image_path);
  bool LoadData(int width, int height, PixelFormat format, unsigned char* data);
  void SetOrigin(const glm::vec3& origin, float resolution = 1.0f);

  uint32_t GetTextureId() const { return texture_id_; }

  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

 private:
  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;

  uint32_t texture_id_ = 0;

  glm::vec3 origin_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float resolution_ = 1.0f;

  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  ShaderProgram shader_;

  // Image dimensions in pixels
  int image_width_ = 0;
  int image_height_ = 0;
};
}  // namespace quickviz

#endif  // TEXTURE_HPP
