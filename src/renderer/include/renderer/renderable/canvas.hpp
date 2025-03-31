/**
 * @file canvas.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_HPP
#define OPENGL_RENDERER_CANVAS_HPP

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>

#include <glm/glm.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"
#include "renderer/renderable/types.hpp"

namespace quickviz {
// Forward declaration of Point struct
struct Point;
struct CanvasData;

class Canvas : public OpenGlObject {
 public:
  Canvas();
  ~Canvas();

  // public methods
  void AddPoint(float x, float y, const glm::vec4& color,
                float thickness = 1.0f);
  void AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
               float thickness = 1.0f, LineType line_type = LineType::kSolid);
  void AddRectangle(float x, float y, float width, float height,
                    const glm::vec4& color, bool filled = true,
                    float thickness = 1.0f,
                    LineType line_type = LineType::kSolid);
  void AddCircle(float x, float y, float radius, const glm::vec4& color,
                 bool filled = true, float thickness = 1.0f,
                 LineType line_type = LineType::kSolid);
  void AddEllipse(float x, float y, float rx, float ry, float angle,
                  float start_angle, float end_angle, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::kSolid);
  void AddPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::kSolid);

  void AddBackgroundImage(const std::string& image_path,
                          const glm::vec3& origin, float resolution);

  // Clear all points from the canvas
  void Clear();

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

 private:
  // Load and setup background image
  void SetupBackgroundImage(int width, int height, int channels,
                            unsigned char* data);

  // Process pending updates
  void ProcessPendingUpdates();

  // Structure to hold pending updates
  struct PendingUpdate {
    enum class Type {
      kPoint,
      kLine,
      kRectangle,
      kCircle,
      kEllipse,
      kPolygon,
      kClear
    };

    Type type;
    glm::vec4 color;
    float thickness;
    LineType line_type;
    bool filled;
    
    // Command-specific parameters
    union {
      struct {  // Point parameters
        float x, y;
      } point;
      
      struct {  // Line parameters
        float x1, y1, x2, y2;
      } line;
      
      struct {  // Rectangle parameters
        float x, y, width, height;
      } rect;
      
      struct {  // Circle parameters
        float x, y, radius;
      } circle;
      
      struct {  // Ellipse parameters
        float x, y, rx, ry, angle, start_angle, end_angle;
      } ellipse;
    };
    
    // Polygon vertices (can't be in union)
    std::vector<glm::vec2> polygon_vertices;
  };

  // Background image texture
  std::mutex background_mutex_;
  std::atomic<uint32_t> background_texture_{0};

  // Background rendering gpu resources
  uint32_t background_vao_ = 0;
  uint32_t background_vbo_ = 0;
  ShaderProgram background_shader_;

  // Thread-safe data structures
  std::mutex data_mutex_;
  std::queue<PendingUpdate> pending_updates_;
  std::atomic<bool> has_pending_updates_{false};
  std::unique_ptr<CanvasData> data_;

  // Primitive rendering gpu resources
  uint32_t primitive_vao_ = 0;
  uint32_t primitive_vbo_ = 0;
  ShaderProgram primitive_shader_;
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
