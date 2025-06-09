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

// Batched rendering structures for improved performance
struct LineBatch {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec4> colors;
  std::vector<float> thicknesses;
  std::vector<LineType> line_types;
  uint32_t vao = 0;
  uint32_t position_vbo = 0;
  uint32_t color_vbo = 0;
  bool needs_update = true;
};

struct ShapeBatch {
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<glm::vec4> colors;
  uint32_t vao = 0;
  uint32_t vertex_vbo = 0;
  uint32_t color_vbo = 0;
  uint32_t ebo = 0;
  bool needs_update = true;
};

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

  // Performance and rendering methods
  void SetBatchingEnabled(bool enabled) { batching_enabled_ = enabled; }
  bool IsBatchingEnabled() const { return batching_enabled_; }
  void FlushBatches(); // Force immediate rendering of all batches
  
  // Performance monitoring
  struct RenderStats {
    uint32_t points_rendered = 0;
    uint32_t lines_rendered = 0;
    uint32_t shapes_rendered = 0;
    uint32_t draw_calls = 0;
    uint32_t state_changes = 0;
    float last_frame_time_ms = 0.0f;
    
    void Reset() {
      points_rendered = 0;
      lines_rendered = 0; 
      shapes_rendered = 0;
      draw_calls = 0;
      state_changes = 0;
      last_frame_time_ms = 0.0f;
    }
  };
  
  const RenderStats& GetRenderStats() const { return render_stats_; }
  void ResetRenderStats() { render_stats_.Reset(); }

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

  // Batching-related members
  bool batching_enabled_ = true;  // Re-enabled, ellipse/polygon renderMode fixed
  LineBatch line_batch_;
  ShapeBatch filled_shape_batch_;
  ShapeBatch outline_shape_batch_;

  // Batch management methods
  void InitializeBatches();
  void ClearBatches();
  void UpdateBatches();
  void RenderBatches(const glm::mat4& projection, const glm::mat4& view, 
                     const glm::mat4& coord_transform);
  
  // Individual shape rendering for non-batched shapes
  void RenderIndividualShapes(const CanvasData& data, const glm::mat4& projection, 
                             const glm::mat4& view, const glm::mat4& coord_transform);
  
  // Shape generation helpers
  void GenerateCircleVertices(float cx, float cy, float radius, int segments,
                             std::vector<float>& vertices, std::vector<uint32_t>& indices,
                             bool filled, uint32_t base_index);
  void GenerateRectangleVertices(float x, float y, float width, float height,
                                std::vector<float>& vertices, std::vector<uint32_t>& indices,
                                bool filled, uint32_t base_index);

  // Performance monitoring
  RenderStats render_stats_;
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
