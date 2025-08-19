/**
 * @file canvas_data_manager.hpp
 * @author Canvas Refactoring Phase 2.1
 * @date 2025-01-11
 * @brief Professional data management layer for Canvas
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_DATA_MANAGER_HPP
#define OPENGL_RENDERER_CANVAS_DATA_MANAGER_HPP

#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <glm/glm.hpp>

#include "gldraw/renderable/types.hpp"
#include "canvas_data.hpp"
#include "gldraw/renderable/details/canvas_batching.hpp"

namespace quickviz {
namespace internal {

/**
 * @brief Thread-safe data manager for Canvas operations
 * 
 * This class encapsulates all data management operations for the Canvas,
 * including pending updates, batch management, and thread-safe access to shape data.
 */
class CanvasDataManager {
public:
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
    struct ellipse_params {
      float x, y, rx, ry, angle, start_angle, end_angle;
    };

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

      ellipse_params ellipse;
    };

    // Polygon vertices (can't be in union)
    std::vector<glm::vec2> polygon_vertices;
  };

public:
  CanvasDataManager();
  ~CanvasDataManager() = default;

  // Thread-safe shape addition methods
  void AddPoint(float x, float y, const glm::vec4& color, float thickness = 1.0f);
  void AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
               float thickness = 1.0f, LineType line_type = LineType::kSolid);
  void AddRectangle(float x, float y, float width, float height,
                    const glm::vec4& color, bool filled = true,
                    float thickness = 1.0f, LineType line_type = LineType::kSolid);
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

  // Clear all data
  void Clear();

  // Batch management
  void SetBatchingEnabled(bool enabled);
  bool IsBatchingEnabled() const { return batching_enabled_; }
  
  // Process pending updates (transforms pending updates into actual data/batches)
  void ProcessPendingUpdates();
  
  // Check if there are pending updates
  bool HasPendingUpdates() const { return has_pending_updates_; }
  
  // Access to shape data (const access for rendering)
  const CanvasData& GetShapeData() const;
  
  // Access to batch data (const access for rendering)
  const std::unordered_map<LineType, LineBatch>& GetLineBatches() const;
  const ShapeBatch& GetFilledShapeBatch() const;
  const std::unordered_map<LineType, ShapeBatch>& GetOutlineShapeBatches() const;
  const BatchOrderTracker& GetBatchOrderTracker() const;
  
  // Memory management
  size_t GetMemoryUsage() const;
  void OptimizeMemory();
  void ShrinkToFit();
  void PreallocateMemory(size_t estimated_objects);

private:
  // Core data structures
  std::unique_ptr<CanvasData> data_;
  
  // Thread-safe pending updates
  mutable std::mutex data_mutex_;
  std::queue<PendingUpdate> pending_updates_;
  std::atomic<bool> has_pending_updates_{false};
  
  // Batching system
  std::atomic<bool> batching_enabled_{true};
  std::unordered_map<LineType, LineBatch> line_batches_;
  ShapeBatch filled_shape_batch_;
  std::unordered_map<LineType, ShapeBatch> outline_shape_batches_;
  BatchOrderTracker batch_order_tracker_;
  
  // Helper methods
  void InitializeBatches();
  void ClearBatches();
  void ProcessPendingUpdateBatched(const PendingUpdate& update);
  void ProcessPendingUpdateIndividual(const PendingUpdate& update);
  
  // Batch processing helpers
  void AddPointToBatch(const PendingUpdate& update);
  void AddLineToBatch(const PendingUpdate& update);
  void AddRectangleToBatch(const PendingUpdate& update);
  void AddCircleToBatch(const PendingUpdate& update);
  void AddEllipseToBatch(const PendingUpdate& update);
  void AddPolygonToBatch(const PendingUpdate& update);
};

} // namespace internal
} // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_DATA_MANAGER_HPP */