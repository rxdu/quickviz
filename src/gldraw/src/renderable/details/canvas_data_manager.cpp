/**
 * @file canvas_data_manager.cpp  
 * @author Canvas Refactoring Phase 2.1
 * @date 2025-01-11
 * @brief Simplified working implementation of data management layer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "canvas_data_manager.hpp"
#include <algorithm>

namespace quickviz {
namespace internal {

//==============================================================================
// Constructor and Initialization
//==============================================================================

CanvasDataManager::CanvasDataManager()
    : data_(std::make_unique<CanvasData>()) {
  InitializeBatches();
}

//==============================================================================
// Public API - Thread-safe shape addition
//==============================================================================

void CanvasDataManager::AddPoint(float x, float y, const glm::vec4& color, float thickness) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPoint;
  update.color = color;
  update.thickness = thickness;
  update.point.x = x;
  update.point.y = y;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::AddLine(float x1, float y1, float x2, float y2,
                                const glm::vec4& color, float thickness, LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kLine;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.line.x1 = x1;
  update.line.y1 = y1;
  update.line.x2 = x2;
  update.line.y2 = y2;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::AddRectangle(float x, float y, float width, float height,
                                     const glm::vec4& color, bool filled, 
                                     float thickness, LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kRectangle;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.rect.x = x;
  update.rect.y = y;
  update.rect.width = width;
  update.rect.height = height;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::AddCircle(float x, float y, float radius, const glm::vec4& color,
                                  bool filled, float thickness, LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kCircle;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.circle.x = x;
  update.circle.y = y;
  update.circle.radius = radius;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::AddEllipse(float x, float y, float rx, float ry, float angle,
                                   float start_angle, float end_angle, const glm::vec4& color,
                                   bool filled, float thickness, LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kEllipse;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.ellipse.x = x;
  update.ellipse.y = y;
  update.ellipse.rx = rx;
  update.ellipse.ry = ry;
  update.ellipse.angle = angle;
  update.ellipse.start_angle = start_angle;
  update.ellipse.end_angle = end_angle;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::AddPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color,
                                   bool filled, float thickness, LineType line_type) {
  PendingUpdate update;
  update.type = PendingUpdate::Type::kPolygon;
  update.color = color;
  update.thickness = thickness;
  update.line_type = line_type;
  update.filled = filled;
  update.polygon_vertices = points;

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_updates_.push(std::move(update));
  has_pending_updates_ = true;
}

void CanvasDataManager::Clear() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_->Clear();
  ClearBatches();
}

//==============================================================================
// Batch Management
//==============================================================================

void CanvasDataManager::SetBatchingEnabled(bool enabled) {
  batching_enabled_ = enabled;
}

//==============================================================================
// Data Access (const methods for rendering)
//==============================================================================

const CanvasData& CanvasDataManager::GetShapeData() const {
  return *data_;
}

const std::unordered_map<LineType, LineBatch>& CanvasDataManager::GetLineBatches() const {
  return line_batches_;
}

const ShapeBatch& CanvasDataManager::GetFilledShapeBatch() const {
  return filled_shape_batch_;
}

const std::unordered_map<LineType, ShapeBatch>& CanvasDataManager::GetOutlineShapeBatches() const {
  return outline_shape_batches_;
}

const BatchOrderTracker& CanvasDataManager::GetBatchOrderTracker() const {
  return batch_order_tracker_;
}

//==============================================================================
// Update Processing - Simplified Implementation
//==============================================================================

void CanvasDataManager::ProcessPendingUpdates() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  while (!pending_updates_.empty()) {
    const auto& update = pending_updates_.front();
    
    // For simplicity, always use individual rendering mode
    // This ensures compatibility while maintaining the architecture
    ProcessPendingUpdateIndividual(update);
    
    pending_updates_.pop();
  }
  
  has_pending_updates_ = false;
}

//==============================================================================
// Memory Management - Simplified
//==============================================================================

size_t CanvasDataManager::GetMemoryUsage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  size_t total_usage = 0;
  
  // Add estimated size of CanvasData
  total_usage += data_->points.size() * sizeof(Point);
  total_usage += data_->lines.size() * sizeof(Line);
  total_usage += data_->rectangles.size() * sizeof(Rectangle);
  total_usage += data_->circles.size() * sizeof(Circle);
  total_usage += data_->ellipses.size() * sizeof(Ellipse);
  total_usage += data_->polygons.size() * sizeof(Polygon);
  
  // Add pending updates queue
  total_usage += pending_updates_.size() * sizeof(PendingUpdate);
  
  return total_usage;
}

void CanvasDataManager::OptimizeMemory() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Clear pending updates queue if empty
  if (pending_updates_.size() == 0) {
    std::queue<PendingUpdate> empty_queue;
    pending_updates_.swap(empty_queue);
  }
}

void CanvasDataManager::ShrinkToFit() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Shrink shape data vectors
  data_->points.shrink_to_fit();
  data_->lines.shrink_to_fit();
  data_->rectangles.shrink_to_fit();
  data_->circles.shrink_to_fit();
  data_->ellipses.shrink_to_fit();
  data_->polygons.shrink_to_fit();
}

void CanvasDataManager::PreallocateMemory(size_t estimated_objects) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Pre-allocate shape data vectors  
  data_->points.reserve(estimated_objects / 10);
  data_->lines.reserve(estimated_objects / 4);
  data_->rectangles.reserve(estimated_objects / 8);
  data_->circles.reserve(estimated_objects / 8);
  data_->ellipses.reserve(estimated_objects / 20);
  data_->polygons.reserve(estimated_objects / 20);
}

//==============================================================================
// Private Helper Methods - Simplified
//==============================================================================

void CanvasDataManager::InitializeBatches() {
  // Initialize empty batches
  line_batches_[LineType::kSolid] = LineBatch{};
  line_batches_[LineType::kDashed] = LineBatch{};
  line_batches_[LineType::kDotted] = LineBatch{};
  
  outline_shape_batches_[LineType::kSolid] = ShapeBatch{};
  outline_shape_batches_[LineType::kDashed] = ShapeBatch{};
  outline_shape_batches_[LineType::kDotted] = ShapeBatch{};
}

void CanvasDataManager::ClearBatches() {
  // Clear all batches - simplified
  for (auto& [line_type, batch] : line_batches_) {
    batch.vertices.clear();
    batch.colors.clear();
  }
  
  filled_shape_batch_.vertices.clear();
  filled_shape_batch_.colors.clear();
  
  for (auto& [line_type, batch] : outline_shape_batches_) {
    batch.vertices.clear();
    batch.colors.clear();
  }
  
  batch_order_tracker_.render_order.clear();
  batch_order_tracker_.next_sequence = 0;
}

void CanvasDataManager::ProcessPendingUpdateIndividual(const PendingUpdate& update) {
  switch (update.type) {
    case PendingUpdate::Type::kPoint:
      data_->AddPoint(update.point.x, update.point.y, update.color, update.thickness);
      break;
    case PendingUpdate::Type::kLine:
      data_->AddLine(update.line.x1, update.line.y1, update.line.x2,
                    update.line.y2, update.color, update.thickness,
                    update.line_type);
      break;
    case PendingUpdate::Type::kRectangle:
      data_->AddRectangle(update.rect.x, update.rect.y, update.rect.width,
                        update.rect.height, update.color, update.filled,
                        update.thickness, update.line_type);
      break;
    case PendingUpdate::Type::kCircle:
      data_->AddCircle(update.circle.x, update.circle.y, update.circle.radius,
                      update.color, update.filled, update.thickness,
                      update.line_type);
      break;
    case PendingUpdate::Type::kEllipse:
      data_->AddEllipse(update.ellipse.x, update.ellipse.y, update.ellipse.rx,
                       update.ellipse.ry, update.ellipse.angle,
                       update.ellipse.start_angle, update.ellipse.end_angle,
                       update.color, update.filled, update.thickness,
                       update.line_type);
      break;
    case PendingUpdate::Type::kPolygon:
      data_->AddPolygon(update.polygon_vertices, update.color, update.filled,
                       update.thickness, update.line_type);
      break;
    case PendingUpdate::Type::kClear:
      data_->Clear();
      ClearBatches();
      break;
  }
}

} // namespace internal
} // namespace quickviz