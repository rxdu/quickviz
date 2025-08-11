/**
 * @file opengl_resource_pool.cpp
 * @author Canvas Refactoring
 * @date 2025-01-11
 * @brief Implementation of OpenGL resource pooling system
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "opengl_resource_pool.hpp"
#include <iostream>
#include <algorithm>

namespace quickviz {
namespace internal {

OpenGLResourcePool::OpenGLResourcePool(size_t initial_pool_size) {
  // Pre-allocate initial pool
  PreallocatePool(initial_pool_size);
}

OpenGLResourcePool::~OpenGLResourcePool() {
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  // Clean up all remaining resources
  for (const auto& resource : available_resources_) {
    DeleteResource(resource);
  }
  available_resources_.clear();
}

OpenGLResourcePool::TempResources OpenGLResourcePool::Acquire() {
  std::lock_guard<std::mutex> lock(pool_mutex_);
  total_acquired_++;
  
  if (!available_resources_.empty()) {
    // Reuse existing resource from pool
    auto resource = available_resources_.back();
    available_resources_.pop_back();
    return resource;
  }
  
  // Pool is empty, create new resource
  return CreateNewResource();
}

void OpenGLResourcePool::Release(TempResources resources) {
  if (!resources.IsValid()) {
    std::cerr << "Warning: Attempting to release invalid OpenGL resources" << std::endl;
    return;
  }
  
  std::lock_guard<std::mutex> lock(pool_mutex_);
  total_released_++;
  
  // Return resource to pool for reuse
  available_resources_.push_back(resources);
}

OpenGLResourcePool::PoolStats OpenGLResourcePool::GetStats() const {
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  PoolStats stats;
  stats.available_resources = available_resources_.size();
  stats.total_created = total_created_;
  stats.total_acquired = total_acquired_;
  stats.total_released = total_released_;
  
  return stats;
}

void OpenGLResourcePool::PreallocatePool(size_t count) {
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  // Reserve space to avoid reallocations
  available_resources_.reserve(available_resources_.size() + count);
  
  // Create requested number of resources
  for (size_t i = 0; i < count; ++i) {
    auto resource = CreateNewResource();
    if (resource.IsValid()) {
      available_resources_.push_back(resource);
    }
  }
}

void OpenGLResourcePool::Cleanup(size_t max_keep_alive) {
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  if (available_resources_.size() <= max_keep_alive) {
    return; // Nothing to clean up
  }
  
  // Delete excess resources
  size_t to_delete = available_resources_.size() - max_keep_alive;
  
  for (size_t i = 0; i < to_delete; ++i) {
    DeleteResource(available_resources_.back());
    available_resources_.pop_back();
  }
  
  // Shrink vector to fit
  available_resources_.shrink_to_fit();
}

OpenGLResourcePool::TempResources OpenGLResourcePool::CreateNewResource() {
  // This method assumes we're in a valid OpenGL context
  GLuint vao, vbo;
  
  // Generate vertex array object
  glGenVertexArrays(1, &vao);
  if (vao == 0) {
    std::cerr << "Error: Failed to generate VAO in resource pool" << std::endl;
    return TempResources();
  }
  
  // Generate vertex buffer object
  glGenBuffers(1, &vbo);
  if (vbo == 0) {
    std::cerr << "Error: Failed to generate VBO in resource pool" << std::endl;
    glDeleteVertexArrays(1, &vao);
    return TempResources();
  }
  
  total_created_++;
  return TempResources(vao, vbo);
}

void OpenGLResourcePool::DeleteResource(const TempResources& resources) {
  if (!resources.IsValid()) {
    return;
  }
  
  // Clean up OpenGL resources
  glDeleteVertexArrays(1, &resources.vao);
  glDeleteBuffers(1, &resources.vbo);
}

} // namespace internal
} // namespace quickviz