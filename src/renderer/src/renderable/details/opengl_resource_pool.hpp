/**
 * @file opengl_resource_pool.hpp
 * @author Canvas Refactoring
 * @date 2025-01-11
 * @brief OpenGL resource pooling for efficient VAO/VBO management
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_OPENGL_RESOURCE_POOL_HPP
#define OPENGL_RENDERER_OPENGL_RESOURCE_POOL_HPP

#include <vector>
#include <mutex>
#include <memory>
#include "glad/glad.h"

namespace quickviz {
namespace internal {

/**
 * @brief Efficient OpenGL resource pool to avoid per-frame VAO/VBO allocation
 * 
 * This class manages a pool of temporary OpenGL resources (VAO/VBO pairs)
 * that can be reused across frames to improve performance by eliminating
 * the overhead of frequent glGenVertexArrays/glGenBuffers calls.
 */
class OpenGLResourcePool {
public:
  /**
   * @brief A pair of OpenGL resources for temporary rendering
   */
  struct TempResources {
    GLuint vao = 0;
    GLuint vbo = 0;
    
    TempResources() = default;
    TempResources(GLuint v, GLuint b) : vao(v), vbo(b) {}
    
    bool IsValid() const { return vao != 0 && vbo != 0; }
  };

public:
  /**
   * @brief Constructor - initializes the resource pool
   * @param initial_pool_size Initial number of resource pairs to create
   */
  explicit OpenGLResourcePool(size_t initial_pool_size = 8);
  
  /**
   * @brief Destructor - cleans up all OpenGL resources
   */
  ~OpenGLResourcePool();

  // Non-copyable, non-movable to prevent resource management issues
  OpenGLResourcePool(const OpenGLResourcePool&) = delete;
  OpenGLResourcePool& operator=(const OpenGLResourcePool&) = delete;
  OpenGLResourcePool(OpenGLResourcePool&&) = delete;
  OpenGLResourcePool& operator=(OpenGLResourcePool&&) = delete;

  /**
   * @brief Acquire a temporary resource pair from the pool
   * @return TempResources pair ready for use
   * 
   * This method is thread-safe and will create new resources if the pool is empty.
   */
  TempResources Acquire();

  /**
   * @brief Return a resource pair to the pool for reuse
   * @param resources The resources to return (must be valid)
   * 
   * This method is thread-safe. Resources should be returned promptly after use.
   */
  void Release(TempResources resources);

  /**
   * @brief Get current pool statistics
   */
  struct PoolStats {
    size_t available_resources = 0;
    size_t total_created = 0;
    size_t total_acquired = 0;
    size_t total_released = 0;
  };
  
  PoolStats GetStats() const;

  /**
   * @brief Pre-allocate additional resources in the pool
   * @param count Number of additional resource pairs to create
   */
  void PreallocatePool(size_t count);

  /**
   * @brief Clean up excess resources from the pool
   * @param max_keep_alive Maximum number of resources to keep in pool
   */
  void Cleanup(size_t max_keep_alive = 16);

private:
  mutable std::mutex pool_mutex_;
  std::vector<TempResources> available_resources_;
  
  // Statistics tracking
  mutable size_t total_created_ = 0;
  mutable size_t total_acquired_ = 0;
  mutable size_t total_released_ = 0;

  /**
   * @brief Create a new resource pair (internal method)
   * @return Newly created TempResources pair
   */
  TempResources CreateNewResource();

  /**
   * @brief Delete a resource pair (internal method)
   * @param resources The resources to delete
   */
  void DeleteResource(const TempResources& resources);
};

} // namespace internal
} // namespace quickviz

#endif /* OPENGL_RENDERER_OPENGL_RESOURCE_POOL_HPP */