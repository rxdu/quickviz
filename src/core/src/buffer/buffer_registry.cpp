/*
 * @file buffer_registry.cpp
 * @date 10/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <stdexcept>

#include "core/buffer/buffer_registry.hpp"

namespace quickviz {
BufferRegistry& BufferRegistry::GetInstance() {
  static BufferRegistry instance;
  return instance;
}

void BufferRegistry::RemoveBuffer(const std::string& buffer_name) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (buffers_.find(buffer_name) == buffers_.end()) {
    throw std::runtime_error("Buffer with the given name does not exist.");
  }
  buffers_.erase(buffer_name);
}
}  // namespace quickviz