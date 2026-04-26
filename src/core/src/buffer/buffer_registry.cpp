/*
 * @file buffer_registry.cpp
 * @date 10/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <stdexcept>
#include <algorithm>

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

bool BufferRegistry::HasBuffer(const std::string& buffer_name) const {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return buffers_.find(buffer_name) != buffers_.end();
}

std::string BufferRegistry::GetBufferTypeName(const std::string& buffer_name) const {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  auto it = buffers_.find(buffer_name);
  if (it == buffers_.end()) {
    return "<buffer not found>";
  }
  return it->second.type_name;
}

std::vector<std::string> BufferRegistry::GetBufferNames() const {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  std::vector<std::string> names;
  names.reserve(buffers_.size());
  
  for (const auto& [name, entry] : buffers_) {
    names.push_back(name);
  }
  
  std::sort(names.begin(), names.end());
  return names;
}

size_t BufferRegistry::GetBufferCount() const {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return buffers_.size();
}
}  // namespace quickviz