/*
 * @file buffer_registry.hpp
 * @date 10/11/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BUFFER_REGISTRY_HPP
#define QUICKVIZ_BUFFER_REGISTRY_HPP

#include <mutex>
#include <memory>
#include <unordered_map>
#include <stdexcept>

#include "core/buffer/buffer_interface.hpp"

namespace quickviz {
class BufferRegistry {
 public:
  static BufferRegistry& GetInstance();

  template <typename T>
  void AddBuffer(const std::string& buffer_name,
                 std::shared_ptr<BufferInterface<T>> buffer) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffers_.find(buffer_name) != buffers_.end()) {
      throw std::runtime_error("Buffer with the given name already exists.");
    }
    buffers_[buffer_name] = buffer;
  }

  void RemoveBuffer(const std::string& buffer_name);

  template <typename T>
  std::shared_ptr<BufferInterface<T>> GetBuffer(
      const std::string& buffer_name) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffers_.find(buffer_name) == buffers_.end()) {
      throw std::runtime_error("Buffer with the given name does not exist.");
    }
    auto buffer =
        std::dynamic_pointer_cast<BufferInterface<T>>(buffers_[buffer_name]);
    if (!buffer) {
      throw std::runtime_error("Buffer type mismatch.");
    }
    return buffer;
  }

 private:
  // private constructor and destructor
  BufferRegistry() = default;

  // do not allow copy or move
  BufferRegistry(const BufferRegistry&) = delete;
  BufferRegistry& operator=(const BufferRegistry&) = delete;
  BufferRegistry(BufferRegistry&&) = delete;
  BufferRegistry& operator=(BufferRegistry&&) = delete;

  std::mutex buffer_mutex_;
  std::unordered_map<std::string, std::shared_ptr<BufferBase>> buffers_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_BUFFER_REGISTRY_HPP