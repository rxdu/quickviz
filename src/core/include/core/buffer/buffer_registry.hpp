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
#include <optional>
#include <typeindex>
#include <typeinfo>

#include "core/buffer/buffer_interface.hpp"

namespace quickviz {

// Helper structure to store buffer metadata for type safety
struct BufferEntry {
  std::shared_ptr<BufferBase> buffer;
  std::type_index type_info;
  std::string type_name;
  
  // Default constructor needed for std::unordered_map
  BufferEntry() : buffer(nullptr), type_info(typeid(void)), type_name("") {}
  
  BufferEntry(std::shared_ptr<BufferBase> buf, std::type_index type_idx, std::string type_str)
    : buffer(std::move(buf)), type_info(type_idx), type_name(std::move(type_str)) {}
};

class BufferRegistry {
 public:
  static BufferRegistry& GetInstance();

  // Add buffer with type tracking
  template <typename T>
  void AddBuffer(const std::string& buffer_name,
                 std::shared_ptr<BufferInterface<T>> buffer) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffers_.find(buffer_name) != buffers_.end()) {
      throw std::runtime_error("Buffer with the given name already exists.");
    }
    buffers_[buffer_name] = BufferEntry{
      buffer, 
      std::type_index(typeid(T)), 
      GetTypeName<T>()
    };
  }

  void RemoveBuffer(const std::string& buffer_name);
  
  // Diagnostic and utility methods
  [[nodiscard]] bool HasBuffer(const std::string& buffer_name) const;
  [[nodiscard]] std::string GetBufferTypeName(const std::string& buffer_name) const;
  [[nodiscard]] std::vector<std::string> GetBufferNames() const;
  [[nodiscard]] size_t GetBufferCount() const;

  // Type-safe buffer retrieval with optional return (default API)
  template <typename T>
  [[nodiscard]] std::optional<std::shared_ptr<BufferInterface<T>>> GetBuffer(
      const std::string& buffer_name) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto it = buffers_.find(buffer_name);
    if (it == buffers_.end()) {
      return std::nullopt;  // Buffer doesn't exist
    }
    
    const auto& entry = it->second;
    const std::type_index requested_type(typeid(T));
    
    if (entry.type_info != requested_type) {
      return std::nullopt;  // Type mismatch
    }
    
    // Safe cast - we verified the type matches
    auto buffer = std::static_pointer_cast<BufferInterface<T>>(entry.buffer);
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

  mutable std::mutex buffer_mutex_;
  std::unordered_map<std::string, BufferEntry> buffers_;
  
  // Helper method to get readable type names
  template<typename T>
  static std::string GetTypeName() {
    // Attempt to demangle the type name for better diagnostics
    const char* name = typeid(T).name();
    // Note: This is implementation-specific. On GCC/Clang, we could use
    // abi::__cxa_demangle, but for simplicity we'll use the raw name
    return std::string(name);
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_BUFFER_REGISTRY_HPP