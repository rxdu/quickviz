/*
 * @file buffer_interface.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_BUFFER_INTERFACE_HPP
#define QUICKVIZ_BUFFER_INTERFACE_HPP

#include <cstddef>
#include <cassert>
#include <vector>

namespace quickviz {
class BufferBase {
 public:
  virtual ~BufferBase() = default;
};

template <typename T>
class BufferInterface : public BufferBase {
 public:
  /// \brief Get the number of bytes that can be read from the buffer
  /// \return number of bytes that can be read
  virtual std::size_t GetOccupiedSize() const = 0;

  /// \brief Read data from the buffer
  /// \param data
  /// \return 0 if failed, otherwise the number of bytes read (1)
  virtual std::size_t Read(T& data) = 0;

  /// \brief Write data to the buffer
  /// \param data
  /// \return 0 if failed, otherwise the number of bytes written (1)
  virtual std::size_t Write(const T& data) = 0;

  /// \brief Peek at the most recent data without consuming it
  /// \param data reference to store the peeked data
  /// \return 0 if failed, otherwise 1
  virtual std::size_t Peek(T& data) const = 0;

  /// \brief Read data from the buffer (burst read)
  /// \param data
  /// \return 0 if failed, otherwise the number of bytes read
  virtual std::size_t Read(std::vector<T>& data, std::size_t size) {
    assert(data.size() >= size);
    std::size_t bytes_read = 0;
    for (std::size_t i = 0; i < size; ++i) {
      if (Read(data[i]) == 0) {
        return bytes_read;
      }
      bytes_read++;
    }
    return bytes_read;
  }

  /// \brief Write data to the buffer (burst write)
  /// \param data
  /// \return 0 if failed, otherwise the number of bytes written
  virtual std::size_t Write(const std::vector<T>& data, std::size_t size) {
    assert(data.size() >= size);
    std::size_t bytes_written = 0;
    for (std::size_t i = 0; i < size; ++i) {
      if (Write(data[i]) == 0) {
        return bytes_written;
      }
      bytes_written++;
    }
    return bytes_written;
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_BUFFER_INTERFACE_HPP
