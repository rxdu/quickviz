/*
 * @file utest_double_buffer.cpp
 * @date 10/10/24
 * @brief Unit tests for DoubleBuffer
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>

#include "core/buffer/double_buffer.hpp"

using namespace quickviz;

class DoubleBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor
  }

  void TearDown() override {
    // Code here will be called immediately after each test
  }

  // Objects declared here can be used by all tests
  DoubleBuffer<int> buffer;
};

TEST_F(DoubleBufferTest, IsEmptyInitially) {
  EXPECT_EQ(buffer.GetOccupiedSize(), 0);
}

TEST_F(DoubleBufferTest, CanWriteAndRead) {
  buffer.Write(42);
  EXPECT_EQ(buffer.GetOccupiedSize(), 1);

  int value;
  EXPECT_EQ(buffer.Read(value), 1);
  EXPECT_EQ(value, 42);
  EXPECT_EQ(buffer.GetOccupiedSize(), 0);
}

TEST_F(DoubleBufferTest, OverwritesBehavior) {
  // Write multiple values - only last one should be kept
  buffer.Write(1);
  buffer.Write(2);
  buffer.Write(3);
  
  EXPECT_EQ(buffer.GetOccupiedSize(), 1);
  
  int value;
  EXPECT_EQ(buffer.Read(value), 1);
  EXPECT_EQ(value, 3);  // Should get the last written value
}

TEST_F(DoubleBufferTest, PeekSingleElement) {
  // Test peek on empty buffer
  int value;
  EXPECT_EQ(buffer.Peek(value), 0);
  
  // Test peek after write
  buffer.Write(42);
  EXPECT_EQ(buffer.Peek(value), 1);
  EXPECT_EQ(value, 42);
  
  // Verify peek doesn't consume data
  EXPECT_EQ(buffer.GetOccupiedSize(), 1);
  
  // Verify we can still read the data
  int read_value;
  EXPECT_EQ(buffer.Read(read_value), 1);
  EXPECT_EQ(read_value, 42);
  EXPECT_EQ(buffer.GetOccupiedSize(), 0);
}

TEST_F(DoubleBufferTest, PeekMultipleWrites) {
  // Write multiple values
  buffer.Write(10);
  buffer.Write(20);
  buffer.Write(30);
  
  // Peek should return the most recent value
  int value;
  EXPECT_EQ(buffer.Peek(value), 1);
  EXPECT_EQ(value, 30);
  
  // Read should also get the most recent value
  int read_value;
  EXPECT_EQ(buffer.Read(read_value), 1);
  EXPECT_EQ(read_value, 30);
}

TEST_F(DoubleBufferTest, PeekAfterRead) {
  buffer.Write(100);
  
  int value;
  EXPECT_EQ(buffer.Read(value), 1);
  EXPECT_EQ(value, 100);
  
  // Peek after read should return 0
  EXPECT_EQ(buffer.Peek(value), 0);
}

TEST_F(DoubleBufferTest, ConcurrentWriteRead) {
  const int NUM_WRITES = 1000;
  std::atomic<int> last_written{0};
  
  // Writer thread
  std::thread writer([this, &last_written, NUM_WRITES]() {
    for (int i = 1; i <= NUM_WRITES; ++i) {
      buffer.Write(i);
      last_written = i;
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });
  
  // Reader thread
  std::thread reader([this, NUM_WRITES]() {
    int last_read = 0;
    int value;
    while (last_read < NUM_WRITES) {
      if (buffer.Read(value) == 1) {
        // Values should be monotonically increasing
        EXPECT_GT(value, last_read);
        last_read = value;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
  });
  
  writer.join();
  reader.join();
}

TEST_F(DoubleBufferTest, PeekThreadSafety) {
  const int NUM_OPERATIONS = 100;
  
  // Writer thread
  std::thread writer([this, NUM_OPERATIONS]() {
    for (int i = 1; i <= NUM_OPERATIONS; ++i) {
      buffer.Write(i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  
  // Peeker thread
  std::thread peeker([this, NUM_OPERATIONS]() {
    int value;
    int non_empty_peeks = 0;
    for (int i = 0; i < NUM_OPERATIONS * 2; ++i) {
      if (buffer.Peek(value) == 1) {
        // Value should be valid (between 1 and NUM_OPERATIONS)
        EXPECT_GE(value, 1);
        EXPECT_LE(value, NUM_OPERATIONS);
        non_empty_peeks++;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    // Should have seen at least some data
    EXPECT_GT(non_empty_peeks, 0);
  });
  
  writer.join();
  peeker.join();
}

TEST_F(DoubleBufferTest, TryReadBehavior) {
  // Test TryRead on empty buffer
  int value;
  EXPECT_FALSE(buffer.TryRead(value));
  
  // Test TryRead after write
  buffer.Write(42);
  EXPECT_TRUE(buffer.TryRead(value));
  EXPECT_EQ(value, 42);
  
  // Test TryRead after consuming data
  EXPECT_FALSE(buffer.TryRead(value));
}