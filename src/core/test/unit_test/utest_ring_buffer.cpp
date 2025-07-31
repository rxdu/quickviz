/*
 * @file utest_ring_buffer.cpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include "core/buffer/ring_buffer.hpp"

using namespace quickviz;

class RingBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right before
    // each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right before the
    // destructor).
  }

  // Objects declared here can be used by all tests in the test case.
  RingBuffer<int, 8> buffer_no_overwrite{false};
  RingBuffer<int, 8> buffer_with_overwrite{true};
};

TEST_F(RingBufferTest, IsEmptyInitially) {
  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
}

TEST_F(RingBufferTest, IsNotFullInitially) {
  EXPECT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_FALSE(buffer_with_overwrite.IsFull());
}

TEST_F(RingBufferTest, CanWriteAndRead) {
  buffer_no_overwrite.Write(1);
  buffer_with_overwrite.Write(2);

  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 1);

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 2);

  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
}

TEST_F(RingBufferTest, PeekData) {
  buffer_no_overwrite.Reset();
  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  buffer_no_overwrite.Write(1);
  buffer_no_overwrite.Write(3);
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 2);

  std::vector<int> value(7);
  buffer_no_overwrite.Peek(value, 1);
  EXPECT_EQ(value[0], 1);
  buffer_no_overwrite.Peek(value, 2);
  EXPECT_EQ(value[0], 1);
  EXPECT_EQ(value[1], 3);
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 2);

  // Test with overwrite enabled
  buffer_with_overwrite.Reset();
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
  buffer_with_overwrite.Write(1);
  buffer_with_overwrite.Write(3);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 2);

  buffer_with_overwrite.Peek(value, 1);
  EXPECT_EQ(value[0], 1);
  buffer_with_overwrite.Peek(value, 2);
  EXPECT_EQ(value[0], 1);
  EXPECT_EQ(value[1], 3);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 2);
}

TEST_F(RingBufferTest, ReadWriteMultiple) {
  std::vector<int> input = {1, 2, 3, 4, 5, 6};
  std::vector<int> output(6);

  buffer_no_overwrite.Reset();
  buffer_no_overwrite.Write(input, 6);
  buffer_no_overwrite.Read(output, 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(input[i], output[i]);
  }

  buffer_with_overwrite.Reset();
  buffer_with_overwrite.Write(input, 6);
  buffer_with_overwrite.Read(output, 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(input[i], output[i]);
  }
}

TEST_F(RingBufferTest, MaintainsOrder) {
  buffer_no_overwrite.Write(1);
  buffer_no_overwrite.Write(2);
  buffer_no_overwrite.Write(3);

  buffer_with_overwrite.Write(4);
  buffer_with_overwrite.Write(5);
  buffer_with_overwrite.Write(6);

  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 1);
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 2);
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 3);

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 4);
  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 5);
  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 6);
}

TEST_F(RingBufferTest, OverwritesWhenFull) {
  buffer_no_overwrite.Reset();
  buffer_with_overwrite.Reset();

  // the buffer can only hold 8 - 1 = 7 elements
  for (int i = 0; i < 7; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_with_overwrite.Write(i);
  }
  ASSERT_TRUE(buffer_no_overwrite.IsFull());
  ASSERT_TRUE(buffer_with_overwrite.IsFull());

  buffer_no_overwrite.Write(8);  // This should not overwrite the first element
  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 0);
  ASSERT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 6);
  EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 1);

  buffer_with_overwrite.Write(8);  // This should overwrite the first element
  ASSERT_TRUE(buffer_with_overwrite.IsFull());

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 1);  // The first element (0) should be overwritten
  ASSERT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 6);
  EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 1);
}

TEST_F(RingBufferTest, HandlesWrapAround) {
  buffer_no_overwrite.Reset();
  int value;
  for (int i = 0; i < 15; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_no_overwrite.Read(value);
    EXPECT_EQ(value, i);
    EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 7);
  }

  buffer_with_overwrite.Reset();
  for (int i = 0; i < 15; ++i) {
    buffer_with_overwrite.Write(i);
    buffer_with_overwrite.Read(value);
    EXPECT_EQ(value, i);
    EXPECT_EQ(buffer_with_overwrite.GetFreeSize(), 7);
  }
}

TEST_F(RingBufferTest, PeekSingleElement) {
  buffer_no_overwrite.Reset();
  buffer_with_overwrite.Reset();
  
  // Test peek on empty buffer
  int value;
  EXPECT_EQ(buffer_no_overwrite.Peek(value), 0);
  EXPECT_EQ(buffer_with_overwrite.Peek(value), 0);
  
  // Test peek after single write
  buffer_no_overwrite.Write(42);
  buffer_with_overwrite.Write(42);
  
  EXPECT_EQ(buffer_no_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 42);
  EXPECT_EQ(buffer_with_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 42);
  
  // Verify peek doesn't consume data
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 1);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 1);
}

TEST_F(RingBufferTest, PeekMultipleElements) {
  buffer_no_overwrite.Reset();
  buffer_with_overwrite.Reset();
  
  // Write multiple elements
  for (int i = 1; i <= 5; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_with_overwrite.Write(i);
  }
  
  // Peek should return the most recent element
  int value;
  EXPECT_EQ(buffer_no_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 5);
  EXPECT_EQ(buffer_with_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 5);
  
  // Verify all data is still there
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 5);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 5);
}

TEST_F(RingBufferTest, PeekAfterWraparound) {
  buffer_no_overwrite.Reset();
  buffer_with_overwrite.Reset();
  
  // Fill buffer completely (7 elements max)
  for (int i = 0; i < 7; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_with_overwrite.Write(i);
  }
  
  // Read some elements to create space
  int value;
  for (int i = 0; i < 3; ++i) {
    buffer_no_overwrite.Read(value);
    buffer_with_overwrite.Read(value);
  }
  
  // Write more to cause wraparound
  for (int i = 10; i < 13; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_with_overwrite.Write(i);
  }
  
  // Peek should return the most recent (12)
  EXPECT_EQ(buffer_no_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 12);
  EXPECT_EQ(buffer_with_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 12);
}

TEST_F(RingBufferTest, PeekWithOverwrite) {
  buffer_with_overwrite.Reset();
  
  // Fill buffer completely
  for (int i = 0; i < 7; ++i) {
    buffer_with_overwrite.Write(i);
  }
  
  // Write one more to trigger overwrite
  buffer_with_overwrite.Write(99);
  
  // Peek should return the most recent
  int value;
  EXPECT_EQ(buffer_with_overwrite.Peek(value), 1);
  EXPECT_EQ(value, 99);
  
  // Read all values to verify order
  std::vector<int> read_values;
  while (buffer_with_overwrite.Read(value) == 1) {
    read_values.push_back(value);
  }
  
  // Should have values 1-6 and 99 (0 was overwritten)
  EXPECT_EQ(read_values.size(), 7);
  EXPECT_EQ(read_values[0], 1);
  EXPECT_EQ(read_values[6], 99);
}

TEST_F(RingBufferTest, PeekEdgeCase_WriteIndexZero) {
  // This test specifically checks the edge case where write_index_ is 0
  RingBuffer<int, 4> small_buffer;
  
  // Write exactly enough to wrap write_index_ back to 0
  for (int i = 0; i < 8; ++i) {
    small_buffer.Write(i);
    int dummy;
    small_buffer.Read(dummy);
  }
  
  // Now write_index_ should be 0
  small_buffer.Write(123);
  
  int value;
  EXPECT_EQ(small_buffer.Peek(value), 1);
  EXPECT_EQ(value, 123);
}
