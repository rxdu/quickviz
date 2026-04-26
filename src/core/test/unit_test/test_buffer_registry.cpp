/*
 * @file test_buffer_registry.cpp
 * @date 9/1/25
 * @brief Unit tests for BufferRegistry type safety improvements
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"
#include "core/buffer/double_buffer.hpp"

using namespace quickviz;

// Test data types
struct TestData {
  int value;
  std::string name;
  
  bool operator==(const TestData& other) const {
    return value == other.value && name == other.name;
  }
};

class BufferRegistryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Clear any existing buffers before each test
    auto& registry = BufferRegistry::GetInstance();
    auto names = registry.GetBufferNames();
    for (const auto& name : names) {
      registry.RemoveBuffer(name);
    }
  }
  
  void TearDown() override {
    // Clean up after each test
    SetUp();
  }
};

TEST_F(BufferRegistryTest, BasicAddAndRetrieve) {
  auto& registry = BufferRegistry::GetInstance();
  
  // Create and add a buffer
  auto int_buffer = std::make_shared<RingBuffer<int>>(10);
  registry.AddBuffer<int>("test_int", int_buffer);
  
  // Retrieve the buffer with correct type
  auto retrieved = registry.GetBuffer<int>("test_int");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved.value(), int_buffer);
}

TEST_F(BufferRegistryTest, TypeSafetyEnforcement) {
  auto& registry = BufferRegistry::GetInstance();
  
  // Add an int buffer
  auto int_buffer = std::make_shared<RingBuffer<int>>(10);
  registry.AddBuffer<int>("test_buffer", int_buffer);
  
  // Try to retrieve as wrong type - should return nullopt
  auto wrong_type = registry.GetBuffer<std::string>("test_buffer");
  EXPECT_FALSE(wrong_type.has_value());
  
  // Correct type should work
  auto correct_type = registry.GetBuffer<int>("test_buffer");
  EXPECT_TRUE(correct_type.has_value());
  EXPECT_EQ(correct_type.value(), int_buffer);
}

TEST_F(BufferRegistryTest, NonExistentBuffer) {
  auto& registry = BufferRegistry::GetInstance();
  
  // GetBuffer should return nullopt for nonexistent buffer
  auto result = registry.GetBuffer<int>("nonexistent");
  EXPECT_FALSE(result.has_value());
}

TEST_F(BufferRegistryTest, HasBufferCheck) {
  auto& registry = BufferRegistry::GetInstance();
  
  EXPECT_FALSE(registry.HasBuffer("test_buffer"));
  
  auto buffer = std::make_shared<RingBuffer<double>>(5);
  registry.AddBuffer<double>("test_buffer", buffer);
  
  EXPECT_TRUE(registry.HasBuffer("test_buffer"));
  
  registry.RemoveBuffer("test_buffer");
  EXPECT_FALSE(registry.HasBuffer("test_buffer"));
}

TEST_F(BufferRegistryTest, TypeNameDiagnostics) {
  auto& registry = BufferRegistry::GetInstance();
  
  // Add buffers of different types
  auto int_buffer = std::make_shared<RingBuffer<int>>(10);
  auto string_buffer = std::make_shared<DoubleBuffer<std::string>>();
  auto custom_buffer = std::make_shared<RingBuffer<TestData>>(5);
  
  registry.AddBuffer<int>("int_buf", int_buffer);
  registry.AddBuffer<std::string>("string_buf", string_buffer);
  registry.AddBuffer<TestData>("custom_buf", custom_buffer);
  
  // Check type names are stored correctly
  auto int_type = registry.GetBufferTypeName("int_buf");
  auto string_type = registry.GetBufferTypeName("string_buf");
  auto custom_type = registry.GetBufferTypeName("custom_buf");
  
  EXPECT_FALSE(int_type.empty());
  EXPECT_FALSE(string_type.empty());
  EXPECT_FALSE(custom_type.empty());
  
  // Type names should be different
  EXPECT_NE(int_type, string_type);
  EXPECT_NE(string_type, custom_type);
  
  // Non-existent buffer should return error message
  auto missing_type = registry.GetBufferTypeName("missing");
  EXPECT_EQ(missing_type, "<buffer not found>");
}

TEST_F(BufferRegistryTest, BufferEnumeration) {
  auto& registry = BufferRegistry::GetInstance();
  
  EXPECT_EQ(registry.GetBufferCount(), 0);
  EXPECT_TRUE(registry.GetBufferNames().empty());
  
  // Add some buffers
  auto buf1 = std::make_shared<RingBuffer<int>>(10);
  auto buf2 = std::make_shared<DoubleBuffer<double>>();
  auto buf3 = std::make_shared<RingBuffer<std::string>>(5);
  
  registry.AddBuffer<int>("buffer_c", buf1);      // Intentionally out of order
  registry.AddBuffer<double>("buffer_a", buf2);
  registry.AddBuffer<std::string>("buffer_b", buf3);
  
  EXPECT_EQ(registry.GetBufferCount(), 3);
  
  auto names = registry.GetBufferNames();
  EXPECT_EQ(names.size(), 3);
  
  // Names should be returned in sorted order
  std::vector<std::string> expected{"buffer_a", "buffer_b", "buffer_c"};
  EXPECT_EQ(names, expected);
}

TEST_F(BufferRegistryTest, SafeAPIBehavior) {
  auto& registry = BufferRegistry::GetInstance();
  
  // Add a buffer
  auto buffer = std::make_shared<RingBuffer<int>>(10);
  registry.AddBuffer<int>("safe_test", buffer);
  
  // GetBuffer should work for existing buffers
  auto retrieved = registry.GetBuffer<int>("safe_test");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved.value(), buffer);
  
  // GetBuffer should return nullopt on missing buffer
  auto missing = registry.GetBuffer<int>("missing");
  EXPECT_FALSE(missing.has_value());
  
  // GetBuffer should return nullopt on type mismatch
  auto wrong_type = registry.GetBuffer<std::string>("safe_test");
  EXPECT_FALSE(wrong_type.has_value());
  
  // Can still get diagnostic information about the buffer
  EXPECT_TRUE(registry.HasBuffer("safe_test"));
  EXPECT_FALSE(registry.GetBufferTypeName("safe_test").empty());
}

TEST_F(BufferRegistryTest, DuplicateNamePrevention) {
  auto& registry = BufferRegistry::GetInstance();
  
  auto buffer1 = std::make_shared<RingBuffer<int>>(10);
  auto buffer2 = std::make_shared<RingBuffer<int>>(20);
  
  registry.AddBuffer<int>("duplicate", buffer1);
  
  // Adding with same name should throw
  EXPECT_THROW(registry.AddBuffer<int>("duplicate", buffer2), std::runtime_error);
  
  // Original buffer should still be there
  auto retrieved = registry.GetBuffer<int>("duplicate");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved.value(), buffer1);
}

TEST_F(BufferRegistryTest, MultipleTypesCoexistence) {
  auto& registry = BufferRegistry::GetInstance();
  
  // Add buffers of different types with similar names
  auto int_buf = std::make_shared<RingBuffer<int>>(10);
  auto double_buf = std::make_shared<RingBuffer<double>>(10);
  auto string_buf = std::make_shared<RingBuffer<std::string>>(10);
  
  registry.AddBuffer<int>("data_int", int_buf);
  registry.AddBuffer<double>("data_double", double_buf);  
  registry.AddBuffer<std::string>("data_string", string_buf);
  
  // Each buffer should be retrievable with its correct type
  auto retrieved_int = registry.GetBuffer<int>("data_int");
  auto retrieved_double = registry.GetBuffer<double>("data_double");
  auto retrieved_string = registry.GetBuffer<std::string>("data_string");
  
  ASSERT_TRUE(retrieved_int.has_value());
  ASSERT_TRUE(retrieved_double.has_value());
  ASSERT_TRUE(retrieved_string.has_value());
  
  EXPECT_EQ(retrieved_int.value(), int_buf);
  EXPECT_EQ(retrieved_double.value(), double_buf);
  EXPECT_EQ(retrieved_string.value(), string_buf);
  
  // Cross-type access should fail
  EXPECT_FALSE(registry.GetBuffer<double>("data_int").has_value());
  EXPECT_FALSE(registry.GetBuffer<std::string>("data_double").has_value());
  EXPECT_FALSE(registry.GetBuffer<int>("data_string").has_value());
}

