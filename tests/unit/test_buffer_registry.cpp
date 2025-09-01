/*
 * @file test_buffer_registry.cpp
 * @date 2024-06-25
 * @brief Unit tests for buffer registry components
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>

#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/double_buffer.hpp"

using namespace quickviz;

class BufferRegistryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // BufferRegistry is a singleton, no need to create instance
    }

    void TearDown() override {
        // Clean up any buffers that were added during tests
        try {
            BufferRegistry::GetInstance().RemoveBuffer("test_buffer");
        } catch (...) {
            // Ignore if buffer doesn't exist
        }
        try {
            BufferRegistry::GetInstance().RemoveBuffer("temp_buffer");
        } catch (...) {
            // Ignore if buffer doesn't exist
        }
    }
};

TEST_F(BufferRegistryTest, CanRegisterAndRetrieveBuffer) {
    // Create a test buffer
    auto buffer = std::make_shared<DoubleBuffer<int>>();
    
    // Register the buffer (cast to BufferInterface)
    BufferRegistry::GetInstance().AddBuffer<int>("test_buffer", buffer);
    
    // Retrieve the buffer
    auto retrieved_opt = BufferRegistry::GetInstance().GetBuffer<int>("test_buffer");
    ASSERT_TRUE(retrieved_opt.has_value());
    EXPECT_EQ(*retrieved_opt, buffer);
}

TEST_F(BufferRegistryTest, ReturnsNulloptForNonExistentBuffer) {
    // The API returns nullopt instead of throwing exceptions
    auto result = BufferRegistry::GetInstance().GetBuffer<int>("non_existent");
    EXPECT_FALSE(result.has_value());
}

TEST_F(BufferRegistryTest, CanRemoveBuffer) {
    auto buffer = std::make_shared<DoubleBuffer<std::string>>();
    
    // Register and verify
    BufferRegistry::GetInstance().AddBuffer<std::string>("temp_buffer", buffer);
    auto retrieved_opt = BufferRegistry::GetInstance().GetBuffer<std::string>("temp_buffer");
    ASSERT_TRUE(retrieved_opt.has_value());
    EXPECT_NE(*retrieved_opt, nullptr);
    
    // Remove and verify
    BufferRegistry::GetInstance().RemoveBuffer("temp_buffer");
    
    auto removed_opt = BufferRegistry::GetInstance().GetBuffer<std::string>("temp_buffer");
    EXPECT_FALSE(removed_opt.has_value());
}

TEST_F(BufferRegistryTest, ThrowsOnDuplicateRegistration) {
    auto buffer1 = std::make_shared<DoubleBuffer<int>>();
    auto buffer2 = std::make_shared<DoubleBuffer<int>>();
    
    // Register first buffer
    BufferRegistry::GetInstance().AddBuffer<int>("duplicate_test", buffer1);
    
    // Try to register with same name - should throw
    EXPECT_THROW(
        BufferRegistry::GetInstance().AddBuffer<int>("duplicate_test", buffer2),
        std::runtime_error
    );
    
    // Clean up
    BufferRegistry::GetInstance().RemoveBuffer("duplicate_test");
}

TEST_F(BufferRegistryTest, HandlesTypeMismatch) {
    auto int_buffer = std::make_shared<DoubleBuffer<int>>();
    BufferRegistry::GetInstance().AddBuffer<int>("type_test", int_buffer);
    
    // Try to retrieve with wrong type - should return nullopt
    auto wrong_type_opt = BufferRegistry::GetInstance().GetBuffer<std::string>("type_test");
    EXPECT_FALSE(wrong_type_opt.has_value());
    
    // Correct type should work
    auto correct_type_opt = BufferRegistry::GetInstance().GetBuffer<int>("type_test");
    ASSERT_TRUE(correct_type_opt.has_value());
    EXPECT_NE(*correct_type_opt, nullptr);
    
    // Clean up
    BufferRegistry::GetInstance().RemoveBuffer("type_test");
}

TEST_F(BufferRegistryTest, BufferFunctionality) {
    // Test that retrieved buffers actually work
    auto buffer = std::make_shared<DoubleBuffer<int>>();
    BufferRegistry::GetInstance().AddBuffer<int>("func_test", buffer);
    
    auto retrieved_opt = BufferRegistry::GetInstance().GetBuffer<int>("func_test");
    ASSERT_TRUE(retrieved_opt.has_value());
    
    auto retrieved = *retrieved_opt;
    
    // Test writing and reading
    retrieved->Write(42);
    EXPECT_EQ(retrieved->GetOccupiedSize(), 1);
    
    int value;
    retrieved->Read(value);
    EXPECT_EQ(value, 42);
    
    // Clean up
    BufferRegistry::GetInstance().RemoveBuffer("func_test");
}