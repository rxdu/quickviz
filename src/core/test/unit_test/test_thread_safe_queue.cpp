/*
 * @file test_thread_safe_queue.cpp
 * @date 9/1/25
 * @brief Unit tests for ThreadSafeQueue improvements
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>

#include "core/event/thread_safe_queue.hpp"

using namespace quickviz;

class ThreadSafeQueueTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ThreadSafeQueueTest, BasicPushPop) {
  ThreadSafeQueue<int> queue;
  
  queue.Push(42);
  queue.Push(84);
  
  EXPECT_FALSE(queue.Empty());
  EXPECT_EQ(queue.Size(), 2);
  
  auto value1 = queue.Pop();
  ASSERT_TRUE(value1.has_value());
  EXPECT_EQ(value1.value(), 42);
  
  auto value2 = queue.Pop();
  ASSERT_TRUE(value2.has_value());
  EXPECT_EQ(value2.value(), 84);
}

TEST_F(ThreadSafeQueueTest, TryPop) {
  ThreadSafeQueue<int> queue;
  
  int value;
  EXPECT_FALSE(queue.TryPop(value));
  
  queue.Push(100);
  EXPECT_TRUE(queue.TryPop(value));
  EXPECT_EQ(value, 100);
  
  EXPECT_FALSE(queue.TryPop(value));
}

TEST_F(ThreadSafeQueueTest, ShutdownProtocol) {
  ThreadSafeQueue<int> queue;
  
  EXPECT_FALSE(queue.IsClosed());
  
  // Start a thread that will block on Pop()
  std::atomic<bool> pop_returned{false};
  std::atomic<bool> pop_result_valid{false};
  
  std::thread consumer([&queue, &pop_returned, &pop_result_valid]() {
    auto result = queue.Pop();
    pop_returned = true;
    pop_result_valid = result.has_value();
  });
  
  // Give the consumer thread time to start blocking
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(pop_returned.load());
  
  // Close the queue - this should unblock the Pop()
  queue.Close();
  EXPECT_TRUE(queue.IsClosed());
  
  // Wait for consumer thread to finish
  consumer.join();
  
  // Pop should have returned with nullopt (no value)
  EXPECT_TRUE(pop_returned.load());
  EXPECT_FALSE(pop_result_valid.load());
}

TEST_F(ThreadSafeQueueTest, PushAfterClose) {
  ThreadSafeQueue<int> queue;
  
  queue.Close();
  
  // Push should throw after close
  EXPECT_THROW(queue.Push(42), std::runtime_error);
}

TEST_F(ThreadSafeQueueTest, PopTimeout) {
  ThreadSafeQueue<int> queue;
  
  auto start = std::chrono::steady_clock::now();
  auto result = queue.PopFor(std::chrono::milliseconds(50));
  auto duration = std::chrono::steady_clock::now() - start;
  
  EXPECT_FALSE(result.has_value());
  EXPECT_GE(duration, std::chrono::milliseconds(45)); // Allow some variance
  EXPECT_LE(duration, std::chrono::milliseconds(100));
}

TEST_F(ThreadSafeQueueTest, MoveConstructor) {
  ThreadSafeQueue<int> queue1;
  queue1.Push(1);
  queue1.Push(2);
  
  ThreadSafeQueue<int> queue2(std::move(queue1));
  
  auto value1 = queue2.Pop();
  auto value2 = queue2.Pop();
  
  ASSERT_TRUE(value1.has_value());
  ASSERT_TRUE(value2.has_value());
  EXPECT_EQ(value1.value(), 1);
  EXPECT_EQ(value2.value(), 2);
}

TEST_F(ThreadSafeQueueTest, MoveAssignment) {
  ThreadSafeQueue<int> queue1;
  ThreadSafeQueue<int> queue2;
  
  queue1.Push(10);
  queue1.Push(20);
  
  queue2 = std::move(queue1);
  
  auto value1 = queue2.Pop();
  auto value2 = queue2.Pop();
  
  ASSERT_TRUE(value1.has_value());
  ASSERT_TRUE(value2.has_value());
  EXPECT_EQ(value1.value(), 10);
  EXPECT_EQ(value2.value(), 20);
}

TEST_F(ThreadSafeQueueTest, PerfectForwarding) {
  ThreadSafeQueue<std::string> queue;
  
  std::string str = "moveable";
  queue.Push(std::move(str));
  
  // str should be moved (empty after move)
  EXPECT_TRUE(str.empty());
  
  auto result = queue.Pop();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "moveable");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}