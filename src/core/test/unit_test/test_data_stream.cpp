/*
 * @file test_data_stream.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "core/data_stream.hpp"

namespace quickviz {
namespace {

TEST(DataStreamTest, EmptyPullReturnsFalse) {
  DataStream<int> stream;
  int out = -1;
  EXPECT_FALSE(stream.TryPull(out));
  EXPECT_EQ(out, -1);  // out untouched on empty
}

TEST(DataStreamTest, OptionalApiReturnsNulloptWhenEmpty) {
  DataStream<int> stream;
  EXPECT_FALSE(stream.TryPull().has_value());
}

TEST(DataStreamTest, PushThenPullDeliversValue) {
  DataStream<int> stream;
  stream.Push(42);
  int out = 0;
  ASSERT_TRUE(stream.TryPull(out));
  EXPECT_EQ(out, 42);
}

TEST(DataStreamTest, PullReturnsFalseAfterFirstSuccessUntilNextPush) {
  DataStream<int> stream;
  stream.Push(7);
  int out = 0;
  ASSERT_TRUE(stream.TryPull(out));
  EXPECT_FALSE(stream.TryPull(out));  // already consumed
}

TEST(DataStreamTest, IntermediatePushesAreDropped) {
  // Latest-only semantics: if the producer pushes faster than the
  // consumer pulls, only the most recent value reaches the consumer.
  DataStream<int> stream;
  for (int i = 1; i <= 5; ++i) stream.Push(i);
  int out = 0;
  ASSERT_TRUE(stream.TryPull(out));
  EXPECT_EQ(out, 5);
  EXPECT_FALSE(stream.TryPull(out));
}

TEST(DataStreamTest, MoveOverloadCompiles) {
  DataStream<std::string> stream;
  std::string s = "hello";
  stream.Push(std::move(s));
  std::string out;
  ASSERT_TRUE(stream.TryPull(out));
  EXPECT_EQ(out, "hello");
}

TEST(DataStreamTest, ProducerConsumerThreadsConcurrent) {
  // Background producer pushes a counter; foreground "renderer" polls it
  // and verifies it always sees a non-decreasing snapshot. This is the
  // headline use case: render thread never blocks, never sees torn data.
  DataStream<int> stream;
  std::atomic<bool> stop{false};
  std::atomic<int> last_pulled{-1};

  std::thread producer([&]() {
    for (int i = 0; i < 1000 && !stop.load(); ++i) {
      stream.Push(i);
      std::this_thread::yield();
    }
  });

  // Pull-loop on the test thread (acts as render thread).
  int observed_max = -1;
  while (observed_max < 999) {
    int out = -1;
    if (stream.TryPull(out)) {
      ASSERT_GE(out, observed_max);  // never goes backwards
      observed_max = out;
    }
    if (last_pulled.load() == 999) break;
    if (out >= 0) last_pulled.store(out);
  }
  stop.store(true);
  producer.join();

  // Once producer is done, one final value should be available
  // (or already have been pulled). Either way, observed_max should
  // have reached the producer's last value at some point.
  EXPECT_GE(observed_max, 0);
  EXPECT_LE(observed_max, 999);
}

}  // namespace
}  // namespace quickviz
