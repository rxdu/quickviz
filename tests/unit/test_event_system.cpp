/*
 * @file test_event_system.cpp
 * @date 2024-06-25
 * @brief Unit tests for event system components (LEGACY API - DISABLED)
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#if 0  // Disabled - uses legacy EventDispatcher singleton API

#include <gtest/gtest.h>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

#include "core/event/event.hpp"
#include "core/event/event_dispatcher.hpp"
#include "core/event/async_event_dispatcher.hpp"
#include "core/event/async_event_emitter.hpp"

using namespace quickviz;

// Test event type using the quickviz Event template
using TestEvent = Event<int, std::string>;

class EventSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        // EventDispatcher and AsyncEventDispatcher are singletons
        // Reset the async dispatcher state for clean tests
        AsyncEventDispatcher::GetInstance().Reset();
    }

    void TearDown() override {
        // Clean up after tests
        AsyncEventDispatcher::GetInstance().Reset();
    }
};

TEST_F(EventSystemTest, DISABLED_CanSubscribeAndPublish_LegacyAPI) {
    bool event_received = false;
    int received_value = 0;
    std::string received_message;
    
    // Subscribe to events
    EventDispatcher::GetInstance().RegisterHandler(
        "test_event",
        [&](std::shared_ptr<BaseEvent> event) {
            event_received = true;
            auto test_event = std::static_pointer_cast<TestEvent>(event);
            const auto& data = test_event->GetData();
            received_value = std::get<0>(data);
            received_message = std::get<1>(data);
        }
    );
    
    // Publish an event
    auto event = std::make_shared<TestEvent>(EventSource::kApplicaton, "test_event", 42, "test message");
    EventDispatcher::GetInstance().Dispatch(event);
    
    EXPECT_TRUE(event_received);
    EXPECT_EQ(received_value, 42);
    EXPECT_EQ(received_message, "test message");
}

TEST_F(EventSystemTest, DISABLED_MultipleSubscribers_LegacyAPI) {
    std::atomic<int> event_count{0};
    
    // Subscribe multiple handlers
    EventDispatcher::GetInstance().RegisterHandler(
        "multi_event",
        [&](std::shared_ptr<BaseEvent> event) {
            event_count++;
        }
    );
    
    EventDispatcher::GetInstance().RegisterHandler(
        "multi_event",
        [&](std::shared_ptr<BaseEvent> event) {
            event_count++;
        }
    );
    
    // Publish event
    auto event = std::make_shared<TestEvent>(EventSource::kApplicaton, "multi_event", 1, "multi");
    EventDispatcher::GetInstance().Dispatch(event);
    
    EXPECT_EQ(event_count.load(), 2);
}

// Note: Current API doesn't support unsubscribing, so we skip this test

TEST_F(EventSystemTest, DISABLED_AsyncEventHandling_LegacyAPI) {
    // Skip async tests due to strict thread ID enforcement in the implementation
    // These would require a more complex test setup with proper thread coordination
    GTEST_SKIP() << "AsyncEventDispatcher requires separate dispatch/handle thread setup";
}

TEST_F(EventSystemTest, DISABLED_EventQueueProcessing_LegacyAPI) {
    // Skip async tests due to strict thread ID enforcement in the implementation
    GTEST_SKIP() << "AsyncEventDispatcher requires separate dispatch/handle thread setup";
}

TEST_F(EventSystemTest, DISABLED_ThreadSafety_LegacyAPI) {
    // Skip this test as the async dispatcher enforces single-thread usage
    // This is by design for thread safety
    GTEST_SKIP() << "AsyncEventDispatcher enforces single-thread usage by design";
}
#endif  // Legacy API disabled
