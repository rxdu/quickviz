/*
 * @file test_async_event.cpp
 * @date 10/7/24
 * @brief Test for new instance-based AsyncEventDispatcher
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "core/event/event.hpp"
#include "core/event/async_event_dispatcher.hpp"
#include "core/event/async_event_emitter.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  std::cout << "=== AsyncEventDispatcher Test ===" << std::endl;
  
  // ====================================================================
  // Test 1: Basic event processing with automatic worker thread
  std::cout << "Test 1: Basic event processing" << std::endl;
  
  {
    AsyncEventDispatcher dispatcher; // Automatically starts worker thread
    AsyncEventEmitter emitter(dispatcher); // Instance-based emitter
    
    // Register handler that returns false (don't consume event)
    auto token1 = dispatcher.RegisterHandler("test_event", 
        [](std::shared_ptr<BaseEvent> event) -> bool {
          auto typed_event = std::static_pointer_cast<Event<int, double, std::string>>(event);
          const auto& data = typed_event->GetData();
          auto a = std::get<0>(data);
          auto b = std::get<1>(data);
          auto c = std::get<2>(data);
          std::cout << "Handler 1: Received event: a = " << a << ", b = " << b
                    << ", c = " << c << std::endl;
          return false; // Don't consume, let other handlers process
        });
    
    // Register second handler that consumes the event
    auto token2 = dispatcher.RegisterHandler("test_event",
        [](std::shared_ptr<BaseEvent> event) -> bool {
          auto typed_event = std::static_pointer_cast<Event<int, double, std::string>>(event);
          const auto& data = typed_event->GetData();
          auto a = std::get<0>(data);
          std::cout << "Handler 2: Consumed event with a = " << a << std::endl;
          return true; // Consume the event
        });
    
    std::cout << "Registered handlers. Token1: " << token1 << ", Token2: " << token2 << std::endl;
    std::cout << "Total handlers: " << dispatcher.GetHandlerCount() << std::endl;
    std::cout << "Handlers for 'test_event': " << dispatcher.GetHandlerCount("test_event") << std::endl;
    
    // Demonstrate both dispatch methods:
    // Method 1: Direct dispatch (when you already have an event object)
    auto event1 = std::make_shared<Event<int, double, std::string>>(
        EventSource::kApplicaton, "test_event", 42, 3.14, "hello");
    dispatcher.Dispatch(event1);
    
    // Method 2: Use emitter (more convenient, constructs event automatically)  
    emitter.Emit<Event<int, double, std::string>>(
        EventSource::kApplicaton, "test_event", 21, 6.28, "world");
    
    // Give worker thread time to process events
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::cout << "Queue size after processing: " << dispatcher.GetQueueSize() << std::endl;
    
    // Test handler unregistration
    dispatcher.UnregisterHandler(token2);
    std::cout << "After unregistering handler 2, total handlers: " << dispatcher.GetHandlerCount() << std::endl;
    
    // Use emitter to dispatch another event (only handler 1 should receive it)
    emitter.Emit<Event<int, double, std::string>>(
        EventSource::kApplicaton, "test_event", 99, 2.71, "after unregister");
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
  } // dispatcher destructor automatically stops worker thread and drains queue
  
  std::cout << "Dispatcher destroyed, worker thread stopped gracefully." << std::endl;
  std::cout << "-----------------------------" << std::endl << std::endl;

  // ====================================================================
  // Test 2: Multiple event types and handler management
  std::cout << "Test 2: Multiple event types and handler management" << std::endl;
  
  {
    AsyncEventDispatcher dispatcher;
    AsyncEventEmitter emitter(dispatcher);
    
    // Register handlers for different event types
    auto token_int = dispatcher.RegisterHandler("int_event",
        [](std::shared_ptr<BaseEvent> event) -> bool {
          auto int_event = std::static_pointer_cast<Event<int>>(event);
          std::cout << "Int handler: " << std::get<0>(int_event->GetData()) << std::endl;
          return false;
        });
        
    auto token_string = dispatcher.RegisterHandler("string_event",
        [](std::shared_ptr<BaseEvent> event) -> bool {
          auto string_event = std::static_pointer_cast<Event<std::string>>(event);
          std::cout << "String handler: " << std::get<0>(string_event->GetData()) << std::endl;
          return false;
        });
    
    // Use emitter to dispatch different event types  
    emitter.Emit<Event<int>>(EventSource::kApplicaton, "int_event", 123);
    emitter.Emit<Event<std::string>>(EventSource::kApplicaton, "string_event", std::string("test"));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Clear handlers for specific event type
    dispatcher.ClearHandlers("int_event");
    std::cout << "After clearing int_event handlers, total: " << dispatcher.GetHandlerCount() << std::endl;
    std::cout << "String event handlers remaining: " << dispatcher.GetHandlerCount("string_event") << std::endl;
    
    // Clear all handlers
    dispatcher.ClearHandlers();
    std::cout << "After clearing all handlers: " << dispatcher.GetHandlerCount() << std::endl;
  }
  
  std::cout << "-----------------------------" << std::endl << std::endl;

  // ====================================================================
  // Test 3: Exception handling in handlers
  std::cout << "Test 3: Exception handling in handlers" << std::endl;
  
  {
    AsyncEventDispatcher dispatcher;
    AsyncEventEmitter emitter(dispatcher);
    
    // Register handler that throws an exception
    [[maybe_unused]] auto token1 = dispatcher.RegisterHandler("exception_event",
        [](std::shared_ptr<BaseEvent> event) -> bool {
          std::cout << "Handler about to throw exception..." << std::endl;
          throw std::runtime_error("Test exception in handler");
          return false;
        });
    
    // Register handler that should still run after exception
    [[maybe_unused]] auto token2 = dispatcher.RegisterHandler("exception_event",
        [](std::shared_ptr<BaseEvent> event) -> bool {
          std::cout << "Handler running after exception handler" << std::endl;
          return false;
        });
    
    // Use emitter to trigger exception handling
    emitter.Emit<Event<int>>(EventSource::kApplicaton, "exception_event", 1);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  
  std::cout << "-----------------------------" << std::endl << std::endl;

  // ====================================================================
  // Test 4: Performance with many events
  std::cout << "Test 4: Performance test with many events" << std::endl;
  
  {
    AsyncEventDispatcher dispatcher;
    AsyncEventEmitter emitter(dispatcher);
    std::atomic<int> processed_count{0};
    
    [[maybe_unused]] auto token = dispatcher.RegisterHandler("perf_event",
        [&processed_count](std::shared_ptr<BaseEvent> event) -> bool {
          processed_count.fetch_add(1);
          return false;
        });
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Use emitter to dispatch 1000 events
    for (int i = 0; i < 1000; ++i) {
      emitter.Emit<Event<int>>(EventSource::kApplicaton, "perf_event", i);
    }
    
    // Wait for processing to complete
    while (processed_count.load() < 1000) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "Processed 1000 events in " << duration.count() << " microseconds" << std::endl;
    std::cout << "Average: " << (duration.count() / 1000.0) << " microseconds per event" << std::endl;
  }

  std::cout << "-----------------------------" << std::endl;
  std::cout << "All tests completed successfully!" << std::endl;
  
  return 0;
}
