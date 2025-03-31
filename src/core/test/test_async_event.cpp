/*
 * @file test_event.cpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "core/event/event.hpp"
#include "core/event/async_event_dispatcher.hpp"
#include "core/event/async_event_emitter.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Event<int, double, std::string> event(EventSource::kApplicaton, "test_event",
                                        42, 3.14, "hello");
  event.Print();

  auto name = event.GetName();
  const auto& data = event.GetData();
  auto a = std::get<0>(data);
  auto b = std::get<1>(data);
  auto c = std::get<2>(data);
  std::cout << "Event a = " << a << ", b = " << b << ", c = " << c << std::endl;

  AsyncEventDispatcher::GetInstance().RegisterHandler(
      "test_event", [](std::shared_ptr<BaseEvent> event) {
        auto data =
            std::static_pointer_cast<Event<int, double, std::string>>(event)
                ->GetData();
        auto a = std::get<0>(data);
        auto b = std::get<1>(data);
        auto c = std::get<2>(data);
        std::cout << "Handler 1: Received event: a = " << a << ", b = " << b
                  << ", c = " << c << std::endl;
      });

  AsyncEventEmitter emitter;
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_event", 42, 3.14, "hello");
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_event", 21, 6.28, "hello again");

  std::cout << std::endl;

  // ====================================================================
  // Test 1: Attempting HandleEvents test without Reset()
  std::cout << "Test 1: Attempting HandleEvents test without Reset()"
            << std::endl;
  std::thread handler_thread_1 =
      std::thread(&AsyncEventDispatcher::HandleEvents,
                  &AsyncEventDispatcher::GetInstance());
  std::cout << "handler_thread_1 id: " << handler_thread_1.get_id()
            << std::endl;
  handler_thread_1.join();

  // Create a dummy thread to guarantee that handler thread 2 gets a different
  // id from handler thread 1
  std::thread dummy_thread(
      []() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });

  // Create another handler thread
  // Without calling Reset(), this should throw an error
  std::thread handler_thread_2;
  handler_thread_2 = std::thread([&]() {
    try {
      AsyncEventDispatcher::GetInstance().HandleEvents();
    } catch (const std::runtime_error& e) {
      std::cerr << "Caught exception: " << e.what() << std::endl;
      std::cout << "Note: This exception is expected." << std::endl;
    }
  });

  std::cout << "handler_thread_2 id: " << handler_thread_2.get_id()
            << std::endl;
  handler_thread_2.join();
  dummy_thread.join();
  std::cout << "-----------------------------" << std::endl << std::endl;

  // ====================================================================
  // Test 2: Attempting HandleEvents test with Reset()
  std::cout << "Test 2: Attempting HandleEvents test with Reset()" << std::endl;

  // Emit another event
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_event", 101, 1.618, "hello");

  handler_thread_1 = std::thread(&AsyncEventDispatcher::HandleEvents,
                                 &AsyncEventDispatcher::GetInstance());
  std::cout << "handler_thread_1 id: " << handler_thread_1.get_id()
            << std::endl;
  // Note: AsyncEventDispatcher still has the handler previously registered
  handler_thread_1.join();

  // Create a dummy thread to guarantee that handler thread 2 gets a different
  // id from handler thread 1
  dummy_thread = std::thread(
      []() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });

  // This event will not be handled since it was emitted after the
  // HandleEvents() call and right before the Reset() call
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_ignored_event", 123, 1.618, "bye");

  // Reset the dispatcher
  // This should clear all handlers, unhandled events and reset any cached
  // thread ids.
  AsyncEventDispatcher::GetInstance().Reset();

  // Emit another event. This event will be handled since it was emitted after
  // the Reset() call
  emitter.Emit<Event<int, double, std::string>>(
      EventSource::kApplicaton, "test_event", 102, 1.618, "hello after reset");

  // Reset() clears the handlers, so we need to re-register them
  AsyncEventDispatcher::GetInstance().RegisterHandler(
      "test_event", [](std::shared_ptr<BaseEvent> event) {
        auto data =
            std::static_pointer_cast<Event<int, double, std::string>>(event)
                ->GetData();
        auto a = std::get<0>(data);
        auto b = std::get<1>(data);
        auto c = std::get<2>(data);
        std::cout << "Handler 2: Received event: a = " << a << ", b = " << b
                  << ", c = " << c << std::endl;
      });

  // Create another handler thread
  handler_thread_2 = std::thread(&AsyncEventDispatcher::HandleEvents,
                                 &AsyncEventDispatcher::GetInstance());
  std::cout << "handler_thread_2 id: " << handler_thread_2.get_id()
            << std::endl;
  handler_thread_2.join();
  dummy_thread.join();
  std::cout << "-----------------------------" << std::endl;

  return 0;
}
