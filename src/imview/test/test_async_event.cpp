/*
 * @file test_event.cpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/event/event.hpp"
#include "imview/event/async_event_dispatcher.hpp"
#include "imview/event/async_event_emitter.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Event<int, double, std::string> event("test_event", 42, 3.14, "hello");
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
        std::cout << "Received event: a = " << a << ", b = " << b
                  << ", c = " << c << std::endl;
      });

  AsyncEventEmitter emitter;
  emitter.Emit<Event<int, double, std::string>>("test_event", 42, 3.14,
                                                "hello");
  emitter.Emit<Event<int, double, std::string>>("test_event", 21, 6.28,
                                                "hello again");

  AsyncEventDispatcher::GetInstance().HandleEvents();

  return 0;
}
